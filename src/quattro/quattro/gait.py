"""
Reference-derived Bézier gait generation for Quattro.

The implementation retains the reference gait's 12-point swing curve, sine
stance curve, diagonal trot phasing, yaw-circle motion and touchdown phase
resynchronization. Public values use SI units and the module has no ROS
dependency. ``GaitGenerator.update_states`` extends the original
position-only ``update`` with analytic foot velocity/acceleration
(``FootTrajectoryState``), computed from closed-form Bezier/sine
derivatives rather than finite differences.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

from quattro.kinematics import LEG_NAMES, RobotGeometry


TROT_PHASE_OFFSETS = {
    'front_left': 0.0,
    'front_right': 0.5,
    'back_left': 0.5,
    'back_right': 0.0,
}


@dataclass(frozen=True)
class GaitParameters:
    """Reference gait parameters in SI units."""

    swing_duration: float = 0.25
    stance_duration: float = 0.30
    clearance_height: float = 0.040
    penetration_depth: float = 0.008
    max_linear_speed: float = 0.30
    max_yaw_rate: float = 1.0
    stop_return_speed: float = 0.08

    def __post_init__(self) -> None:
        if self.swing_duration <= 0.0 or self.stance_duration <= 0.0:
            raise ValueError('swing and stance durations must be positive')
        if self.stance_duration > 1.3 * self.swing_duration:
            raise ValueError('stance_duration must not exceed 1.3 * swing_duration')
        if self.clearance_height < 0.0 or self.penetration_depth < 0.0:
            raise ValueError('foot heights must not be negative')
        if (self.max_linear_speed <= 0.0 or self.max_yaw_rate <= 0.0 or
                self.stop_return_speed <= 0.0):
            raise ValueError('speed limits must be positive')


@dataclass(frozen=True)
class FootPhase:
    """Local phase information for one leg."""

    normalized: float
    in_swing: bool


@dataclass(frozen=True)
class FootTrajectoryState:
    """
    Foot trajectory target with analytic velocity and acceleration.

    Position is metres in the nominal (body) frame, matching the dict
    returned by ``GaitGenerator.update``. Velocity/acceleration are the
    time-derivatives of that same trajectory (m/s, m/s^2), obtained from the
    closed-form Bezier/sine derivatives -- not finite differences.
    """

    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    acceleration: NDArray[np.float64]
    phase: float
    in_swing: bool


class GaitGenerator:
    """Generate reference-style diagonal-trot foot trajectories."""

    _CONTROL_POINT_COUNT = 12

    def __init__(
        self,
        geometry: RobotGeometry = RobotGeometry(),
        parameters: GaitParameters = GaitParameters(),
    ) -> None:
        self.geometry = geometry
        self.parameters = parameters
        self._time = 0.0
        self._phase = 0.0
        self._previous_offsets = {
            name: np.zeros(3, dtype=float) for name in LEG_NAMES}
        self._nominal_feet = self._stance_positions()
        self._last_linear_velocity = np.zeros(2, dtype=float)
        self._last_yaw_rate = 0.0
        self._finishing_cycle = False

    @property
    def phase(self) -> float:
        """Return the normalized reference-leg gait phase."""
        return self._phase

    @property
    def finishing_cycle(self) -> bool:
        """Return whether a released command is completing its current cycle."""
        return self._finishing_cycle

    @property
    def stride_duration(self) -> float:
        """Return stance plus swing duration."""
        return self.parameters.stance_duration + self.parameters.swing_duration

    def reset(self, phase: float = 0.0) -> None:
        """Reset the gait clock and stored yaw offsets."""
        if not math.isfinite(phase):
            raise ValueError('phase must be finite')
        self._phase = phase % 1.0
        self._time = self._phase * self.stride_duration
        for offset in self._previous_offsets.values():
            offset.fill(0.0)

    def _stance_positions(self) -> dict[str, NDArray[np.float64]]:
        geometry = self.geometry
        return {
            'front_left': np.array([
                geometry.foot_spacing_x / 2.0,
                geometry.foot_spacing_y / 2.0,
                -geometry.nominal_height]),
            'front_right': np.array([
                geometry.foot_spacing_x / 2.0,
                -geometry.foot_spacing_y / 2.0,
                -geometry.nominal_height]),
            'back_left': np.array([
                -geometry.foot_spacing_x / 2.0,
                geometry.foot_spacing_y / 2.0,
                -geometry.nominal_height]),
            'back_right': np.array([
                -geometry.foot_spacing_x / 2.0,
                -geometry.foot_spacing_y / 2.0,
                -geometry.nominal_height]),
        }

    def leg_phase(self, leg_name: str) -> FootPhase:
        """Return reference stance/swing phase for one leg."""
        if leg_name not in TROT_PHASE_OFFSETS:
            raise ValueError(f'unknown leg: {leg_name}')
        cycle_phase = (self._phase + TROT_PHASE_OFFSETS[leg_name]) % 1.0
        stance_fraction = self.parameters.stance_duration / self.stride_duration
        if cycle_phase < stance_fraction:
            return FootPhase(cycle_phase / stance_fraction, False)
        swing_phase = ((cycle_phase - stance_fraction)
                       / (1.0 - stance_fraction))
        return FootPhase(swing_phase, True)

    @staticmethod
    def _bernstein(phase: float, control_points: NDArray[np.float64]) -> float:
        """Evaluate a Bernstein polynomial of degree len(control_points)-1."""
        degree = len(control_points) - 1
        return float(sum(
            point * math.comb(degree, index)
            * phase ** index * (1.0 - phase) ** (degree - index)
            for index, point in enumerate(control_points)
        ))

    @classmethod
    def _bernstein_with_derivatives(
        cls, phase: float, control_points: NDArray[np.float64],
    ) -> tuple[float, float, float]:
        """
        Return (value, d/ds, d^2/ds^2) of a Bernstein polynomial at s=phase.

        The derivative of a degree-n Bezier curve is itself a degree-(n-1)
        Bezier curve over forward-difference control points scaled by n
        (standard Bernstein-derivative identity), so both derivatives are
        analytic -- no finite differencing of the curve itself.
        """
        points = np.asarray(control_points, dtype=float)
        degree = len(points) - 1
        value = cls._bernstein(phase, points)
        if degree >= 1:
            first_points = np.diff(points) * degree
            first = cls._bernstein(phase, first_points)
        else:
            first = 0.0
        if degree >= 2:
            second_points = np.diff(first_points) * (degree - 1)
            second = cls._bernstein(phase, second_points)
        else:
            second = 0.0
        return value, first, second

    @classmethod
    def _bezier(cls, phase: float, control_points: NDArray[np.float64]) -> float:
        """Evaluate the reference degree-11 Bernstein polynomial."""
        return cls._bernstein(phase, control_points)

    @classmethod
    def _bezier_swing_control_points(
        cls, half_step: float, clearance_height: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        step = half_step * np.array([
            -1.0, -1.4, -1.5, -1.5, -1.5, 0.0,
            0.0, 0.0, 1.5, 1.5, 1.4, 1.0,
        ])
        vertical = clearance_height * np.array([
            0.0, 0.0, 0.9, 0.9, 0.9, 0.9,
            0.9, 1.1, 1.1, 1.1, 0.0, 0.0,
        ])
        return step, vertical

    @classmethod
    def bezier_swing(
        cls,
        phase: float,
        half_step: float,
        direction: float,
        clearance_height: float,
    ) -> NDArray[np.float64]:
        """Evaluate the reference 12-control-point swing trajectory."""
        position, _, _ = cls.bezier_swing_with_derivatives(
            phase, half_step, direction, clearance_height)
        return position

    @classmethod
    def bezier_swing_with_derivatives(
        cls,
        phase: float,
        half_step: float,
        direction: float,
        clearance_height: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Return (position, d/ds, d^2/ds^2) of the swing curve at s=phase.

        Derivatives are with respect to normalized phase s, not time --
        callers divide by the swing segment duration (and its square) to
        obtain m/s and m/s^2.
        """
        step, vertical = cls._bezier_swing_control_points(
            half_step, clearance_height)
        distance, d_distance, dd_distance = cls._bernstein_with_derivatives(
            phase, step)
        height, d_height, dd_height = cls._bernstein_with_derivatives(
            phase, vertical)
        cos_d, sin_d = math.cos(direction), math.sin(direction)
        position = np.array([distance * cos_d, distance * sin_d, height])
        velocity = np.array([d_distance * cos_d, d_distance * sin_d, d_height])
        acceleration = np.array(
            [dd_distance * cos_d, dd_distance * sin_d, dd_height])
        return position, velocity, acceleration

    @staticmethod
    def sine_stance(
        phase: float,
        half_step: float,
        direction: float,
        penetration_depth: float,
    ) -> NDArray[np.float64]:
        """Evaluate the reference sine stance trajectory."""
        position, _, _ = GaitGenerator.sine_stance_with_derivatives(
            phase, half_step, direction, penetration_depth)
        return position

    @staticmethod
    def sine_stance_with_derivatives(
        phase: float,
        half_step: float,
        direction: float,
        penetration_depth: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Return (position, d/ds, d^2/ds^2) of the stance curve at s=phase.

        Derivatives are with respect to normalized phase s (see
        ``bezier_swing_with_derivatives``); both are closed-form since the
        stance curve is an explicit sine/linear function of phase.
        """
        cos_d, sin_d = math.cos(direction), math.sin(direction)
        distance = half_step * (1.0 - 2.0 * phase)
        d_distance = -2.0 * half_step
        height = -penetration_depth * math.sin(math.pi * phase)
        d_height = -penetration_depth * math.pi * math.cos(math.pi * phase)
        dd_height = penetration_depth * math.pi ** 2 * math.sin(math.pi * phase)
        position = np.array([distance * cos_d, distance * sin_d, height])
        velocity = np.array([d_distance * cos_d, d_distance * sin_d, d_height])
        acceleration = np.array([0.0, 0.0, dd_height])
        return position, velocity, acceleration

    def _yaw_direction(self, leg_name: str) -> float:
        """Return the tangential yaw-circle direction for a foot."""
        nominal = self._nominal_feet[leg_name]
        radius = float(np.linalg.norm(nominal[:2]))
        base_direction = math.atan2(nominal[1], nominal[0])
        previous_magnitude = float(np.linalg.norm(
            self._previous_offsets[leg_name][:2]))
        phase_modifier = math.atan2(previous_magnitude, radius)
        if leg_name in ('front_right', 'back_left'):
            return math.pi / 2.0 + base_direction + phase_modifier
        return math.pi / 2.0 - base_direction + phase_modifier

    def _trajectory_offset_with_derivatives(
        self,
        leg_name: str,
        phase: FootPhase,
        linear_half_step: float,
        linear_direction: float,
        yaw_half_step: float,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """
        Return (offset, velocity, acceleration) for one leg's curve.

        velocity/acceleration are with respect to time: the phase-derivative
        (see ``bezier_swing_with_derivatives``) is scaled by the swing or
        stance segment duration, since normalized phase advances at
        1/segment_duration while that segment is active.
        """
        if phase.in_swing:
            curve = self.bezier_swing_with_derivatives
            height = self.parameters.clearance_height
            segment_duration = self.parameters.swing_duration
        else:
            curve = self.sine_stance_with_derivatives
            height = self.parameters.penetration_depth
            segment_duration = self.parameters.stance_duration
        linear_pos, linear_vel, linear_acc = curve(
            phase.normalized, linear_half_step, linear_direction, height)
        rotational_pos, rotational_vel, rotational_acc = curve(
            phase.normalized, yaw_half_step,
            self._yaw_direction(leg_name), height)
        offset = linear_pos + rotational_pos
        velocity = (linear_vel + rotational_vel) / segment_duration
        acceleration = (linear_acc + rotational_acc) / (segment_duration ** 2)
        self._previous_offsets[leg_name] = offset
        return offset, velocity, acceleration

    def _trajectory_offset(
        self,
        leg_name: str,
        phase: FootPhase,
        linear_half_step: float,
        linear_direction: float,
        yaw_half_step: float,
    ) -> NDArray[np.float64]:
        offset, _, _ = self._trajectory_offset_with_derivatives(
            leg_name, phase, linear_half_step, linear_direction, yaw_half_step)
        return offset

    def update(
        self,
        dt: float,
        linear_velocity: Sequence[float] = (0.0, 0.0),
        yaw_rate: float = 0.0,
        contacts: Mapping[str, bool] | None = None,
        complete_cycle_on_stop: bool = True,
    ) -> dict[str, NDArray[np.float64]]:
        """
        Advance the reference gait and return nominal-frame foot targets.

        Linear velocity is metres per second and yaw rate is radians per
        second. Touchdown of the front-left reference leg near the end of
        swing resynchronizes the gait clock, matching the reference behavior.

        This is a thin wrapper over ``update_states`` for backward
        compatibility; prefer ``update_states`` when velocity/acceleration
        are also needed, since it computes the same trajectory once.
        """
        states = self.update_states(
            dt, linear_velocity, yaw_rate, contacts, complete_cycle_on_stop)
        return {name: state.position for name, state in states.items()}

    def update_states(
        self,
        dt: float,
        linear_velocity: Sequence[float] = (0.0, 0.0),
        yaw_rate: float = 0.0,
        contacts: Mapping[str, bool] | None = None,
        complete_cycle_on_stop: bool = True,
    ) -> dict[str, FootTrajectoryState]:
        """
        Advance the reference gait and return full foot trajectory states.

        Same arguments and gait-clock behavior as ``update``, but returns
        analytic velocity/acceleration alongside position for every leg
        instead of position alone.
        """
        velocity = np.asarray(linear_velocity, dtype=float)
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError('dt must be finite and non-negative')
        if velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
            raise ValueError('linear_velocity must contain two finite values')
        if not math.isfinite(yaw_rate):
            raise ValueError('yaw_rate must be finite')
        speed = float(np.linalg.norm(velocity))
        if speed > self.parameters.max_linear_speed + 1.0e-12:
            raise ValueError('linear velocity exceeds max_linear_speed')
        if abs(yaw_rate) > self.parameters.max_yaw_rate + 1.0e-12:
            raise ValueError('yaw rate exceeds max_yaw_rate')
        contact_state = contacts or {name: False for name in LEG_NAMES}
        if set(contact_state) != set(LEG_NAMES):
            raise ValueError(f'contacts must contain {LEG_NAMES}')

        moving = speed > 1.0e-6 or abs(yaw_rate) > 1.0e-6
        if moving:
            self._last_linear_velocity = velocity.copy()
            self._last_yaw_rate = yaw_rate
            self._finishing_cycle = False
        elif complete_cycle_on_stop and (
                np.linalg.norm(self._last_linear_velocity) > 1.0e-6 or
                abs(self._last_yaw_rate) > 1.0e-6):
            self._finishing_cycle = True
            velocity = self._last_linear_velocity.copy()
            yaw_rate = self._last_yaw_rate
            speed = float(np.linalg.norm(velocity))
            moving = True
        else:
            self._finishing_cycle = False
            self._last_linear_velocity.fill(0.0)
            self._last_yaw_rate = 0.0

        if not moving:
            states = {}
            return_step = self.parameters.stop_return_speed * dt
            all_nominal = True
            for name in LEG_NAMES:
                offset = self._previous_offsets[name]
                offset_before = offset.copy()
                distance = float(np.linalg.norm(offset))
                if distance <= return_step:
                    offset.fill(0.0)
                elif distance > 0.0:
                    offset *= (distance - return_step) / distance
                    all_nominal = False
                # Rate-limited ease-back to nominal stance: the velocity
                # command is the exact applied step rate, not a finite
                # difference of an external trajectory function.
                velocity = ((offset - offset_before) / dt
                            if dt > 0.0 else np.zeros(3))
                states[name] = FootTrajectoryState(
                    position=self._nominal_feet[name] + offset.copy(),
                    velocity=velocity,
                    acceleration=np.zeros(3),
                    phase=0.0,
                    in_swing=False,
                )
            if all_nominal:
                self._time = 0.0
                self._phase = 0.0
            return states

        cycle_finished = False
        if self._finishing_cycle:
            remaining = self.stride_duration - self._time
            self._time += min(dt, remaining)
            cycle_finished = self._time >= self.stride_duration - 1.0e-12
            self._phase = (self._time / self.stride_duration) % 1.0
        else:
            reference_phase = self.leg_phase('front_left')
            touchdown = (
                reference_phase.in_swing
                and reference_phase.normalized >= 0.9
                and bool(contact_state['front_left'])
            )
            if touchdown:
                self.reset()
            else:
                self._time = (self._time + dt) % self.stride_duration
                self._phase = self._time / self.stride_duration

        linear_half_step = 0.5 * speed * self.parameters.stance_duration
        linear_direction = math.atan2(velocity[1], velocity[0]) if speed else 0.0
        mean_radius = 0.5 * math.hypot(
            self.geometry.foot_spacing_x, self.geometry.foot_spacing_y)
        yaw_half_step = (
            0.5 * yaw_rate * self.parameters.stance_duration * mean_radius)

        states = {}
        for name in LEG_NAMES:
            leg_phase = self.leg_phase(name)
            offset, velocity, acceleration = self._trajectory_offset_with_derivatives(
                name, leg_phase, linear_half_step, linear_direction, yaw_half_step)
            states[name] = FootTrajectoryState(
                position=self._nominal_feet[name] + offset,
                velocity=velocity,
                acceleration=acceleration,
                phase=leg_phase.normalized,
                in_swing=leg_phase.in_swing,
            )
        if cycle_finished:
            self._time = 0.0
            self._phase = 0.0
            self._last_linear_velocity.fill(0.0)
            self._last_yaw_rate = 0.0
            self._finishing_cycle = False
        return states
