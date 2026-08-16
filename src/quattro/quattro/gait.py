"""
Reference-derived Bézier gait generation for Quattro.

The implementation retains the reference gait's 12-point swing curve, sine
stance curve, diagonal trot phasing, yaw-circle motion and touchdown phase
resynchronization. Public values use SI units and the module has no ROS
dependency.
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

    def __post_init__(self) -> None:
        if self.swing_duration <= 0.0 or self.stance_duration <= 0.0:
            raise ValueError('swing and stance durations must be positive')
        if self.stance_duration > 1.3 * self.swing_duration:
            raise ValueError('stance_duration must not exceed 1.3 * swing_duration')
        if self.clearance_height < 0.0 or self.penetration_depth < 0.0:
            raise ValueError('foot heights must not be negative')
        if self.max_linear_speed <= 0.0 or self.max_yaw_rate <= 0.0:
            raise ValueError('speed limits must be positive')


@dataclass(frozen=True)
class FootPhase:
    """Local phase information for one leg."""

    normalized: float
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

    @property
    def phase(self) -> float:
        """Return the normalized reference-leg gait phase."""
        return self._phase

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

    @classmethod
    def _bezier(cls, phase: float, control_points: NDArray[np.float64]) -> float:
        """Evaluate the reference degree-11 Bernstein polynomial."""
        degree = cls._CONTROL_POINT_COUNT - 1
        return float(sum(
            point * math.comb(degree, index)
            * phase ** index * (1.0 - phase) ** (degree - index)
            for index, point in enumerate(control_points)
        ))

    @classmethod
    def bezier_swing(
        cls,
        phase: float,
        half_step: float,
        direction: float,
        clearance_height: float,
    ) -> NDArray[np.float64]:
        """Evaluate the reference 12-control-point swing trajectory."""
        step = half_step * np.array([
            -1.0, -1.4, -1.5, -1.5, -1.5, 0.0,
            0.0, 0.0, 1.5, 1.5, 1.4, 1.0,
        ])
        vertical = clearance_height * np.array([
            0.0, 0.0, 0.9, 0.9, 0.9, 0.9,
            0.9, 1.1, 1.1, 1.1, 0.0, 0.0,
        ])
        distance = cls._bezier(phase, step)
        return np.array([
            distance * math.cos(direction),
            distance * math.sin(direction),
            cls._bezier(phase, vertical),
        ])

    @staticmethod
    def sine_stance(
        phase: float,
        half_step: float,
        direction: float,
        penetration_depth: float,
    ) -> NDArray[np.float64]:
        """Evaluate the reference sine stance trajectory."""
        distance = half_step * (1.0 - 2.0 * phase)
        return np.array([
            distance * math.cos(direction),
            distance * math.sin(direction),
            -penetration_depth * math.sin(math.pi * phase),
        ])

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

    def _trajectory_offset(
        self,
        leg_name: str,
        phase: FootPhase,
        linear_half_step: float,
        linear_direction: float,
        yaw_half_step: float,
    ) -> NDArray[np.float64]:
        curve = self.bezier_swing if phase.in_swing else self.sine_stance
        height = (self.parameters.clearance_height if phase.in_swing
                  else self.parameters.penetration_depth)
        linear = curve(
            phase.normalized, linear_half_step, linear_direction, height)
        rotational = curve(
            phase.normalized, yaw_half_step,
            self._yaw_direction(leg_name), height)
        offset = linear + rotational
        self._previous_offsets[leg_name] = offset
        return offset

    def update(
        self,
        dt: float,
        linear_velocity: Sequence[float] = (0.0, 0.0),
        yaw_rate: float = 0.0,
        contacts: Mapping[str, bool] | None = None,
    ) -> dict[str, NDArray[np.float64]]:
        """
        Advance the reference gait and return nominal-frame foot targets.

        Linear velocity is metres per second and yaw rate is radians per
        second. Touchdown of the front-left reference leg near the end of
        swing resynchronizes the gait clock, matching the reference behavior.
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
        if not moving:
            self.reset()
            return {name: foot.copy() for name, foot in self._nominal_feet.items()}

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

        targets = {}
        for name in LEG_NAMES:
            targets[name] = self._nominal_feet[name] + self._trajectory_offset(
                name, self.leg_phase(name), linear_half_step,
                linear_direction, yaw_half_step)
        return targets
