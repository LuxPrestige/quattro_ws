"""
Inverse kinematics for the Quattro robot.

All public lengths and angles use SI units.  This module deliberately has no
ROS dependency so it can be reused by simulation and unit tests.
"""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray


LEG_NAMES = ('front_left', 'front_right', 'back_left', 'back_right')
JOINT_SUFFIXES = ('hip_joint', 'upper_leg_joint', 'lower_leg_joint')


class UnreachableTargetError(ValueError):
    """Raised when a requested foot position is outside the workspace."""


@dataclass(frozen=True)
class RobotGeometry:
    """Dimensions of the robot's nominal stance in metres."""

    hip_offset_y: float = 0.0905
    upper_leg_length: float = 0.210
    lower_leg_length: float = 0.210
    hip_spacing_x: float = 0.405
    hip_spacing_y: float = 0.120
    foot_spacing_x: float = 0.405
    foot_spacing_y: float = 0.301
    nominal_height: float = 0.300
    center_of_mass_offset_x: float = -0.040

    def __post_init__(self) -> None:
        positive = (
            self.hip_offset_y,
            self.upper_leg_length,
            self.lower_leg_length,
            self.hip_spacing_x,
            self.hip_spacing_y,
            self.foot_spacing_x,
            self.foot_spacing_y,
            self.nominal_height,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError('robot dimensions must be positive')


def rotation_matrix_from_rpy(
    roll: float, pitch: float, yaw: float,
) -> NDArray[np.float64]:
    """Return a body rotation matrix for fixed-axis roll, pitch and yaw."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


class LegKinematics:
    """Analytic inverse kinematics for one three-joint leg."""

    def __init__(
        self, hip_offset_y: float, upper_length: float,
        lower_length: float, is_left: bool,
    ) -> None:
        self.hip_offset_y = hip_offset_y
        self.upper_length = upper_length
        self.lower_length = lower_length
        self.is_left = is_left

    def inverse(self, foot: Sequence[float]) -> NDArray[np.float64]:
        """Solve a hip-frame foot target and return three joint angles."""
        target = np.asarray(foot, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('foot target must contain three finite values')
        x, y, z = target
        offset = self.hip_offset_y
        radial_squared = y * y + z * z - offset * offset
        if radial_squared < -1.0e-12:
            raise UnreachableTargetError('foot is inside the hip-offset workspace')
        radial = math.sqrt(max(0.0, radial_squared))
        cosine_knee = (
            x * x + radial * radial - self.upper_length ** 2
            - self.lower_length ** 2
        ) / (2.0 * self.upper_length * self.lower_length)
        if cosine_knee < -1.0 - 1.0e-9 or cosine_knee > 1.0 + 1.0e-9:
            raise UnreachableTargetError('foot is beyond the leg workspace')
        cosine_knee = min(1.0, max(-1.0, cosine_knee))
        knee = math.atan2(-math.sqrt(max(0.0, 1.0 - cosine_knee ** 2)),
                          cosine_knee)
        offset_sign = offset if self.is_left else -offset
        hip_intermediate = -math.atan2(z, y) - math.atan2(radial, offset_sign)
        hip = -hip_intermediate
        upper = math.atan2(-x, radial) - math.atan2(
            self.lower_length * math.sin(knee),
            self.upper_length + self.lower_length * math.cos(knee),
        )
        return np.array([hip, upper, knee], dtype=float)

    def forward(self, joints: Sequence[float]) -> NDArray[np.float64]:
        """Return the hip-frame foot position for three joint angles."""
        angles = np.asarray(joints, dtype=float)
        if angles.shape != (3,) or not np.all(np.isfinite(angles)):
            raise ValueError('joint angles must contain three finite values')
        hip, upper, knee = angles
        planar_x = -(
            self.upper_length * math.sin(upper)
            + self.lower_length * math.sin(upper + knee)
        )
        radial = (
            self.upper_length * math.cos(upper)
            + self.lower_length * math.cos(upper + knee)
        )
        side = 1.0 if self.is_left else -1.0
        y = side * self.hip_offset_y * math.cos(hip) + radial * math.sin(hip)
        z = side * self.hip_offset_y * math.sin(hip) - radial * math.cos(hip)
        return np.array([planar_x, y, z], dtype=float)

    def jacobian(self, joints: Sequence[float]) -> NDArray[np.float64]:
        """
        Return the analytic 3x3 hip-frame Jacobian d(foot)/d(joints).

        Columns follow joint order [hip, upper, knee]; rows follow the
        hip-frame foot position order returned by ``forward`` ([x, y, z]).
        Differentiates the closed-form ``forward`` expressions directly
        (no numerical differencing).
        """
        angles = np.asarray(joints, dtype=float)
        if angles.shape != (3,) or not np.all(np.isfinite(angles)):
            raise ValueError('joint angles must contain three finite values')
        hip, upper, knee = angles
        side = 1.0 if self.is_left else -1.0
        radial = (
            self.upper_length * math.cos(upper)
            + self.lower_length * math.cos(upper + knee)
        )
        d_radial_d_upper = -(
            self.upper_length * math.sin(upper)
            + self.lower_length * math.sin(upper + knee)
        )
        d_radial_d_knee = -self.lower_length * math.sin(upper + knee)
        d_planar_x_d_upper = -radial
        d_planar_x_d_knee = -self.lower_length * math.cos(upper + knee)
        sin_h, cos_h = math.sin(hip), math.cos(hip)
        offset = side * self.hip_offset_y
        return np.array([
            [0.0, d_planar_x_d_upper, d_planar_x_d_knee],
            [-offset * sin_h + radial * cos_h,
             d_radial_d_upper * sin_h, d_radial_d_knee * sin_h],
            [offset * cos_h + radial * sin_h,
             -d_radial_d_upper * cos_h, -d_radial_d_knee * cos_h],
        ], dtype=float)

    def foot_velocity(
        self, joints: Sequence[float], joint_velocities: Sequence[float],
    ) -> NDArray[np.float64]:
        """Return hip-frame foot velocity v = J(q) * qdot."""
        rates = np.asarray(joint_velocities, dtype=float)
        if rates.shape != (3,) or not np.all(np.isfinite(rates)):
            raise ValueError('joint velocities must contain three finite values')
        return self.jacobian(joints) @ rates

    def joint_velocity_from_foot_velocity(
        self,
        joints: Sequence[float],
        foot_velocity: Sequence[float],
        damping: float = 1.0e-3,
    ) -> NDArray[np.float64]:
        """
        Return joint velocities for a desired hip-frame foot velocity.

        Uses the damped least-squares (Levenberg-Marquardt) pseudoinverse
        qdot = J^T (J J^T + damping^2 I)^-1 v instead of a plain inverse, so
        the result stays bounded near Jacobian singularities instead of
        diverging.
        """
        velocity = np.asarray(foot_velocity, dtype=float)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError('foot velocity must contain three finite values')
        if not math.isfinite(damping) or damping < 0.0:
            raise ValueError('damping must be finite and non-negative')
        jacobian = self.jacobian(joints)
        gram = jacobian @ jacobian.T + (damping ** 2) * np.eye(3)
        return jacobian.T @ np.linalg.solve(gram, velocity)

    def force_to_joint_torque(
        self, joints: Sequence[float], force: Sequence[float],
    ) -> NDArray[np.float64]:
        """Return joint torque tau = J(q)^T * F for a hip-frame foot force."""
        applied_force = np.asarray(force, dtype=float)
        if applied_force.shape != (3,) or not np.all(np.isfinite(applied_force)):
            raise ValueError('force must contain three finite values')
        return self.jacobian(joints).T @ applied_force


class QuadrupedKinematics:
    """Forward and inverse kinematics for the four Quattro legs."""

    def __init__(self, geometry: RobotGeometry = RobotGeometry()) -> None:
        self.geometry = geometry
        self._hip_positions = self._stance_positions(
            geometry.hip_spacing_x, geometry.hip_spacing_y, 0.0)
        self.nominal_foot_positions = self._stance_positions(
            geometry.foot_spacing_x, geometry.foot_spacing_y,
            -geometry.nominal_height)
        self._legs = {
            name: LegKinematics(
                geometry.hip_offset_y, geometry.upper_leg_length,
                geometry.lower_leg_length, 'left' in name)
            for name in LEG_NAMES
        }

    @staticmethod
    def _stance_positions(
        spacing_x: float, spacing_y: float, z: float,
    ) -> dict[str, NDArray[np.float64]]:
        return {
            'front_left': np.array([spacing_x / 2, spacing_y / 2, z]),
            'front_right': np.array([spacing_x / 2, -spacing_y / 2, z]),
            'back_left': np.array([-spacing_x / 2, spacing_y / 2, z]),
            'back_right': np.array([-spacing_x / 2, -spacing_y / 2, z]),
        }

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return the canonical front/back joint ordering."""
        return tuple(
            f'{leg}_{suffix}' for leg in LEG_NAMES for suffix in JOINT_SUFFIXES)

    def inverse(
        self,
        rpy: Sequence[float] = (0.0, 0.0, 0.0),
        translation: Sequence[float] = (0.0, 0.0, 0.0),
        foot_positions: Mapping[str, Sequence[float]] | None = None,
    ) -> NDArray[np.float64]:
        """Return a 4x3 joint matrix for a body pose and world-frame feet."""
        angles = np.asarray(rpy, dtype=float)
        position = np.asarray(translation, dtype=float).copy()
        if angles.shape != (3,) or position.shape != (3,):
            raise ValueError('rpy and translation must each contain three values')
        if not np.all(np.isfinite(angles)) or not np.all(np.isfinite(position)):
            raise ValueError('body pose must contain finite values')
        position[0] += self.geometry.center_of_mass_offset_x
        rotation = rotation_matrix_from_rpy(*angles)
        feet = (
            self.nominal_foot_positions
            if foot_positions is None else foot_positions
        )
        if set(feet) != set(LEG_NAMES):
            raise ValueError(f'foot positions must contain {LEG_NAMES}')

        result = np.empty((4, 3), dtype=float)
        for index, name in enumerate(LEG_NAMES):
            foot_world = np.asarray(feet[name], dtype=float)
            hip_body = self._hip_positions[name]
            foot_in_body = rotation.T @ (foot_world - position)
            result[index] = self._legs[name].inverse(foot_in_body - hip_body)
        return result

    def forward(
        self,
        joint_angles: Sequence[Sequence[float]],
        rpy: Sequence[float] = (0.0, 0.0, 0.0),
        translation: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> dict[str, NDArray[np.float64]]:
        """Return world-frame foot positions for a body pose and joints."""
        joints = np.asarray(joint_angles, dtype=float)
        angles = np.asarray(rpy, dtype=float)
        position = np.asarray(translation, dtype=float).copy()
        if joints.shape != (4, 3) or not np.all(np.isfinite(joints)):
            raise ValueError('joint_angles must be a finite 4x3 array')
        if angles.shape != (3,) or position.shape != (3,):
            raise ValueError('rpy and translation must each contain three values')
        if not np.all(np.isfinite(angles)) or not np.all(np.isfinite(position)):
            raise ValueError('body pose must contain finite values')
        position[0] += self.geometry.center_of_mass_offset_x
        rotation = rotation_matrix_from_rpy(*angles)
        feet = {}
        for index, name in enumerate(LEG_NAMES):
            foot_body = self._hip_positions[name] + self._legs[name].forward(
                joints[index])
            feet[name] = position + rotation @ foot_body
        return feet

    def joint_velocities(
        self,
        rpy: Sequence[float],
        joint_angles: NDArray[np.float64],
        foot_velocities: Mapping[str, Sequence[float]],
        damping: float = 1.0e-3,
    ) -> NDArray[np.float64]:
        """
        Return a 4x3 joint velocity matrix for world-frame foot velocities.

        ``joint_angles`` are the operating-point joint angles (typically the
        IK solution for the same command) used to evaluate each leg's
        Jacobian. ``foot_velocities`` use the same world/body-aligned frame
        as the ``foot_positions`` argument to ``inverse``; only the body
        rotation (not its rate) is used to express them in each hip frame,
        matching the quasi-static assumption already used by ``inverse``.
        Uses the damped least-squares pseudoinverse per leg (see
        ``LegKinematics.joint_velocity_from_foot_velocity``) so results stay
        bounded near IK singularities.
        """
        angles = np.asarray(rpy, dtype=float)
        joints = np.asarray(joint_angles, dtype=float)
        if angles.shape != (3,) or not np.all(np.isfinite(angles)):
            raise ValueError('rpy must contain three finite values')
        if joints.shape != (4, 3) or not np.all(np.isfinite(joints)):
            raise ValueError('joint_angles must be a finite 4x3 array')
        if set(foot_velocities) != set(LEG_NAMES):
            raise ValueError(f'foot velocities must contain {LEG_NAMES}')
        rotation = rotation_matrix_from_rpy(*angles)
        result = np.empty((4, 3), dtype=float)
        for index, name in enumerate(LEG_NAMES):
            velocity_body = np.asarray(foot_velocities[name], dtype=float)
            if velocity_body.shape != (3,) or not np.all(np.isfinite(velocity_body)):
                raise ValueError('foot velocities must contain three finite values')
            velocity_hip = rotation.T @ velocity_body
            result[index] = self._legs[name].joint_velocity_from_foot_velocity(
                joints[index], velocity_hip, damping)
        return result
