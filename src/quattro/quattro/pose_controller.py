"""ROS node that converts a requested body pose to a joint trajectory."""

import math

from geometry_msgs.msg import PoseStamped
from quattro.kinematics import (
    QuadrupedKinematics,
    RobotGeometry,
    UnreachableTargetError,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, ...]:
    """Convert a quaternion to roll, pitch and yaw."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        raise ValueError('orientation quaternion must be non-zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class PoseController(Node):
    """Generate stationary joint targets while keeping nominal feet fixed."""

    def __init__(self) -> None:
        super().__init__('pose_controller')
        defaults = RobotGeometry()
        values = {}
        for name in defaults.__dataclass_fields__:
            values[name] = float(self.declare_parameter(
                name, getattr(defaults, name)).value)
        self._model = QuadrupedKinematics(RobotGeometry(**values))
        self._trajectory_duration = float(self.declare_parameter(
            'trajectory_duration', 0.25).value)
        if self._trajectory_duration <= 0.0:
            raise ValueError('trajectory_duration must be positive')
        self._publisher = self.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self._subscription = self.create_subscription(
            PoseStamped, 'body_pose', self._on_pose, 10)

    def _on_pose(self, message: PoseStamped) -> None:
        pose = message.pose
        try:
            rpy = quaternion_to_rpy(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            positions = self._model.inverse(
                rpy, (pose.position.x, pose.position.y, pose.position.z))
        except (ValueError, UnreachableTargetError) as error:
            self.get_logger().error(f'Rejected body pose: {error}')
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self._model.joint_names)
        point = JointTrajectoryPoint()
        point.positions = positions.reshape(-1).tolist()
        seconds = int(self._trajectory_duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            (self._trajectory_duration - seconds) * 1.0e9)
        trajectory.points = [point]
        self._publisher.publish(trajectory)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
