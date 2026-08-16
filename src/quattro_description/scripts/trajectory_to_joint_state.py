#!/usr/bin/env python3
"""Convert commanded trajectories to joint states for visualization only."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class TrajectoryToJointState(Node):
    """Expose trajectory positions to robot_state_publisher for RViz."""

    def __init__(self) -> None:
        super().__init__('trajectory_to_joint_state')
        self._publisher = self.create_publisher(JointState, 'joint_states', 10)
        self._subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory',
            self._on_trajectory,
            10,
        )
        self.get_logger().warning(
            'Publishing commanded positions as joint_states for visualization; '
            'this is not hardware feedback')

    def _on_trajectory(self, message: JointTrajectory) -> None:
        if not message.points:
            self.get_logger().warning('Ignoring trajectory without points')
            return
        positions = message.points[0].positions
        if len(message.joint_names) != len(positions):
            self.get_logger().error(
                'Ignoring trajectory with mismatched names and positions')
            return
        if not all(math.isfinite(position) for position in positions):
            self.get_logger().error('Ignoring trajectory with non-finite positions')
            return
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = list(message.joint_names)
        state.position = list(positions)
        self._publisher.publish(state)


def main(args=None) -> None:
    """Run the visualization-only trajectory bridge."""
    rclpy.init(args=args)
    node = TrajectoryToJointState()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
