"""Timeout-based keyboard teleoperation for desktop development."""

import select
import sys
import termios
import tty
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


HELP = """
Quattro keyboard control
------------------------
W/S: forward/backward    A/D: left/right
Q/E: turn left/right     Space: stop
Ctrl-C: quit

Commands automatically stop when key input becomes stale.
"""


def command_for_key(
    key: str, linear_speed: float, lateral_speed: float, yaw_rate: float,
) -> Optional[Twist]:
    """Map one key to a velocity command, or return None if unsupported."""
    command = Twist()
    normalized = key.lower()
    if normalized == 'w':
        command.linear.x = linear_speed
    elif normalized == 's':
        command.linear.x = -linear_speed
    elif normalized == 'a':
        command.linear.y = lateral_speed
    elif normalized == 'd':
        command.linear.y = -lateral_speed
    elif normalized == 'q':
        command.angular.z = yaw_rate
    elif normalized == 'e':
        command.angular.z = -yaw_rate
    elif key != ' ':
        return None
    return command


class KeyboardTeleop(Node):
    """Publish short-lived velocity commands from terminal key presses."""

    def __init__(self) -> None:
        super().__init__('quattro_keyboard_teleop')
        self._linear_speed = float(self.declare_parameter(
            'linear_speed', 0.10).value)
        self._lateral_speed = float(self.declare_parameter(
            'lateral_speed', 0.08).value)
        self._yaw_rate = float(self.declare_parameter('yaw_rate', 0.40).value)
        self._key_timeout = float(self.declare_parameter(
            'key_timeout', 0.25).value)
        self._publish_rate = float(self.declare_parameter(
            'publish_rate', 20.0).value)
        if min(self._linear_speed, self._lateral_speed, self._yaw_rate) < 0.0:
            raise ValueError('keyboard speeds must not be negative')
        if self._key_timeout <= 0.0 or self._publish_rate <= 0.0:
            raise ValueError('timeouts and rates must be positive')
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_teleop requires an interactive terminal')

        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._command = Twist()
        self._last_key_time = None
        self._terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.create_timer(1.0 / self._publish_rate, self._update)
        self.get_logger().info(HELP)

    def restore_terminal(self) -> None:
        """Restore terminal settings changed for non-blocking key input."""
        termios.tcsetattr(
            sys.stdin, termios.TCSADRAIN, self._terminal_settings)

    def _update(self) -> None:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if readable:
            key = sys.stdin.read(1)
            command = command_for_key(
                key, self._linear_speed, self._lateral_speed, self._yaw_rate)
            if command is not None:
                self._command = command
                self._last_key_time = self.get_clock().now()

        output = Twist()
        if self._last_key_time is not None:
            age = (self.get_clock().now() - self._last_key_time).nanoseconds / 1e9
            if age <= self._key_timeout:
                output = self._command
        self._publisher.publish(output)


def main(args=None) -> None:
    """Run keyboard teleoperation in an interactive terminal."""
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
