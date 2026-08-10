"""Safe Joy-to-Twist teleoperation node for Quattro."""

import math
from typing import List

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply a continuous joystick deadzone and rescale the remainder."""
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def axis_value(axes: List[float], index: int, deadzone: float) -> float:
    """Read a validated joystick axis."""
    if index < 0 or index >= len(axes):
        return 0.0
    return apply_deadzone(float(axes[index]), deadzone)


class TeleopNode(Node):
    """Publish velocity commands only while a deadman button is held."""

    def __init__(self) -> None:
        super().__init__('quattro_teleop')
        defaults = {
            'axis_linear_x': 1,
            'axis_linear_y': 0,
            'axis_angular_z': 2,
            'scale_linear_x': 0.5,
            'scale_linear_y': 0.3,
            'scale_angular_z': 1.0,
            'invert_linear_x': False,
            'invert_linear_y': True,
            'invert_angular_z': True,
            'deadzone': 0.10,
            'enable_button': 9,
            'estop_button': 5,
            'joy_timeout_sec': 0.5,
            'publish_rate_hz': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._axis_indices = [int(self.get_parameter(name).value) for name in (
            'axis_linear_x', 'axis_linear_y', 'axis_angular_z')]
        self._scales = [float(self.get_parameter(name).value) for name in (
            'scale_linear_x', 'scale_linear_y', 'scale_angular_z')]
        self._signs = [
            -1.0 if bool(self.get_parameter(name).value) else 1.0
            for name in ('invert_linear_x', 'invert_linear_y',
                         'invert_angular_z')]
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._enable_button = int(self.get_parameter('enable_button').value)
        self._estop_button = int(self.get_parameter('estop_button').value)
        self._timeout = float(self.get_parameter('joy_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError('deadzone must be in [0.0, 1.0)')
        if self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')

        self._last_joy_time = None
        self._command = Twist()
        self._enabled = False
        self._estop_latched = False
        self._last_estop_pressed = False
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.create_service(
            Trigger, '/teleop/clear_estop', self._clear_estop_callback)
        self.create_timer(1.0 / rate, self._publish_command)
        self.get_logger().info(
            'Teleop ready; hold the enable button to publish motion')

    @staticmethod
    def _button_pressed(buttons: List[int], index: int) -> bool:
        return 0 <= index < len(buttons) and bool(buttons[index])

    def _joy_callback(self, message: Joy) -> None:
        self._last_joy_time = self.get_clock().now()
        estop_pressed = self._button_pressed(
            message.buttons, self._estop_button)
        if estop_pressed and not self._last_estop_pressed:
            self._estop_latched = True
            self.get_logger().error('Software E-stop latched')
        self._last_estop_pressed = estop_pressed
        self._enabled = self._button_pressed(
            message.buttons, self._enable_button)

        values = [axis_value(message.axes, index, self._deadzone)
                  for index in self._axis_indices]
        self._command.linear.x = values[0] * self._scales[0] * self._signs[0]
        self._command.linear.y = values[1] * self._scales[1] * self._signs[1]
        self._command.angular.z = values[2] * self._scales[2] * self._signs[2]

    def _publish_command(self) -> None:
        command = Twist()
        fresh = False
        if self._last_joy_time is not None:
            age = (self.get_clock().now() - self._last_joy_time).nanoseconds
            fresh = age / 1e9 <= self._timeout
        if fresh and self._enabled and not self._estop_latched:
            command = self._command
        self._publisher.publish(command)

    def _clear_estop_callback(self, request, response):
        del request
        if self._last_estop_pressed:
            response.success = False
            response.message = 'Release the E-stop button before clearing'
            return response
        self._estop_latched = False
        response.success = True
        response.message = 'Software E-stop cleared'
        self.get_logger().warning(response.message)
        return response


def main(args=None) -> None:
    """Run the joystick teleoperation node."""
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
