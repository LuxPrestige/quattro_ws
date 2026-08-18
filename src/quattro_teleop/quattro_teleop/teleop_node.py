"""Reference-mapped Switch Pro teleoperation using standard ROS messages."""

import math
from typing import List

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool


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
    """Implement the reference controller mapping with standard messages."""

    def __init__(self) -> None:
        super().__init__('quattro_teleop')
        defaults = {
            'axis_linear_x': 1,
            'axis_linear_y': 0,
            'axis_height': 3,
            'axis_yaw': 2,
            'axis_dpad_vertical': 7,
            'axis_dpad_horizontal': 6,
            'scale_linear_x': 0.30,
            'scale_linear_y': 0.30,
            'scale_yaw': 1.0,
            'scale_height': 0.15,
            'scale_roll_pitch': 0.785,
            'button_switch_mode': 0,
            'button_estop': 1,
            'button_imu_auto': 3,
            'button_left_bumper': 4,
            'button_right_bumper': 5,
            'deadzone': 0.10,
            'joy_timeout_sec': 0.5,
            'publish_rate_hz': 20.0,
            'clearance_height': 0.040,
            'penetration_depth': 0.008,
            'start_stepping': True,
            'swing_duration': 0.25,
            'adjustment_step': 0.001,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._parameters = {
            name: self.get_parameter(name).value for name in defaults}
        self._deadzone = float(self._parameters['deadzone'])
        self._timeout = float(self._parameters['joy_timeout_sec'])
        rate = float(self._parameters['publish_rate_hz'])
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError('deadzone must be in [0.0, 1.0)')
        if self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')

        self._last_joy_time = None
        self._last_buttons: List[int] = []
        self._stepping = bool(self._parameters['start_stepping'])
        self._estop = False
        self._imu_auto = False
        self._twist = Twist()
        self._pose = PoseStamped()
        self._clearance = float(self._parameters['clearance_height'])
        self._penetration = float(self._parameters['penetration_depth'])
        self._swing_duration = float(self._parameters['swing_duration'])

        self._cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pose_publisher = self.create_publisher(
            PoseStamped, '/body_pose', 10)
        self._estop_publisher = self.create_publisher(Bool, '/estop', 10)
        self._imu_publisher = self.create_publisher(Bool, '/imu_auto', 10)
        self._clearance_publisher = self.create_publisher(
            Float64, '/gait/clearance_height', 10)
        self._penetration_publisher = self.create_publisher(
            Float64, '/gait/penetration_depth', 10)
        self._swing_publisher = self.create_publisher(
            Float64, '/gait/swing_duration', 10)
        self._gait_client = self.create_client(SetBool, '/gait/enable')
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_timer(1.0 / rate, self._publish)

    def _axis(self, message: Joy, parameter: str) -> float:
        return axis_value(
            message.axes, int(self._parameters[parameter]), self._deadzone)

    def _rising_edge(self, buttons: List[int], parameter: str) -> bool:
        index = int(self._parameters[parameter])
        current = 0 <= index < len(buttons) and bool(buttons[index])
        previous = 0 <= index < len(self._last_buttons) and bool(
            self._last_buttons[index])
        return current and not previous

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _on_joy(self, message: Joy) -> None:
        self._last_joy_time = self.get_clock().now()
        if self._rising_edge(message.buttons, 'button_switch_mode'):
            if self._gait_client.service_is_ready():
                self._stepping = not self._stepping
                request = SetBool.Request()
                request.data = self._stepping
                self._gait_client.call_async(request)
            else:
                self.get_logger().warning(
                    'Gait service is not ready; mode change was ignored.')
        if self._rising_edge(message.buttons, 'button_estop'):
            self._estop = not self._estop
        if self._rising_edge(message.buttons, 'button_imu_auto'):
            self._imu_auto = not self._imu_auto

        forward = self._axis(message, 'axis_linear_x')
        lateral = self._axis(message, 'axis_linear_y')
        height = self._axis(message, 'axis_height')
        yaw = self._axis(message, 'axis_yaw')
        self._twist = Twist()
        self._pose = PoseStamped()
        self._pose.header.frame_id = 'base_link'
        if self._stepping:
            self._twist.linear.x = forward * float(
                self._parameters['scale_linear_x'])
            self._twist.linear.y = -lateral * float(
                self._parameters['scale_linear_y'])
            self._twist.angular.z = -yaw * float(self._parameters['scale_yaw'])
            self._pose.pose.position.z = height * float(
                self._parameters['scale_height'])
        else:
            roll = -lateral * float(self._parameters['scale_roll_pitch'])
            pitch = forward * float(self._parameters['scale_roll_pitch'])
            yaw_angle = -yaw * float(self._parameters['scale_roll_pitch'])
            self._pose.pose.position.z = height * float(
                self._parameters['scale_height'])
            quaternion = self._quaternion_from_rpy(roll, pitch, yaw_angle)
            (self._pose.pose.orientation.x, self._pose.pose.orientation.y,
             self._pose.pose.orientation.z,
             self._pose.pose.orientation.w) = quaternion

        adjustment = float(self._parameters['adjustment_step'])
        self._clearance = max(0.0, self._clearance + self._axis(
            message, 'axis_dpad_vertical') * adjustment)
        self._penetration = max(0.0, self._penetration - self._axis(
            message, 'axis_dpad_horizontal') * adjustment)
        reset_gait = (
            self._rising_edge(message.buttons, 'button_left_bumper')
            or self._rising_edge(message.buttons, 'button_right_bumper')
        )
        if reset_gait:
            self._clearance = float(self._parameters['clearance_height'])
            self._penetration = float(self._parameters['penetration_depth'])
            self._swing_duration = float(self._parameters['swing_duration'])
        self._last_buttons = list(message.buttons)

    def _publish(self) -> None:
        fresh = self._last_joy_time is not None and (
            (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
            <= self._timeout)
        self._cmd_publisher.publish(
            self._twist if fresh and self._stepping and not self._estop
            else Twist())
        if fresh and not self._stepping and not self._estop:
            self._pose.header.stamp = self.get_clock().now().to_msg()
            self._pose_publisher.publish(self._pose)
        self._estop_publisher.publish(Bool(data=self._estop))
        self._imu_publisher.publish(Bool(data=self._imu_auto))
        self._clearance_publisher.publish(Float64(data=self._clearance))
        self._penetration_publisher.publish(Float64(data=self._penetration))
        self._swing_publisher.publish(Float64(data=self._swing_duration))


def main(args=None) -> None:
    """Run the reference-mapped joystick node."""
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
