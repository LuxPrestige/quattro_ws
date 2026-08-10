"""Publish BNO085 measurements as sensor_msgs/Imu."""

import math
from typing import List, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from quattro_sensors.bno085_driver import Bno085Driver, ImuSample
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def diagonal_covariance(stddev: float) -> List[float]:
    """Return a diagonal 3x3 covariance matrix."""
    variance = stddev ** 2
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def sample_is_finite(sample: ImuSample) -> bool:
    """Check all values before publication."""
    values = (sample.orientation + sample.angular_velocity +
              sample.linear_acceleration)
    return all(math.isfinite(value) for value in values)


class Bno085Node(Node):
    """BNO085 ROS 2 sensor and diagnostics node."""

    def __init__(self) -> None:
        super().__init__('bno085')
        defaults = {
            'frame_id': 'imu_link', 'i2c_address': 0x4A,
            'publish_rate_hz': 100.0, 'reconnect_interval_sec': 2.0,
            'orientation_stddev': 0.05,
            'angular_velocity_stddev': 0.02,
            'linear_acceleration_stddev': 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._address = int(self.get_parameter('i2c_address').value)
        self._rate = float(self.get_parameter('publish_rate_hz').value)
        retry = float(self.get_parameter('reconnect_interval_sec').value)
        if self._rate <= 0.0 or retry <= 0.0:
            raise ValueError('rates and intervals must be positive')
        self._covariances = [diagonal_covariance(float(
            self.get_parameter(name).value)) for name in (
                'orientation_stddev', 'angular_velocity_stddev',
                'linear_acceleration_stddev')]
        self._driver: Optional[Bno085Driver] = None
        self._last_error = 'not connected'
        self._samples = 0
        self._imu_pub = self.create_publisher(
            Imu, '/imu/data', qos_profile_sensor_data)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0 / self._rate, self._publish_imu)
        self.create_timer(retry, self._connect)
        self.create_timer(1.0, self._publish_diagnostic)
        self._connect()

    def _connect(self) -> None:
        if self._driver is not None:
            return
        try:
            self._driver = Bno085Driver(self._address, self._rate)
            self._last_error = ''
            self.get_logger().info(
                f'BNO085 connected at 0x{self._address:02X}')
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            self._last_error = str(error)
            self.get_logger().warning(
                f'BNO085 connection failed: {error}',
                throttle_duration_sec=5.0)

    def _publish_imu(self) -> None:
        if self._driver is None:
            return
        try:
            sample = self._driver.read()
            if not sample_is_finite(sample):
                raise ValueError('non-finite sensor value')
        except (OSError, RuntimeError, ValueError) as error:
            self._last_error = str(error)
            self.get_logger().error(
                f'BNO085 read failed: {error}', throttle_duration_sec=2.0)
            self._disconnect()
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.x, msg.orientation.y = sample.orientation[:2]
        msg.orientation.z, msg.orientation.w = sample.orientation[2:]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            sample.angular_velocity)
        msg.linear_acceleration.x, msg.linear_acceleration.y, \
            msg.linear_acceleration.z = sample.linear_acceleration
        msg.orientation_covariance = self._covariances[0]
        msg.angular_velocity_covariance = self._covariances[1]
        msg.linear_acceleration_covariance = self._covariances[2]
        self._imu_pub.publish(msg)
        self._samples += 1

    def _disconnect(self) -> None:
        if self._driver is not None:
            try:
                self._driver.close()
            except (OSError, RuntimeError):
                pass
        self._driver = None

    def _publish_diagnostic(self) -> None:
        connected = self._driver is not None
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR,
            name='quattro_sensors: BNO085',
            message='Connected' if connected else self._last_error,
            hardware_id=f'bno085_i2c_0x{self._address:02x}',
            values=[KeyValue(key='frame_id', value=self._frame_id),
                    KeyValue(key='published_samples', value=str(self._samples))])
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    def destroy_node(self) -> bool:
        """Close hardware before node teardown."""
        self._disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = Bno085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
