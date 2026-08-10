"""ROS-independent BNO085 hardware adapter."""

from dataclasses import dataclass
from typing import Tuple

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class ImuSample:
    """One set of fused IMU values."""

    orientation: Quaternion
    angular_velocity: Vector3
    linear_acceleration: Vector3


class Bno085Driver:
    """Wrap the Adafruit BNO08x CircuitPython API."""

    def __init__(self, address: int, report_rate_hz: float) -> None:
        if address not in (0x4A, 0x4B):
            raise ValueError('I2C address must be 0x4A or 0x4B')
        if report_rate_hz <= 0.0:
            raise ValueError('report_rate_hz must be positive')
        from adafruit_extended_bus import ExtendedI2C
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_ROTATION_VECTOR,
        )
        from adafruit_bno08x.i2c import BNO08X_I2C

        self._i2c = ExtendedI2C(1)
        self._sensor = BNO08X_I2C(self._i2c, address=address)
        interval_us = max(1000, int(1_000_000 / report_rate_hz))
        for feature in (BNO_REPORT_ROTATION_VECTOR,
                        BNO_REPORT_GYROSCOPE,
                        BNO_REPORT_ACCELEROMETER):
            self._sensor.enable_feature(feature, interval_us)

    def read(self) -> ImuSample:
        """Read quaternion, rad/s, and m/s^2 values."""
        orientation = self._sensor.quaternion
        angular_velocity = self._sensor.gyro
        acceleration = self._sensor.acceleration
        if orientation is None or angular_velocity is None or acceleration is None:
            raise RuntimeError('sensor report is not available yet')
        return ImuSample(
            tuple(map(float, orientation)),
            tuple(map(float, angular_velocity)),
            tuple(map(float, acceleration)),
        )

    def close(self) -> None:
        """Release the I2C bus."""
        self._i2c.deinit()
