"""Tests for BNO085 sample utilities."""

import math

from quattro_sensors.bno085_driver import ImuSample
from quattro_sensors.bno085_node import diagonal_covariance, sample_is_finite


def test_diagonal_covariance() -> None:
    assert diagonal_covariance(0.5) == [
        0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25]


def test_sample_validation() -> None:
    valid = ImuSample((0.0, 0.0, 0.0, 1.0),
                      (0.1, 0.2, 0.3), (0.0, 0.0, 9.81))
    invalid = ImuSample((0.0, 0.0, 0.0, 1.0),
                        (0.0, math.nan, 0.0), (0.0, 0.0, 9.81))
    assert sample_is_finite(valid)
    assert not sample_is_finite(invalid)
