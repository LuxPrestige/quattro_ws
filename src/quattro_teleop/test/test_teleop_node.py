"""Tests for joystick command conversion utilities."""

import pytest

from quattro_teleop.teleop_node import apply_deadzone, axis_value


def test_deadzone_suppresses_small_input() -> None:
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.1, 0.1) == 0.0


def test_deadzone_rescales_remaining_range() -> None:
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-0.55, 0.1) == pytest.approx(-0.5)


def test_missing_axis_is_zero() -> None:
    assert axis_value([0.5], 3, 0.1) == 0.0
