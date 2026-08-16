"""Tests for joystick command conversion utilities."""

import pytest

from quattro_teleop.keyboard_teleop import command_for_key
from quattro_teleop.teleop_node import apply_deadzone, axis_value


def test_deadzone_suppresses_small_input() -> None:
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.1, 0.1) == 0.0


def test_deadzone_rescales_remaining_range() -> None:
    assert apply_deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert apply_deadzone(-0.55, 0.1) == pytest.approx(-0.5)


def test_missing_axis_is_zero() -> None:
    assert axis_value([0.5], 3, 0.1) == 0.0


def test_keyboard_commands_use_ros_axes() -> None:
    forward = command_for_key('w', 0.1, 0.08, 0.4)
    left = command_for_key('a', 0.1, 0.08, 0.4)
    turn_right = command_for_key('e', 0.1, 0.08, 0.4)

    assert forward.linear.x == pytest.approx(0.1)
    assert left.linear.y == pytest.approx(0.08)
    assert turn_right.angular.z == pytest.approx(-0.4)


def test_keyboard_stop_and_unknown_key() -> None:
    stop = command_for_key(' ', 0.1, 0.08, 0.4)

    assert stop.linear.x == 0.0
    assert stop.linear.y == 0.0
    assert stop.angular.z == 0.0
    assert command_for_key('x', 0.1, 0.08, 0.4) is None
