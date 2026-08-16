"""Unit tests for ROS-independent kinematics."""

import numpy as np
import pytest

from quattro.kinematics import (
    LEG_NAMES,
    LegKinematics,
    QuadrupedKinematics,
    UnreachableTargetError,
)


def test_nominal_stance_is_finite_and_symmetric():
    model = QuadrupedKinematics()
    angles = model.inverse()

    assert angles.shape == (4, 3)
    assert np.all(np.isfinite(angles))
    assert angles[0, 0] == pytest.approx(-angles[1, 0])
    assert angles[2, 0] == pytest.approx(-angles[3, 0])
    assert angles[0, 1:] == pytest.approx(angles[1, 1:])
    assert angles[2, 1:] == pytest.approx(angles[3, 1:])


def test_forward_inverse_round_trip():
    model = QuadrupedKinematics()
    expected_feet = {
        name: position.copy()
        for name, position in model.nominal_foot_positions.items()
    }
    joints = model.inverse(foot_positions=expected_feet)

    actual_feet = model.forward(joints)

    for name in LEG_NAMES:
        assert actual_feet[name] == pytest.approx(expected_feet[name], abs=1e-10)


def test_forward_inverse_round_trip_with_body_pose():
    model = QuadrupedKinematics()
    rpy = (0.04, -0.03, 0.02)
    translation = (0.01, -0.005, 0.01)
    joints = model.inverse(rpy, translation)

    actual_feet = model.forward(joints, rpy, translation)

    for name in LEG_NAMES:
        assert actual_feet[name] == pytest.approx(
            model.nominal_foot_positions[name], abs=1e-10)


def test_joint_names_use_canonical_order():
    names = QuadrupedKinematics().joint_names

    assert len(names) == 12
    assert names[0] == 'front_left_hip_joint'
    assert names[-1] == 'back_right_lower_leg_joint'
    assert all(any(name.startswith(leg) for leg in LEG_NAMES) for name in names)


def test_leg_rejects_target_beyond_workspace():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)

    with pytest.raises(UnreachableTargetError):
        leg.inverse((1.0, 0.2, -0.2))


def test_invalid_foot_mapping_is_rejected():
    with pytest.raises(ValueError):
        QuadrupedKinematics().inverse(foot_positions={})
