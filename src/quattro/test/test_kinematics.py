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


def _numerical_jacobian(leg, joints, step=1.0e-6):
    joints = np.asarray(joints, dtype=float)
    columns = []
    for index in range(3):
        forward = joints.copy()
        backward = joints.copy()
        forward[index] += step
        backward[index] -= step
        columns.append(
            (leg.forward(forward) - leg.forward(backward)) / (2.0 * step))
    return np.stack(columns, axis=1)


def test_analytic_jacobian_matches_numerical_jacobian():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    for joints in ((0.1, -0.3, -1.2), (-0.2, 0.4, -0.9), (0.0, 0.0, -1.5)):
        analytic = leg.jacobian(joints)
        numerical = _numerical_jacobian(leg, joints)
        assert analytic == pytest.approx(numerical, abs=1e-6)


def test_analytic_jacobian_matches_numerical_jacobian_right_leg():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=False)
    joints = (0.15, -0.25, -1.1)
    analytic = leg.jacobian(joints)
    numerical = _numerical_jacobian(leg, joints)
    assert analytic == pytest.approx(numerical, abs=1e-6)


def test_foot_velocity_matches_jacobian_times_joint_velocity():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    joints = (0.1, -0.3, -1.2)
    joint_velocities = (0.4, -0.2, 0.7)

    velocity = leg.foot_velocity(joints, joint_velocities)

    assert velocity == pytest.approx(leg.jacobian(joints) @ np.array(joint_velocities))


def test_joint_velocity_from_foot_velocity_round_trips_away_from_singularity():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    joints = (0.1, -0.3, -1.2)
    desired_foot_velocity = np.array([0.05, -0.03, 0.02])

    joint_velocities = leg.joint_velocity_from_foot_velocity(
        joints, desired_foot_velocity, damping=1.0e-6)
    reconstructed = leg.foot_velocity(joints, joint_velocities)

    assert reconstructed == pytest.approx(desired_foot_velocity, abs=1e-4)


def test_force_to_joint_torque_matches_jacobian_transpose():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    joints = (0.1, -0.3, -1.2)
    force = (0.0, 0.0, -40.0)

    torque = leg.force_to_joint_torque(joints, force)

    assert torque == pytest.approx(leg.jacobian(joints).T @ np.array(force))


def test_damped_pseudoinverse_stays_finite_near_singularity():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    # knee near full extension collapses the leg toward a 2D workspace
    # (upper_length == lower_length here), a near-singular configuration.
    joints = (0.0, 0.3, -1.0e-6)

    joint_velocities = leg.joint_velocity_from_foot_velocity(
        joints, (0.1, 0.1, 0.1), damping=1.0e-2)

    assert np.all(np.isfinite(joint_velocities))
    assert np.all(np.abs(joint_velocities) < 1.0e6)


def test_jacobian_rejects_invalid_shape():
    leg = LegKinematics(0.0905, 0.210, 0.210, is_left=True)
    with pytest.raises(ValueError):
        leg.jacobian((0.0, 0.0))
    with pytest.raises(ValueError):
        leg.foot_velocity((0.0, 0.0, 0.0), (0.0, 0.0))
    with pytest.raises(ValueError):
        leg.force_to_joint_torque((0.0, 0.0, 0.0), (0.0, 0.0, float('nan')))


def test_quadruped_joint_velocities_matches_per_leg_jacobian():
    model = QuadrupedKinematics()
    joints = model.inverse()
    foot_velocities = {
        'front_left': (0.02, 0.01, -0.03),
        'front_right': (0.02, -0.01, -0.03),
        'back_left': (-0.02, 0.01, -0.03),
        'back_right': (-0.02, -0.01, -0.03),
    }

    result = model.joint_velocities((0.0, 0.0, 0.0), joints, foot_velocities)

    assert result.shape == (4, 3)
    assert np.all(np.isfinite(result))
    for index, name in enumerate(LEG_NAMES):
        leg = model._legs[name]
        reconstructed = leg.foot_velocity(joints[index], result[index])
        assert reconstructed == pytest.approx(
            np.array(foot_velocities[name]), abs=1e-4)


def test_quadruped_joint_velocities_rejects_missing_leg():
    model = QuadrupedKinematics()
    joints = model.inverse()
    with pytest.raises(ValueError):
        model.joint_velocities((0.0, 0.0, 0.0), joints, {})
