"""Unit tests for the reference-derived Bézier gait generator."""

import numpy as np
import pytest

from quattro.gait import FootTrajectoryState, GaitGenerator, GaitParameters
from quattro.gait_controller import ramp_value, staged_joint_targets
from quattro.kinematics import LEG_NAMES, QuadrupedKinematics


def test_staged_joint_targets_move_hips_before_remaining_joints():
    current = list(range(12))
    target = [value + 100 for value in current]

    stages = staged_joint_targets(current, target)

    assert len(stages) == 2
    assert stages[0] == [
        target[index] if index % 3 == 0 else current[index]
        for index in range(12)
    ]
    assert stages[1] == target


def test_staged_joint_targets_reject_invalid_dimensions():
    with pytest.raises(ValueError):
        staged_joint_targets([0.0], [0.0, 1.0])


def test_stationary_command_returns_nominal_stance():
    generator = GaitGenerator()

    feet = generator.update(0.1)

    assert generator.phase == 0.0
    for name in LEG_NAMES:
        assert feet[name] == pytest.approx(
            QuadrupedKinematics().nominal_foot_positions[name])


def test_stop_finishes_current_cycle_before_returning_to_nominal():
    parameters = GaitParameters(stop_return_speed=0.08)
    generator = GaitGenerator(parameters=parameters)
    nominal = QuadrupedKinematics().nominal_foot_positions
    generator.update(0.1, (0.2, 0.0))
    moving_phase = generator.phase

    released = generator.update(0.01)

    assert generator.finishing_cycle
    assert generator.phase > moving_phase
    assert any(np.linalg.norm(released[name] - nominal[name]) > 0.0
               for name in LEG_NAMES)

    for _ in range(100):
        boundary = generator.update(0.01)
        if not generator.finishing_cycle:
            break

    assert not generator.finishing_cycle
    assert generator.phase == 0.0
    returning = generator.update(0.01)
    maximum_step = parameters.stop_return_speed * 0.01
    for name in LEG_NAMES:
        assert np.linalg.norm(returning[name] - boundary[name]) <= (
            maximum_step + 1.0e-12)

    for _ in range(1000):
        stopped = generator.update(0.01)

    for name in LEG_NAMES:
        assert stopped[name] == pytest.approx(nominal[name])


def test_immediate_stop_does_not_finish_cycle():
    parameters = GaitParameters(stop_return_speed=0.08)
    generator = GaitGenerator(parameters=parameters)
    moving = generator.update(0.1, (0.2, 0.0))

    stopped = generator.update(0.01, complete_cycle_on_stop=False)

    assert not generator.finishing_cycle
    maximum_step = parameters.stop_return_speed * 0.01
    for name in LEG_NAMES:
        assert np.linalg.norm(stopped[name] - moving[name]) <= (
            maximum_step + 1.0e-12)


def test_trot_diagonal_legs_share_phase():
    generator = GaitGenerator()
    generator.update(0.07, (0.1, 0.0))

    assert generator.leg_phase('front_left') == generator.leg_phase('back_right')
    assert generator.leg_phase('front_right') == generator.leg_phase('back_left')
    assert generator.leg_phase('front_left') != generator.leg_phase('front_right')


def test_phase_wraps_after_one_stride():
    parameters = GaitParameters(swing_duration=0.25, stance_duration=0.30)
    generator = GaitGenerator(parameters=parameters)

    generator.update(generator.stride_duration * 1.25, (0.1, 0.0))

    assert generator.phase == pytest.approx(0.25)


def test_reference_bezier_swing_endpoints_and_clearance():
    start = GaitGenerator.bezier_swing(0.0, 0.05, 0.0, 0.04)
    middle = GaitGenerator.bezier_swing(0.5, 0.05, 0.0, 0.04)
    end = GaitGenerator.bezier_swing(1.0, 0.05, 0.0, 0.04)

    assert start == pytest.approx((-0.05, 0.0, 0.0))
    assert end == pytest.approx((0.05, 0.0, 0.0))
    assert middle[2] > 0.0


def test_sine_stance_endpoints_and_penetration():
    start = GaitGenerator.sine_stance(0.0, 0.05, 0.0, 0.008)
    middle = GaitGenerator.sine_stance(0.5, 0.05, 0.0, 0.008)
    end = GaitGenerator.sine_stance(1.0, 0.05, 0.0, 0.008)

    assert start == pytest.approx((0.05, 0.0, 0.0))
    assert middle == pytest.approx((0.0, 0.0, -0.008))
    assert end == pytest.approx((-0.05, 0.0, 0.0))


def test_touchdown_resynchronizes_reference_phase():
    generator = GaitGenerator()
    stance_fraction = (
        generator.parameters.stance_duration / generator.stride_duration)
    generator.reset(stance_fraction + 0.95 * (1.0 - stance_fraction))
    contacts = {name: False for name in LEG_NAMES}
    contacts['front_left'] = True

    generator.update(0.01, (0.1, 0.0), contacts=contacts)

    assert generator.phase == 0.0


def test_generated_feet_remain_reachable():
    kinematics = QuadrupedKinematics()
    generator = GaitGenerator()

    for _ in range(100):
        feet = generator.update(0.01, (0.1, 0.04), 0.2)
        joints = kinematics.inverse(foot_positions=feet)
        assert joints.shape == (4, 3)
        assert np.all(np.isfinite(joints))


def test_velocity_above_limit_is_rejected():
    generator = GaitGenerator()

    with pytest.raises(ValueError):
        generator.update(0.01, (1.0, 0.0))


def _numerical_derivative(function, phase, step=1.0e-6):
    return (function(phase + step) - function(phase - step)) / (2.0 * step)


def test_bezier_swing_velocity_matches_numerical_derivative():
    half_step, direction, clearance = 0.05, 0.3, 0.04

    def position(phase):
        return GaitGenerator.bezier_swing(phase, half_step, direction, clearance)

    for phase in (0.05, 0.25, 0.5, 0.75, 0.95):
        _, velocity, _ = GaitGenerator.bezier_swing_with_derivatives(
            phase, half_step, direction, clearance)
        expected = _numerical_derivative(position, phase)
        assert velocity == pytest.approx(expected, abs=1e-4)


def test_bezier_swing_acceleration_matches_numerical_derivative():
    half_step, direction, clearance = 0.05, 0.3, 0.04

    def velocity(phase):
        _, velocity_value, _ = GaitGenerator.bezier_swing_with_derivatives(
            phase, half_step, direction, clearance)
        return velocity_value

    for phase in (0.05, 0.25, 0.5, 0.75, 0.95):
        _, _, acceleration = GaitGenerator.bezier_swing_with_derivatives(
            phase, half_step, direction, clearance)
        expected = _numerical_derivative(velocity, phase)
        assert acceleration == pytest.approx(expected, abs=1e-2)


def test_sine_stance_velocity_matches_numerical_derivative():
    half_step, direction, penetration = 0.05, -0.2, 0.008

    def position(phase):
        return GaitGenerator.sine_stance(phase, half_step, direction, penetration)

    for phase in (0.05, 0.25, 0.5, 0.75, 0.95):
        _, velocity, _ = GaitGenerator.sine_stance_with_derivatives(
            phase, half_step, direction, penetration)
        expected = _numerical_derivative(position, phase)
        assert velocity == pytest.approx(expected, abs=1e-4)


def test_sine_stance_acceleration_matches_numerical_derivative():
    half_step, direction, penetration = 0.05, -0.2, 0.008

    def velocity(phase):
        _, velocity_value, _ = GaitGenerator.sine_stance_with_derivatives(
            phase, half_step, direction, penetration)
        return velocity_value

    for phase in (0.05, 0.25, 0.5, 0.75, 0.95):
        _, _, acceleration = GaitGenerator.sine_stance_with_derivatives(
            phase, half_step, direction, penetration)
        expected = _numerical_derivative(velocity, phase)
        assert acceleration == pytest.approx(expected, abs=1e-2)


def test_update_states_matches_update_positions():
    generator = GaitGenerator()

    for _ in range(20):
        positions = generator.update(0.01, (0.1, 0.03), 0.15)

    generator_replay = GaitGenerator()
    for _ in range(19):
        generator_replay.update_states(0.01, (0.1, 0.03), 0.15)
    states = generator_replay.update_states(0.01, (0.1, 0.03), 0.15)

    for name in LEG_NAMES:
        assert isinstance(states[name], FootTrajectoryState)
        assert states[name].position == pytest.approx(positions[name])


def test_update_states_reports_phase_and_swing_state():
    generator = GaitGenerator()
    generator.reset(0.7)

    states = generator.update_states(0.001, (0.1, 0.0), 0.0)

    for name in LEG_NAMES:
        state = states[name]
        assert 0.0 <= state.phase <= 1.0
        assert isinstance(state.in_swing, bool)
        assert np.all(np.isfinite(state.velocity))
        assert np.all(np.isfinite(state.acceleration))
    swinging = [states[name].in_swing for name in LEG_NAMES]
    assert any(swinging)
    assert not all(swinging)


def test_update_states_velocity_and_acceleration_are_finite_when_stopped():
    generator = GaitGenerator()
    generator.update(0.1, (0.2, 0.0))

    states = generator.update_states(0.01, complete_cycle_on_stop=False)

    for name in LEG_NAMES:
        assert np.all(np.isfinite(states[name].velocity))
        assert np.all(np.isfinite(states[name].acceleration))
        assert states[name].in_swing is False


def test_stop_ramp_reduces_command_without_crossing_zero():
    value = 0.1
    samples = []
    for _ in range(40):
        value = ramp_value(value, 0.0, rate=0.3, dt=0.01)
        samples.append(value)

    assert samples[0] == pytest.approx(0.097)
    assert all(next_value <= current
               for current, next_value in zip(samples, samples[1:]))
    assert samples[-1] == 0.0
