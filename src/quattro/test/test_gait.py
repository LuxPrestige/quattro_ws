"""Unit tests for the reference-derived Bézier gait generator."""

import numpy as np
import pytest

from quattro.gait import GaitGenerator, GaitParameters
from quattro.gait_controller import ramp_value
from quattro.kinematics import LEG_NAMES, QuadrupedKinematics


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
