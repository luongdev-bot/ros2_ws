"""Tests for the ROS-independent PID controller."""

import pytest

from line_follow.domain.pid import PID


def test_proportional_only_response_is_constant_for_constant_error():
    pid = PID(kp=2.0, ki=0.0, kd=0.0)

    assert pid.update(error=3.0, dt=0.0) == pytest.approx(6.0)
    assert pid.update(error=3.0, dt=0.1) == pytest.approx(6.0)


def test_output_is_clamped_to_both_limits():
    pid = PID(kp=10.0, ki=0.0, kd=0.0, output_min=-2.0, output_max=2.0)

    assert pid.update(error=1.0, dt=0.1) == pytest.approx(2.0)
    assert pid.update(error=-1.0, dt=0.1) == pytest.approx(-2.0)


def test_non_positive_dt_skips_integral_without_raising():
    pid = PID(kp=0.0, ki=1.0, kd=1.0)

    assert pid.update(error=5.0, dt=0.0) == pytest.approx(0.0)
    assert pid.update(error=10.0, dt=-1.0) == pytest.approx(0.0)
    assert pid.update(error=10.0, dt=1.0) == pytest.approx(10.0)


def test_reset_matches_a_fresh_controller():
    pid = PID(kp=0.5, ki=2.0, kd=0.25)
    for _ in range(4):
        pid.update(error=3.0, dt=0.2)

    pid.reset()
    fresh = PID(kp=0.5, ki=2.0, kd=0.25)

    assert pid.update(error=2.0, dt=0.5) == pytest.approx(
        fresh.update(error=2.0, dt=0.5)
    )


def test_sustained_saturation_does_not_wind_up_integral():
    pid = PID(
        kp=0.0,
        ki=1.0,
        kd=0.0,
        output_min=-1.0,
        output_max=1.0,
    )
    for _ in range(100):
        assert pid.update(error=10.0, dt=0.1) == pytest.approx(1.0)

    first_reversed = pid.update(error=-10.0, dt=0.1)
    second_reversed = pid.update(error=-10.0, dt=0.1)

    assert first_reversed == pytest.approx(0.0)
    assert second_reversed == pytest.approx(-1.0)
