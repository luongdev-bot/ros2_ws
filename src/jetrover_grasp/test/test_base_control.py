"""Unit tests for pure holonomic base feedback control."""

import math

import pytest

from jetrover_grasp.application.base_control import (
    BaseGains,
    Pose2D,
    body_velocity_command,
    yaw_from_quaternion,
)


def test_already_at_goal_is_reached_with_zero_command():
    pose = Pose2D(1.0, -2.0, 0.4)

    command = body_velocity_command(pose, pose, BaseGains())

    assert command == (0.0, 0.0, 0.0, True)


def test_world_positive_x_goal_drives_forward_at_zero_yaw():
    current = Pose2D(0.0, 0.0, 0.0)
    goal = Pose2D(0.1, 0.0, 0.0)

    vx, vy, wz, reached = body_velocity_command(
        current,
        goal,
        BaseGains(),
    )

    assert vx > 0.0
    assert vy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)
    assert reached is False


def test_world_positive_y_goal_strafes_left_at_zero_yaw():
    current = Pose2D(0.0, 0.0, 0.0)
    goal = Pose2D(0.0, 0.1, 0.0)

    vx, vy, wz, reached = body_velocity_command(
        current,
        goal,
        BaseGains(),
    )

    assert vx == pytest.approx(0.0)
    assert vy > 0.0
    assert wz == pytest.approx(0.0)
    assert reached is False


def test_world_y_goal_is_body_forward_when_robot_faces_positive_y():
    current = Pose2D(0.0, 0.0, math.pi / 2.0)
    goal = Pose2D(0.0, 0.1, math.pi / 2.0)

    vx, vy, wz, reached = body_velocity_command(
        current,
        goal,
        BaseGains(),
    )

    assert vx > 0.0
    assert vy == pytest.approx(0.0, abs=1.0e-12)
    assert wz == pytest.approx(0.0)
    assert reached is False


def test_large_errors_saturate_each_velocity_axis():
    current = Pose2D(0.0, 0.0, 0.0)
    goal = Pose2D(10.0, -10.0, 2.0)
    gains = BaseGains(max_lin=0.2, max_ang=0.7)

    vx, vy, wz, reached = body_velocity_command(
        current,
        goal,
        gains,
    )

    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(-0.2)
    assert wz == pytest.approx(0.7)
    assert reached is False


def test_yaw_only_error_wraps_across_pi_with_correct_sign():
    current = Pose2D(0.0, 0.0, -3.0)
    goal = Pose2D(0.0, 0.0, 3.0)

    vx, vy, wz, reached = body_velocity_command(
        current,
        goal,
        BaseGains(),
    )

    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.0)
    assert -1.0 < wz < 0.0
    assert wz == pytest.approx(2.0 * (6.0 - 2.0 * math.pi))
    assert reached is False


@pytest.mark.parametrize(
    ("current", "goal", "gains"),
    [
        (
            Pose2D(math.nan, 0.0, 0.0),
            Pose2D(0.0, 0.0, 0.0),
            BaseGains(),
        ),
        (
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.0, math.inf, 0.0),
            BaseGains(),
        ),
        (
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.0, 0.0, 0.0),
            BaseGains(kp_lin=math.nan),
        ),
    ],
)
def test_non_finite_controller_input_raises(current, goal, gains):
    with pytest.raises(ValueError):
        body_velocity_command(current, goal, gains)


def test_yaw_from_quaternion_identity_and_ninety_degrees():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

    half_angle = math.pi / 4.0
    yaw = yaw_from_quaternion(
        0.0,
        0.0,
        math.sin(half_angle),
        math.cos(half_angle),
    )
    assert yaw == pytest.approx(math.pi / 2.0)
