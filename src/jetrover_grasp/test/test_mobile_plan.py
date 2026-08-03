"""Unit tests for pure mobile-base frame planning."""

import math

import pytest

from jetrover_grasp.application.base_control import Pose2D
from jetrover_grasp.application.mobile_plan import (
    base_goal_to_center_target,
    base_to_odom,
)


def test_base_to_odom_identity_leaves_point_unchanged():
    point = (0.4, -0.2)

    assert base_to_odom(point, Pose2D(0.0, 0.0, 0.0)) == point


def test_base_to_odom_rotates_then_translates():
    result = base_to_odom(
        (1.0, 0.0),
        Pose2D(1.0, 2.0, math.pi / 2.0),
    )

    assert result == pytest.approx((1.0, 3.0), abs=1.0e-12)


def test_block_at_sweet_spot_keeps_identity_base_pose():
    sweet_spot = (0.25, -0.1)

    goal = base_goal_to_center_target(
        sweet_spot,
        Pose2D(0.0, 0.0, 0.0),
        sweet_spot,
    )

    assert goal == Pose2D(0.0, 0.0, 0.0)


def test_goal_places_fixed_target_at_sweet_spot():
    block_base = (0.6, -0.15)
    odom_pose = Pose2D(1.2, -0.4, math.pi / 3.0)
    sweet_spot = (0.28, 0.05)
    target_odom = base_to_odom(block_base, odom_pose)

    goal = base_goal_to_center_target(
        block_base,
        odom_pose,
        sweet_spot,
        goal_yaw=0.0,
    )

    cos_yaw = math.cos(-goal.yaw)
    sin_yaw = math.sin(-goal.yaw)
    target_from_goal_x = target_odom[0] - goal.x
    target_from_goal_y = target_odom[1] - goal.y
    recomputed_block_base = (
        target_from_goal_x * cos_yaw
        - target_from_goal_y * sin_yaw,
        target_from_goal_x * sin_yaw
        + target_from_goal_y * cos_yaw,
    )
    assert recomputed_block_base == pytest.approx(
        sweet_spot,
        abs=1.0e-9,
    )


@pytest.mark.parametrize(
    ("block_base", "odom_pose", "sweet_spot", "goal_yaw"),
    [
        ((math.nan, 0.0), Pose2D(0.0, 0.0, 0.0), (0.2, 0.0), 0.0),
        ((0.4, 0.0), Pose2D(math.inf, 0.0, 0.0), (0.2, 0.0), 0.0),
        ((0.4, 0.0), Pose2D(0.0, 0.0, -math.inf), (0.2, 0.0), 0.0),
        ((0.4, 0.0), Pose2D(0.0, 0.0, 0.0), (math.nan, 0.0), 0.0),
        ((0.4, 0.0), Pose2D(0.0, 0.0, 0.0), (0.2, 0.0), math.inf),
    ],
)
def test_non_finite_inputs_raise_value_error(
    block_base,
    odom_pose,
    sweet_spot,
    goal_yaw,
):
    with pytest.raises(ValueError):
        base_goal_to_center_target(
            block_base,
            odom_pose,
            sweet_spot,
            goal_yaw,
        )


def test_base_to_odom_rejects_non_finite_input():
    with pytest.raises(ValueError):
        base_to_odom(
            (0.0, math.inf),
            Pose2D(0.0, 0.0, 0.0),
        )
