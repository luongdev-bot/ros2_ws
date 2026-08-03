"""Unit tests for the pure JetRover pick-and-place planner."""

import numpy as np
import pytest

from jetrover_grasp.application.grasp_plan import (
    plan_is_reachable,
    plan_pick,
    plan_pick_and_place,
    plan_place,
)
from jetrover_kinematics import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
)


_BLUE_BIN = (0.025, -0.32, 0.02)
_GREEN_BIN = (-0.055, -0.32, 0.02)
_YELLOW_BIN = (0.105, -0.32, 0.02)
_EXPECTED_LABELS = [
    "home",
    "approach",
    "descend",
    "close",
    "lift",
    "to_bin",
    "release",
    "home",
]


@pytest.mark.parametrize(
    "block_xyz, bin_xyz",
    [
        ((0.025, -0.245, 0.085), _BLUE_BIN),
        ((-0.055, -0.245, 0.085), _GREEN_BIN),
        ((0.105, -0.245, 0.085), _YELLOW_BIN),
    ],
    ids=["blue", "green", "yellow"],
)
def test_reachable_blocks_have_complete_plans(block_xyz, bin_xyz):
    """Each reachable simulation block yields a complete grasp plan."""
    plan = plan_pick_and_place(block_xyz, bin_xyz)

    assert plan is not None
    assert len(plan) == 8
    assert [waypoint.label for waypoint in plan] == _EXPECTED_LABELS
    assert plan[3].joint_positions is None
    assert plan[6].joint_positions is None

    for waypoint in plan:
        if waypoint.joint_positions is not None:
            assert len(waypoint.joint_positions) == 5
            assert np.all(
                np.asarray(waypoint.joint_positions) >= JOINT_LIMITS_LOWER
            )
            assert np.all(
                np.asarray(waypoint.joint_positions) <= JOINT_LIMITS_UPPER
            )

    open_labels = {"home", "approach", "descend", "release"}
    close_labels = {"close", "lift", "to_bin"}
    move_labels = {"home", "approach", "descend", "lift", "to_bin"}
    grasp_labels = {"close", "release"}
    for waypoint in plan:
        if waypoint.label in open_labels:
            assert waypoint.gripper_position == -1.0
        if waypoint.label in close_labels:
            assert waypoint.gripper_position == 0.3
        if waypoint.label in move_labels:
            assert waypoint.settle_time_s == 3.0
        if waypoint.label in grasp_labels:
            assert waypoint.settle_time_s == 2.0


def test_red_block_is_outside_the_known_outer_workspace():
    """The red block is the known strict top-down workspace limitation."""
    red_block = (-0.135, -0.245, 0.085)

    # The red pick is outside the known strict top-down outer-workspace
    # limitation, even though the position-only place target is reachable.
    assert plan_pick_and_place(red_block, _BLUE_BIN) is None
    assert plan_is_reachable(red_block, _BLUE_BIN) is False


def test_plan_pick_returns_only_pick_waypoints():
    block = (0.025, -0.245, 0.085)

    plan = plan_pick(block)

    assert plan is not None
    assert [waypoint.label for waypoint in plan] == [
        "home",
        "approach",
        "descend",
        "close",
        "lift",
    ]


def test_plan_place_returns_only_place_waypoints():
    plan = plan_place(_BLUE_BIN)

    assert plan is not None
    assert [waypoint.label for waypoint in plan] == [
        "to_bin",
        "release",
        "home",
    ]


def test_plan_parts_compose_to_the_complete_plan_labels():
    block = (0.025, -0.245, 0.085)

    pick = plan_pick(block)
    assert pick is not None
    place = plan_place(
        _BLUE_BIN,
        q_seed=pick[-1].joint_positions,
    )
    complete = plan_pick_and_place(block, _BLUE_BIN)

    assert place is not None
    assert complete is not None
    assert [waypoint.label for waypoint in pick + place] == _EXPECTED_LABELS
    assert [waypoint.label for waypoint in complete] == _EXPECTED_LABELS


def test_red_block_is_unreachable_via_plan_pick():
    red_block = (-0.135, -0.245, 0.085)

    assert plan_pick(red_block) is None


@pytest.mark.parametrize(
    "block_xyz, bin_xyz",
    [
        ((0.0, 0.0), _BLUE_BIN),
        ((0.025, np.nan, 0.085), _BLUE_BIN),
        ((0.025, -0.245, 0.085), (0.0, 0.0)),
        ((0.025, -0.245, 0.085), (0.025, -0.32, np.inf)),
    ],
)
def test_invalid_positions_raise_value_error(block_xyz, bin_xyz):
    """Positions must be finite vectors of exactly three values."""
    with pytest.raises(ValueError):
        plan_pick_and_place(block_xyz, bin_xyz)
