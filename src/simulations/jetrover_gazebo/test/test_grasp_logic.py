from math import inf, nan, sqrt

import pytest

from jetrover_gazebo.grasp_logic import (
    GraspAction,
    GraspController,
    Pose,
    TimeRollbackDetector,
    is_valid_pose,
    timer_period_from_rate,
)


GRIPPER = Pose(position=(0.0, 0.0, 0.0))


@pytest.mark.parametrize(('update_rate', 'expected_period'), [
    (50.0, 0.02),
    (1e9, 1e-9),
    (1e-9, 1e9),
])
def test_timer_period_from_valid_rate(update_rate, expected_period):
    assert timer_period_from_rate(update_rate) == pytest.approx(
        expected_period)


@pytest.mark.parametrize('update_rate', [
    nan,
    inf,
    -inf,
    0.0,
    -1.0,
    1e-320,
    1e-10,
    1e10,
])
def test_timer_period_rejects_invalid_or_unrepresentable_rate(update_rate):
    with pytest.raises(ValueError):
        timer_period_from_rate(update_rate)


@pytest.fixture
def controller():
    return GraspController(
        closed_position=0.30,
        open_position=-1.00,
        closed_tolerance=0.05,
        open_tolerance=0.05,
        grasp_radius=0.05,
    )


def test_time_rollback_is_reported_once_per_backwards_jump():
    clock = TimeRollbackDetector()

    assert not clock.observe(10_000)
    assert not clock.observe(10_000)
    assert clock.observe(9_999)
    assert not clock.observe(10_000)


def test_time_rollback_can_clear_held_controller_state(controller):
    controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.02, 0.0, 0.0))},
    )
    clock = TimeRollbackDetector()
    clock.observe(10_000)

    assert clock.observe(9_999)
    controller.clear_held_state()

    assert controller.held_block_name is None


def test_rollback_while_closed_does_not_regrasp(controller):
    nearby_block = {'block_red': Pose(position=(0.02, 0.0, 0.0))}
    controller.update(0.30, GRIPPER, nearby_block)

    controller.clear_held_state()
    decision = controller.update(0.30, GRIPPER, nearby_block)

    assert decision.action is GraspAction.IDLE
    assert controller.held_block_name is None


def test_rollback_grasping_resumes_only_after_open_sample(controller):
    nearby_block = {'block_red': Pose(position=(0.02, 0.0, 0.0))}
    controller.update(0.30, GRIPPER, nearby_block)
    controller.clear_held_state()

    intermediate = controller.update(-0.50, GRIPPER, nearby_block)
    still_closed = controller.update(0.30, GRIPPER, nearby_block)
    reopened = controller.update(-1.00, GRIPPER, nearby_block)

    assert intermediate.action is GraspAction.IDLE
    assert still_closed.action is GraspAction.IDLE
    assert reopened.action is GraspAction.IDLE

    decision = controller.update(0.30, GRIPPER, nearby_block)

    assert decision.action is GraspAction.GRASP
    assert decision.block_name == 'block_red'


def test_closed_gripper_grasps_nearest_block(controller):
    decision = controller.update(
        0.30,
        GRIPPER,
        {
            'block_red': Pose(position=(0.03, 0.0, 0.0)),
            'block_blue': Pose(position=(0.01, 0.0, 0.0)),
        },
    )

    assert decision.action is GraspAction.GRASP
    assert decision.block_name == 'block_blue'
    assert controller.held_block_name == 'block_blue'


def test_block_at_grasp_radius_is_included(controller):
    decision = controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.03, 0.04, 0.0))},
    )

    assert decision.action is GraspAction.GRASP


def test_block_just_outside_grasp_radius_is_ignored(controller):
    decision = controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.03, 0.040001, 0.0))},
    )

    assert decision.action is GraspAction.IDLE
    assert controller.held_block_name is None


def test_half_open_gripper_does_not_grasp(controller):
    midpoint = (-1.00 + 0.30) / 2.0
    decision = controller.update(
        midpoint,
        GRIPPER,
        {'block_red': Pose(position=(0.0, 0.0, 0.0))},
    )

    assert decision.action is GraspAction.IDLE


def test_closed_tolerance_boundary_is_included(controller):
    decision = controller.update(
        0.25,
        GRIPPER,
        {'block_red': Pose(position=(0.0, 0.0, 0.0))},
    )

    assert decision.action is GraspAction.GRASP


def test_position_just_outside_closed_tolerance_is_not_closed(controller):
    decision = controller.update(
        0.249999,
        GRIPPER,
        {'block_red': Pose(position=(0.0, 0.0, 0.0))},
    )

    assert decision.action is GraspAction.IDLE


def test_already_held_block_is_not_replaced(controller):
    controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.02, 0.0, 0.0))},
    )

    moved_gripper = Pose(position=(1.0, 0.0, 0.0))
    decision = controller.update(
        0.30,
        moved_gripper,
        {'block_blue': Pose(position=(1.0, 0.0, 0.0))},
    )

    assert decision.action is GraspAction.HOLD
    assert decision.block_name == 'block_red'
    assert decision.target_pose.position == pytest.approx((1.02, 0.0, 0.0))


def test_hold_rotates_the_original_grasp_offset(controller):
    controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.02, 0.0, 0.0))},
    )
    quarter_turn = Pose(
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, sqrt(0.5), sqrt(0.5)),
    )

    decision = controller.update(0.30, quarter_turn, {})

    assert decision.action is GraspAction.HOLD
    assert decision.target_pose.position == pytest.approx((0.0, 0.02, 0.0))


def test_open_gripper_releases_held_block(controller):
    controller.update(
        0.30,
        GRIPPER,
        {'block_yellow': Pose(position=(0.0, 0.0, 0.0))},
    )

    decision = controller.update(-1.00, GRIPPER, {})

    assert decision.action is GraspAction.RELEASE
    assert decision.block_name == 'block_yellow'
    assert decision.target_pose is None
    assert controller.held_block_name is None


@pytest.mark.parametrize('pose', [
    Pose(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 0.0)),
    Pose(position=(nan, 0.0, 0.0)),
    Pose(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, inf)),
    Pose(position=(0.0, 0.0, 0.0), orientation=(1e308,) * 4),
])
def test_malformed_pose_is_rejected(pose):
    assert not is_valid_pose(pose)


def test_malformed_block_pose_is_ignored(controller):
    decision = controller.update(
        0.30,
        GRIPPER,
        {
            'block_bad': Pose(
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 0.0),
            ),
            'block_good': Pose(position=(0.02, 0.0, 0.0)),
        },
    )

    assert decision.action is GraspAction.GRASP
    assert decision.block_name == 'block_good'


def test_extreme_finite_quaternion_block_is_ignored(controller):
    extreme_orientation = (1e308,) * 4

    decision = controller.update(
        0.30,
        GRIPPER,
        {
            'block_bad': Pose(
                position=(0.0, 0.0, 0.0),
                orientation=extreme_orientation,
            ),
        },
    )

    assert decision.action is GraspAction.IDLE
    assert controller.held_block_name is None


def test_malformed_gripper_sample_does_not_drop_held_block(controller):
    controller.update(
        0.30,
        GRIPPER,
        {'block_red': Pose(position=(0.02, 0.0, 0.0))},
    )

    malformed_gripper = Pose(
        position=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 0.0),
    )
    decision = controller.update(0.30, malformed_gripper, {})

    assert decision.action is GraspAction.HOLD
    assert decision.block_name == 'block_red'
    assert decision.target_pose is None
    assert controller.held_block_name == 'block_red'
