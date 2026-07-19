"""Unit tests for the pure-domain layer (no ROS, no I/O)."""

import pytest

from arm_motion.domain.errors import (
    IncompletePoseError,
    InvalidMotionError,
    JointLimitError,
    UnsupportedJointMotionError,
)
from arm_motion.domain.joint_spec import GripperCommand, JointKind, JointSpec
from arm_motion.domain.motion import Motion, MotionStep, validate_motion_name
from arm_motion.domain.pose import Pose
from arm_motion.domain.robot_profile import build_profile
from arm_motion.domain.servo_scale import ServoScale


def make_profile():
    joints = [
        JointSpec('joint1', lower=-2.09, upper=2.09, jog_step=0.1),
        JointSpec(
            'r_joint',
            lower=-1.57,
            upper=1.57,
            kind=JointKind.GRIPPER,
            group='gripper',
            open_position=-1.0,
            closed_position=0.3,
        ),
    ]
    scales = [
        ServoScale(1, 'joint1', 0, 1000, -2.0943951, 2.0943951),
        ServoScale(10, 'r_joint', 0, 1000, -1.57, 1.57),
    ]
    return build_profile(joints, scales)


class TestJointSpec:

    def test_validate_rejects_out_of_range(self):
        spec = JointSpec('joint1', lower=-1.0, upper=1.0)
        assert spec.validate(0.5) == 0.5
        with pytest.raises(JointLimitError):
            spec.validate(1.5)

    def test_clamp_squeezes_into_range(self):
        spec = JointSpec('joint1', lower=-1.0, upper=1.0)
        assert spec.clamp(2.0) == 1.0
        assert spec.clamp(-2.0) == -1.0
        assert spec.clamp(0.25) == 0.25

    def test_gripper_cannot_be_jogged(self):
        spec = JointSpec(
            'r_joint', lower=-1.57, upper=1.57, kind=JointKind.GRIPPER,
            open_position=-1.0, closed_position=0.3,
        )
        with pytest.raises(UnsupportedJointMotionError):
            spec.jog(0.0, 0.1)

    def test_revolute_cannot_be_opened(self):
        spec = JointSpec('joint1', lower=-1.0, upper=1.0)
        with pytest.raises(UnsupportedJointMotionError):
            spec.gripper_position(GripperCommand.OPEN)

    def test_jog_clamps_at_the_limit(self):
        spec = JointSpec('joint1', lower=-1.0, upper=1.0, jog_step=0.1)
        assert spec.jog(0.95, 0.5) == 1.0

    def test_gripper_requires_both_named_positions(self):
        with pytest.raises(ValueError):
            JointSpec('g', lower=-1.0, upper=1.0, kind=JointKind.GRIPPER)

    def test_gripper_named_positions_must_respect_limits(self):
        with pytest.raises(ValueError):
            JointSpec(
                'g', lower=-1.0, upper=1.0, kind=JointKind.GRIPPER,
                open_position=-5.0, closed_position=0.0,
            )


class TestServoScale:

    def test_centre_pulse_is_zero_radians(self):
        scale = ServoScale(1, 'joint1', 0, 1000, -2.0943951, 2.0943951)
        assert scale.to_radians(500) == pytest.approx(0.0)

    def test_round_trip(self):
        scale = ServoScale(1, 'joint1', 0, 1000, -2.0943951, 2.0943951)
        for pulse in (0, 15, 215, 500, 650, 1000):
            assert scale.to_pulse(scale.to_radians(pulse)) == pulse

    def test_invert_mirrors_the_range(self):
        scale = ServoScale(1, 'j', 0, 1000, -1.0, 1.0, invert=True)
        assert scale.to_radians(0) == pytest.approx(1.0)
        assert scale.to_radians(1000) == pytest.approx(-1.0)

    def test_to_pulse_clamps(self):
        scale = ServoScale(1, 'j', 0, 1000, -1.0, 1.0)
        assert scale.to_pulse(99.0) == 1000
        assert scale.to_pulse(-99.0) == 0


class TestRobotProfile:

    def test_validate_pose_requires_every_joint(self):
        profile = make_profile()
        with pytest.raises(IncompletePoseError):
            profile.validate_pose(Pose({'joint1': 0.0}))

    def test_validate_pose_rejects_out_of_range(self):
        profile = make_profile()
        pose = Pose({'joint1': 3.0, 'r_joint': 0.0})
        with pytest.raises(JointLimitError):
            profile.validate_pose(pose)

    def test_pose_from_pulses_clamps_to_joint_limits(self):
        profile = make_profile()
        # Pulse 1000 maps to 2.0944 rad but joint1's limit is 2.09.
        pose = profile.pose_from_pulses({'joint1': 1000})
        assert pose['joint1'] == pytest.approx(2.09)

    def test_set_gripper_uses_named_positions(self):
        profile = make_profile()
        pose = Pose({'joint1': 0.0, 'r_joint': 0.0})
        opened = profile.set_gripper(pose, 'r_joint', GripperCommand.OPEN)
        assert opened['r_joint'] == pytest.approx(-1.0)
        closed = profile.set_gripper(pose, 'r_joint', GripperCommand.CLOSE)
        assert closed['r_joint'] == pytest.approx(0.3)

    def test_jog_on_gripper_is_rejected(self):
        profile = make_profile()
        pose = Pose({'joint1': 0.0, 'r_joint': 0.0})
        with pytest.raises(UnsupportedJointMotionError):
            profile.jog(pose, 'r_joint', 0.1)

    def test_validate_rejects_a_half_open_gripper(self):
        profile = make_profile()
        # In range for r_joint (-1.57..1.57) but neither open nor closed.
        pose = Pose({'joint1': 0.0, 'r_joint': 0.0})
        with pytest.raises(UnsupportedJointMotionError):
            profile.validate_pose(pose)

    def test_snap_grippers_moves_to_the_nearest_detent(self):
        profile = make_profile()
        pose = Pose({'joint1': 0.0, 'r_joint': 0.2})
        snapped = profile.snap_grippers(pose)
        assert snapped['r_joint'] == pytest.approx(0.3)
        profile.validate_pose(snapped)

        pose = Pose({'joint1': 0.0, 'r_joint': -0.9})
        assert profile.snap_grippers(pose)['r_joint'] == pytest.approx(-1.0)

    def test_home_pose_is_valid(self):
        profile = make_profile()
        # The centre pulse is 0.0 rad, which is NOT a gripper detent — home
        # must snap it, or every fill_missing() would produce an illegal pose.
        profile.validate_pose(profile.home_pose())

    def test_pose_from_pulses_detents_the_gripper(self):
        profile = make_profile()
        pose = profile.pose_from_pulses({'joint1': 500, 'r_joint': 500})
        assert pose['r_joint'] in (pytest.approx(-1.0), pytest.approx(0.3))

    def test_duplicate_servo_ids_rejected(self):
        joints = [
            JointSpec('a', lower=-1.0, upper=1.0),
            JointSpec('b', lower=-1.0, upper=1.0),
        ]
        scales = [ServoScale(1, 'a'), ServoScale(1, 'b')]
        with pytest.raises(ValueError):
            build_profile(joints, scales)


class TestMotion:

    def _step(self, duration_ms=1000, angle=0.0):
        return MotionStep(Pose({'joint1': angle, 'r_joint': 0.0}), duration_ms)

    def test_step_rejects_zero_duration(self):
        with pytest.raises(InvalidMotionError):
            MotionStep(Pose({'joint1': 0.0}), 0)

    def test_cumulative_times(self):
        motion = Motion('m', [self._step(200), self._step(500), self._step(300)])
        assert motion.cumulative_times_ms() == [200, 700, 1000]

    def test_total_duration(self):
        motion = Motion('m', [self._step(200), self._step(500)])
        assert motion.total_duration_ms == 700

    def test_rescale_hits_the_target_exactly(self):
        motion = Motion('m', [self._step(200), self._step(500), self._step(1500)])
        rescaled = motion.rescaled(1000)
        assert rescaled.total_duration_ms == 1000
        assert len(rescaled) == 3

    def test_rescale_keeps_proportions(self):
        motion = Motion('m', [self._step(1000), self._step(3000)])
        rescaled = motion.rescaled(4000)
        assert [s.duration_ms for s in rescaled.steps] == [1000, 3000]

    def test_rescale_rejects_impossible_budget(self):
        motion = Motion('m', [self._step(1000) for _ in range(10)])
        with pytest.raises(InvalidMotionError):
            motion.rescaled(50)

    def test_editing_helpers(self):
        motion = Motion('m', [self._step(angle=0.1), self._step(angle=0.2)])

        appended = motion.with_step_appended(self._step(angle=0.3))
        assert len(appended) == 3

        removed = appended.with_step_removed(0)
        assert len(removed) == 2

        moved, index = motion.with_step_moved(0, 1)
        assert index == 1
        assert moved.steps[1].pose['joint1'] == pytest.approx(0.1)

        # Moving past the end is a no-op rather than an error.
        unmoved, index = motion.with_step_moved(0, -1)
        assert index == 0
        assert unmoved is motion

    def test_name_validation(self):
        validate_motion_name('left_right')
        for bad in ('', 'has space', 'a/b', '..'):
            with pytest.raises(InvalidMotionError):
                validate_motion_name(bad)
