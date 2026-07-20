"""PlayMotionUseCase — resolution, rescaling, and the 'always go home first' prelude."""

import pytest

from arm_motion.application.play_motion import PlayMotionRequest, PlayMotionUseCase
from arm_motion.application.task_motions import TaskMotionMap, TaskBinding
from arm_motion.domain.errors import InvalidMotionError, MotionNotFoundError
from arm_motion.domain.motion import Motion, MotionStep
from arm_motion.domain.pose import Pose
from arm_motion.domain.robot_profile import RobotProfile
from arm_motion.domain.joint_spec import JointSpec, JointKind
from arm_motion.domain.servo_scale import ServoScale


def profile():
    joints = [
        JointSpec(name="joint1", kind=JointKind.REVOLUTE, group="arm",
                  lower=-2.09, upper=2.09),
        JointSpec(name="r_joint", kind=JointKind.GRIPPER, group="gripper",
                  lower=-1.57, upper=1.57,
                  open_position=-1.0, closed_position=0.3),
    ]
    scales = {
        "joint1": ServoScale(1, "joint1", 0, 1000, -2.0943951, 2.0943951),
        "r_joint": ServoScale(10, "r_joint", 0, 1000, -1.57, 1.57),
    }
    return RobotProfile(joints=joints, scales=scales)


# The gripper is a detent joint: only its open/closed positions validate.
CLOSED = 0.3


def step(angle=0.0, duration=500):
    return MotionStep(Pose({"joint1": angle, "r_joint": CLOSED}), duration)


class FakeRepo:
    def __init__(self, motions):
        self._motions = dict(motions)

    def load(self, name):
        try:
            return self._motions[name]
        except KeyError:
            raise MotionNotFoundError(f"no motion '{name}'") from None

    def save(self, motion, *, overwrite=False):
        raise NotImplementedError

    def list(self):
        return list(self._motions.values())

    def delete(self, name):
        raise NotImplementedError

    def exists(self, name):
        return name in self._motions


class FakeExecutor:
    def __init__(self):
        self.executed = []

    def execute(self, motion, *, on_progress=None, should_cancel=None, owner=None):
        self.executed.append(motion)


def build(*, prelude="", motions=None, tasks=None):
    motions = motions or {
        "home": Motion("home", [step(0.0, 1500)]),
        "pick": Motion("pick", [step(0.5), step(0.9), step(0.4)]),
    }
    executor = FakeExecutor()
    use_case = PlayMotionUseCase(
        repository=FakeRepo(motions),
        executor=executor,
        profile=profile(),
        task_motions=TaskMotionMap(tasks or {}),
        prelude_motion=prelude,
    )
    return use_case, executor


class TestWithoutPrelude:
    def test_plays_exactly_the_requested_motion(self):
        use_case, executor = build()

        use_case.execute(PlayMotionRequest(motion_name="pick"))

        assert len(executor.executed) == 1
        assert len(executor.executed[0]) == 3

    def test_unknown_motion_raises(self):
        use_case, _ = build()
        with pytest.raises(MotionNotFoundError):
            use_case.execute(PlayMotionRequest(motion_name="nope"))


class TestPrelude:
    def test_home_steps_are_prepended(self):
        use_case, executor = build(prelude="home")

        result = use_case.execute(PlayMotionRequest(motion_name="pick"))

        played = executor.executed[0]
        assert len(played) == 4          # 1 home step + 3 pick steps
        assert result.steps_executed == 4

    def test_home_comes_first_not_last(self):
        use_case, executor = build(prelude="home")

        use_case.execute(PlayMotionRequest(motion_name="pick"))

        played = executor.executed[0]
        assert played.steps[0].pose["joint1"] == 0.0     # the home pose
        assert played.steps[1].pose["joint1"] == 0.5     # first pick step

    def test_everything_runs_as_one_trajectory(self):
        """One executor call keeps progress and cancellation coherent."""
        use_case, executor = build(prelude="home")

        use_case.execute(PlayMotionRequest(motion_name="pick"))

        assert len(executor.executed) == 1

    def test_playing_home_itself_does_not_double_it(self):
        use_case, executor = build(prelude="home")

        use_case.execute(PlayMotionRequest(motion_name="home"))

        assert len(executor.executed[0]) == 1

    def test_empty_prelude_disables_the_feature(self):
        use_case, executor = build(prelude="")

        use_case.execute(PlayMotionRequest(motion_name="pick"))

        assert len(executor.executed[0]) == 3

    def test_missing_prelude_fails_loudly(self):
        """Silently skipping the safety pose would defeat the point."""
        use_case, _ = build(prelude="not_there")

        with pytest.raises(InvalidMotionError, match="prelude motion 'not_there'"):
            use_case.execute(PlayMotionRequest(motion_name="pick"))

    def test_prelude_keeps_its_own_timing_when_the_request_is_rescaled(self):
        """duration_ms budgets the requested motion, not the trip home."""
        use_case, executor = build(prelude="home")

        use_case.execute(PlayMotionRequest(motion_name="pick", duration_ms=300))

        played = executor.executed[0]
        assert played.steps[0].duration_ms == 1500        # home untouched
        assert sum(s.duration_ms for s in played.steps[1:]) == 300

    def test_prelude_applies_to_task_requests_too(self):
        tasks = {"grab": TaskBinding(motion_name="pick", duration_ms=0)}
        use_case, executor = build(prelude="home", tasks=tasks)

        use_case.execute(PlayMotionRequest(task="grab"))

        assert len(executor.executed[0]) == 4

    def test_resolve_reports_the_combined_motion(self):
        use_case, _ = build(prelude="home")

        motion = use_case.resolve(PlayMotionRequest(motion_name="pick"))

        assert len(motion) == 4
        assert motion.name == "pick"      # still named after what was asked for
