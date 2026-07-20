"""Use case: play a stored motion, by name or by task."""

from dataclasses import dataclass
from typing import Optional

from ..domain.errors import InvalidMotionError
from ..domain.motion import Motion
from ..domain.ports import CancelCheck, MotionRepository, ProgressCallback, TrajectoryExecutor
from ..domain.robot_profile import RobotProfile
from .task_motions import TaskMotionMap


@dataclass(frozen=True)
class PlayMotionRequest:
    """What to play.

    Exactly one of ``motion_name`` / ``task`` must be set. ``duration_ms``
    overrides the recorded timings; 0 keeps them.
    """

    motion_name: str = ""
    task: str = ""
    duration_ms: int = 0

    def __post_init__(self) -> None:
        if bool(self.motion_name) == bool(self.task):
            raise InvalidMotionError(
                "specify exactly one of 'motion_name' or 'task'"
            )
        if self.duration_ms < 0:
            raise InvalidMotionError("duration_ms must be >= 0")


@dataclass(frozen=True)
class PlayMotionResult:
    motion_name: str
    steps_executed: int
    total_duration_ms: int


@dataclass(frozen=True)
class PlayMotionUseCase:
    """Resolve a request to a motion, validate it, and execute it.

    ``prelude_motion`` names an action group the arm always passes through
    before the requested one - typically "home". Its steps are prepended to
    the resolved motion and the whole thing runs as a single trajectory, so
    progress reporting and cancellation stay correct across the join.
    Set it to "" to disable.
    """

    repository: MotionRepository
    executor: TrajectoryExecutor
    profile: RobotProfile
    task_motions: TaskMotionMap
    prelude_motion: str = ""

    def resolve(self, request: PlayMotionRequest) -> Motion:
        """Load and prepare the motion without executing it."""
        if request.task:
            binding = self.task_motions.require(request.task)
            name = binding.motion_name
            # An explicit request duration wins over the configured default.
            duration_ms = request.duration_ms or binding.duration_ms
        else:
            name = request.motion_name
            duration_ms = request.duration_ms

        motion = self._load_validated(name)

        # Rescale before prepending the prelude: `duration_ms` is the budget
        # for the motion that was asked for, not for the trip home first.
        if duration_ms > 0:
            motion = motion.rescaled(duration_ms)

        return self._with_prelude(motion, name)

    def _load_validated(self, name: str) -> Motion:
        """Load ``name`` and reject any pose the arm is not allowed to reach.

        Files on disk can predate the current joint limits, so every step is
        re-checked against the profile rather than trusted.
        """
        motion = self.repository.load(name)
        motion.require_steps()
        for index, step in enumerate(motion.steps):
            try:
                self.profile.validate_pose(step.pose)
            except Exception as exc:
                raise InvalidMotionError(
                    f"motion '{name}' step {index + 1}: {exc}"
                ) from exc
        return motion

    def _with_prelude(self, motion: Motion, name: str) -> Motion:
        """Prepend the configured prelude, unless it is what was asked for."""
        if not self.prelude_motion or name == self.prelude_motion:
            return motion

        try:
            prelude = self._load_validated(self.prelude_motion)
        except InvalidMotionError:
            raise
        except Exception as exc:
            # A missing prelude must not silently skip the safety pose - the
            # whole point of configuring one is that the arm always goes
            # through it.
            raise InvalidMotionError(
                f"prelude motion '{self.prelude_motion}' could not be loaded: "
                f"{exc}. Set the 'prelude_motion' parameter to '' to disable it."
            ) from exc

        return motion.prepended_with(prelude)

    def execute(
        self,
        request: PlayMotionRequest,
        *,
        on_progress: Optional[ProgressCallback] = None,
        should_cancel: Optional[CancelCheck] = None,
        owner: Optional[object] = None,
    ) -> PlayMotionResult:
        motion = self.resolve(request)
        self.executor.execute(
            motion,
            on_progress=on_progress,
            should_cancel=should_cancel,
            owner=owner,
        )
        return PlayMotionResult(
            motion_name=motion.name,
            steps_executed=len(motion),
            total_duration_ms=motion.total_duration_ms,
        )
