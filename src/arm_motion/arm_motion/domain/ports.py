"""Ports — interfaces the domain owns and the infrastructure implements."""

from abc import ABC, abstractmethod
from typing import Callable, List, Optional

from .motion import Motion
from .pose import Pose


class MotionRepository(ABC):
    """Persistence for action groups (one ``.d6a`` SQLite file each)."""

    @abstractmethod
    def save(self, motion: Motion, *, overwrite: bool = False) -> Motion:
        """Persist ``motion``.

        Raises:
            MotionAlreadyExistsError: if it exists and ``overwrite`` is False.
        """

    @abstractmethod
    def load(self, name: str) -> Motion:
        """Read a motion back.

        Raises:
            MotionNotFoundError: if no such motion exists.
        """

    @abstractmethod
    def list(self) -> List[Motion]:
        """All stored motions, without their step payloads populated."""

    @abstractmethod
    def delete(self, name: str) -> None:
        """Remove a motion.

        Raises:
            MotionNotFoundError: if no such motion exists.
        """

    @abstractmethod
    def exists(self, name: str) -> bool:
        """Whether a motion of that name is stored."""


class JointStateSource(ABC):
    """Where the arm currently is."""

    @abstractmethod
    def current_pose(self) -> Optional[Pose]:
        """Latest known pose, or None if no joint state has arrived yet."""


#: Called with (step_index, step_count) as each waypoint is dispatched.
ProgressCallback = Callable[[int, int], None]

#: Polled by the executor; returning True aborts the motion.
CancelCheck = Callable[[], bool]


class TrajectoryExecutor(ABC):
    """Sends a motion to the robot and waits for it to finish."""

    @abstractmethod
    def execute(
        self,
        motion: Motion,
        *,
        on_progress: Optional[ProgressCallback] = None,
        should_cancel: Optional[CancelCheck] = None,
        owner: Optional[object] = None,
    ) -> None:
        """Run ``motion`` to completion.

        Args:
            owner: Opaque tag identifying the caller, so that a later
                ``cancel(owner=...)`` can target this execution and no other.

        Raises:
            MotionExecutionError: if the controller rejected or aborted it.
            MotionCancelledError: if ``should_cancel`` returned True.
        """

    @abstractmethod
    def cancel(self, owner: Optional[object] = None) -> bool:
        """Ask the in-flight execution to stop. Safe to call from any thread.

        Returns whether an execution was actually cancelled.
        """
