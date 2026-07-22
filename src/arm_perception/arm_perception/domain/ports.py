"""Ports — interfaces the domain owns and the infrastructure implements."""

from abc import ABC, abstractmethod
from typing import Any, Mapping, Sequence

from .detection import BlockDetection
from .motion_step import MotionStep


class BlockDetector(ABC):
    """Finds coloured blocks in a camera frame.

    ``frame`` is whatever the adapter's imaging library uses (a NumPy
    BGR array for the OpenCV implementation). The domain never inspects
    it — it only passes it through — so no imaging dependency leaks in.
    """

    @abstractmethod
    def detect(self, frame: Any) -> Sequence[BlockDetection]:
        """All blocks visible in ``frame``, one per configured colour at most."""


class MotionPlayer(ABC):
    """Plays a stored action group (.d6a) on the arm."""

    @abstractmethod
    def play(self, motion_name: str, *, duration_ms: int = 0) -> None:
        """Start ``motion_name``. Returns as soon as the goal is accepted.

        Completion is reported out-of-band via the callback registered
        with :meth:`on_finished`, so the caller is never blocked while
        the arm moves.

        Raises:
            MotionPlaybackError: if the arm rejects the goal.
        """

    @abstractmethod
    def on_finished(self, callback) -> None:
        """Register ``callback(motion_name, success)``, fired once per motion."""

    @abstractmethod
    def is_busy(self) -> bool:
        """Whether a motion is currently executing."""

    @abstractmethod
    def cancel(self) -> None:
        """Best-effort abort of the running motion, if any.

        Used by the watchdog to stop the arm when a motion overruns; safe
        to call when nothing is running.
        """


class ColorMotionMap(ABC):
    """Which action group(s) to play for a detected colour."""

    @abstractmethod
    def steps_for(self, color: str) -> Sequence[MotionStep]:
        """Ordered action groups to play for ``color`` (e.g. pick then place).

        Always at least one step. Played in order, each starting only once
        the previous one has succeeded.

        Raises:
            UnknownColorError: if the colour has no mapping.
        """

    @abstractmethod
    def motion_for(self, color: str) -> str:
        """Name of the first (pick) step bound to ``color``.

        Raises:
            UnknownColorError: if the colour has no mapping.
        """

    @abstractmethod
    def duration_for(self, color: str) -> int:
        """Playback budget of the first step in ms; 0 keeps the .d6a timings."""

    @abstractmethod
    def as_dict(self) -> Mapping[str, str]:
        """The whole colour -> first-motion mapping, for logging on startup."""
