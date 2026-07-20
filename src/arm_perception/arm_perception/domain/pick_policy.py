"""Decides *when* seeing a block should turn into playing an action group.

A detector fires every frame (~30 Hz). Handing every frame straight to
the arm would re-trigger the same pick dozens of times, so this policy
sits in between and answers one question per frame: play a motion now,
or not?

It is deliberately pure — no ROS, no OpenCV, no wall clock. Time is
passed in, so the whole state machine is testable without a simulator
and without sleeping.

    IDLE ──same colour seen `confirm_frames` times, centred──▶ PLAYING
      ▲                                                          │
      │                                            motion finishes│
      └──────── cooldown_s elapsed ──── COOLDOWN ◀────────────────┘

Losing the block, or seeing a *different* colour, resets the streak — so
a flicker on the edge of a threshold cannot accumulate into a trigger.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .detection import BlockDetection


class PolicyState(Enum):
    IDLE = "idle"
    PLAYING = "playing"
    COOLDOWN = "cooldown"


@dataclass(frozen=True)
class PickDecision:
    """The policy's verdict for one frame."""

    should_play: bool
    color: Optional[str] = None
    detection: Optional[BlockDetection] = None
    #: Why nothing happened — for logging, never for control flow.
    reason: str = ""


#: Nothing to do this frame.
_NO_ACTION = PickDecision(should_play=False)


class PickPolicy:
    """Debounces detections into at most one motion request at a time.

    Args:
        confirm_frames: consecutive frames of the same colour required
            before triggering. Guards against a single noisy frame.
        cooldown_s: quiet period after a motion finishes, so the arm is
            not re-triggered by the block it just failed to move.
        center_tolerance_px: how far off frame-centre the block may sit
            and still be considered reachable by the fixed trajectory.
            ``None`` disables the check.
    """

    def __init__(
        self,
        *,
        confirm_frames: int = 5,
        cooldown_s: float = 3.0,
        center_tolerance_px: Optional[float] = None,
    ) -> None:
        if confirm_frames < 1:
            raise ValueError(f"confirm_frames must be >= 1, got {confirm_frames}")
        if cooldown_s < 0:
            raise ValueError(f"cooldown_s must not be negative, got {cooldown_s}")
        if center_tolerance_px is not None and center_tolerance_px <= 0:
            raise ValueError(
                f"center_tolerance_px must be positive, got {center_tolerance_px}"
            )

        self._confirm_frames = confirm_frames
        self._cooldown_s = cooldown_s
        self._center_tolerance_px = center_tolerance_px

        self._state = PolicyState.IDLE
        self._streak_color: Optional[str] = None
        self._streak_count = 0
        self._cooldown_until = 0.0

    @property
    def state(self) -> PolicyState:
        return self._state

    @property
    def streak(self) -> int:
        """Consecutive frames of the current candidate colour."""
        return self._streak_count

    def observe(
        self,
        detection: Optional[BlockDetection],
        now: float,
        *,
        frame_center: Optional[tuple] = None,
    ) -> PickDecision:
        """Feed one frame's best detection (or None) and get a verdict."""
        if self._state is PolicyState.PLAYING:
            # The arm is busy; ignore the camera entirely rather than
            # queueing goals we would only have to cancel.
            self._reset_streak()
            return PickDecision(should_play=False, reason="arm busy")

        if self._state is PolicyState.COOLDOWN:
            if now < self._cooldown_until:
                self._reset_streak()
                remaining = self._cooldown_until - now
                return PickDecision(
                    should_play=False, reason=f"cooling down ({remaining:.1f}s left)"
                )
            self._state = PolicyState.IDLE

        if detection is None:
            self._reset_streak()
            return PickDecision(should_play=False, reason="no block in view")

        if not self._is_centered(detection, frame_center):
            self._reset_streak()
            return PickDecision(
                should_play=False,
                color=detection.color,
                detection=detection,
                reason="block too far from frame centre",
            )

        if detection.color != self._streak_color:
            self._streak_color = detection.color
            self._streak_count = 1
        else:
            self._streak_count += 1

        if self._streak_count < self._confirm_frames:
            return PickDecision(
                should_play=False,
                color=detection.color,
                detection=detection,
                reason=f"confirming {self._streak_count}/{self._confirm_frames}",
            )

        self._state = PolicyState.PLAYING
        self._reset_streak()
        return PickDecision(
            should_play=True, color=detection.color, detection=detection
        )

    def motion_finished(self, now: float) -> None:
        """Call when the arm reports the motion ended — success or not.

        Cooldown runs either way: a failed pick usually leaves the block
        where it was, and retrying immediately just loops.
        """
        self._state = PolicyState.COOLDOWN
        self._cooldown_until = now + self._cooldown_s
        self._reset_streak()

    def reset(self) -> None:
        """Drop all state — used when the pipeline is disabled/re-enabled."""
        self._state = PolicyState.IDLE
        self._cooldown_until = 0.0
        self._reset_streak()

    def _is_centered(
        self, detection: BlockDetection, frame_center: Optional[tuple]
    ) -> bool:
        if self._center_tolerance_px is None or frame_center is None:
            return True
        dx, dy = detection.offset_from(*frame_center)
        return (dx * dx + dy * dy) ** 0.5 <= self._center_tolerance_px

    def _reset_streak(self) -> None:
        self._streak_color = None
        self._streak_count = 0
