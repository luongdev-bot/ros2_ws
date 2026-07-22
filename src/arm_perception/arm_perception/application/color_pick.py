"""Use case: see a coloured block, play the matching stored action group.

This is the whole behaviour in one place — detect, debounce, dispatch —
with every external concern (camera, OpenCV, the arm's action server)
behind a port. It can be exercised end to end with fakes.
"""

import logging
from typing import Any, Callable, List, Optional, Sequence, Tuple

from ..domain.detection import BlockDetection, largest
from ..domain.errors import MotionPlaybackError, UnknownColorError
from ..domain.motion_step import MotionStep
from ..domain.pick_policy import PickDecision, PickPolicy
from ..domain.ports import BlockDetector, ColorMotionMap, MotionPlayer

logger = logging.getLogger(__name__)

#: Called with every frame's detections, for visualisation/telemetry.
DetectionSink = Callable[[Sequence[BlockDetection]], None]


class ColorPickUseCase:
    """Drives one camera frame through to (possibly) an arm motion."""

    def __init__(
        self,
        detector: BlockDetector,
        policy: PickPolicy,
        motion_map: ColorMotionMap,
        player: MotionPlayer,
        *,
        clock: Callable[[], float],
        detection_sink: Optional[DetectionSink] = None,
        motion_timeout_s: Optional[float] = None,
    ) -> None:
        if motion_timeout_s is not None and motion_timeout_s <= 0:
            raise ValueError(
                f"motion_timeout_s must be positive, got {motion_timeout_s}"
            )

        self._detector = detector
        self._policy = policy
        self._motion_map = motion_map
        self._player = player
        self._clock = clock
        self._detection_sink = detection_sink
        self._motion_timeout_s = motion_timeout_s
        self._enabled = True

        # The action group(s) still to play for the block currently being
        # handled. Populated on dispatch, drained one step at a time as each
        # motion finishes. Only ever touched from the node's single
        # (mutually-exclusive) callback group, so it needs no locking.
        self._pending_steps: List[MotionStep] = []
        self._active_color: Optional[str] = None
        # When the current step must have reported back by; None disables the
        # watchdog. Guards against an accepted goal that never fires
        # on_finished (a hung/lost server) wedging the pipeline in PLAYING.
        self._deadline: Optional[float] = None

        self._player.on_finished(self._handle_motion_finished)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        """Resume acting on detections."""
        if not self._enabled:
            self._enabled = True
            # Don't disturb a sequence that is still running: resetting the
            # policy mid-pick would drop it out of PLAYING and let the next
            # frame stack a second grasp on top of the one in flight.
            if self._active_color is None:
                self._policy.reset()

    def disable(self) -> None:
        """Stop triggering motions; detections are still published."""
        self._enabled = False

    def process_frame(
        self, frame: Any, *, frame_center: Optional[Tuple[float, float]] = None
    ) -> PickDecision:
        """Run detection on one frame and dispatch a motion if warranted.

        Never raises on a failed dispatch — a camera callback that throws
        would take the node down. Failures are logged and reported in the
        returned decision instead.
        """
        detections = list(self._detector.detect(frame))

        if self._detection_sink is not None:
            # The sink is telemetry only. A broken publisher must not stop
            # the arm from picking, so its failure is logged, not raised.
            try:
                self._detection_sink(detections)
            except Exception:
                logger.exception("detection sink failed; continuing")

        # The watchdog runs before the enabled check: a stuck motion must be
        # recovered even while triggering is paused via ~/enable.
        if self.check_watchdog():
            return PickDecision(should_play=False, reason="motion timed out")

        if not self._enabled:
            return PickDecision(should_play=False, reason="pipeline disabled")

        best = largest(detections)
        decision = self._policy.observe(
            best, self._clock(), frame_center=frame_center
        )
        if not decision.should_play or decision.color is None:
            return decision

        return self._dispatch(decision)

    def _dispatch(self, decision: PickDecision) -> PickDecision:
        color = decision.color

        # Belt and braces: never stack a second sequence. The policy normally
        # blocks this (it stays PLAYING for the whole sequence), but a
        # disable/enable cycle could reset it mid-motion. We key off our own
        # accounting, not the player's is_busy(): if the player were wedged
        # busy, play() below raises and we recover into cooldown rather than
        # stranding the policy in PLAYING.
        if self._active_color is not None:
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason="arm busy",
            )

        try:
            steps = list(self._motion_map.steps_for(color))
        except UnknownColorError as exc:
            # The policy already moved to PLAYING; release it, otherwise
            # an unmapped colour would wedge the pipeline forever.
            logger.warning("%s", exc)
            self._release()
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason=str(exc),
            )

        # steps_for never returns empty, but guard anyway so a future map
        # implementation cannot silently strand the policy in PLAYING.
        if not steps:
            logger.error("colour %s maps to no motion steps", color)
            self._release()
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason="no motion steps",
            )

        self._active_color = color
        self._pending_steps = steps[1:]
        if not self._play(steps[0], color):
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason="playback failed",
            )
        return decision

    def _play(self, step: MotionStep, color: str) -> bool:
        """Start one step. On failure, release the policy and return False."""
        try:
            logger.info("playing %r for %s block", step.name, color)
            self._player.play(step.name, duration_ms=step.duration_ms)
        except MotionPlaybackError as exc:
            logger.error("could not play %r: %s", step.name, exc)
            self._release()
            return False
        # Arm the watchdog for this step: each step gets the full budget, so a
        # long place after a long pick is not cut short by the pick's clock.
        if self._motion_timeout_s is not None:
            self._deadline = self._clock() + self._motion_timeout_s
        return True

    def check_watchdog(self) -> bool:
        """Cancel and recover a motion that has overrun. Returns whether it did.

        Safe to call from a timer independent of the camera, so a hung motion
        is recovered even if the image stream stops. Must run in the same
        callback group as the frame/completion callbacks (it mutates the same
        state), which the node guarantees.
        """
        if not self._watchdog_expired():
            return False
        logger.error(
            "motion for %s block overran %ss; cancelling and recovering",
            self._active_color, self._motion_timeout_s,
        )
        try:
            self._player.cancel()
        except Exception:  # noqa: BLE001 - recovery must not itself raise
            logger.exception("cancel during watchdog recovery failed")
        self._release()
        return True

    def _watchdog_expired(self) -> bool:
        return (
            self._active_color is not None
            and self._deadline is not None
            and self._clock() >= self._deadline
        )

    def _handle_motion_finished(self, motion_name: str, success: bool) -> None:
        if self._active_color is None:
            # A stale or duplicate completion for a sequence we already
            # released. Ignore it rather than release a newer one or start a
            # place step out of nowhere.
            return

        if not success:
            logger.warning("motion %r did not complete successfully", motion_name)
            # Abandon the rest of the sequence: a failed grasp means there is
            # nothing to place, and retrying the place would fling an empty
            # gripper at the bin.
            self._release()
            return

        logger.info("motion %r finished", motion_name)
        if self._pending_steps:
            next_step = self._pending_steps.pop(0)
            color = self._active_color or "?"
            self._play(next_step, color)
            return

        self._release()

    def _release(self) -> None:
        """Drop any pending sequence and start the policy's cooldown."""
        self._pending_steps = []
        self._active_color = None
        self._deadline = None
        self._policy.motion_finished(self._clock())
