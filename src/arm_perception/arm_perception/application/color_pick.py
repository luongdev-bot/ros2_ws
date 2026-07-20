"""Use case: see a coloured block, play the matching stored action group.

This is the whole behaviour in one place — detect, debounce, dispatch —
with every external concern (camera, OpenCV, the arm's action server)
behind a port. It can be exercised end to end with fakes.
"""

import logging
from typing import Any, Callable, Optional, Sequence, Tuple

from ..domain.detection import BlockDetection, largest
from ..domain.errors import MotionPlaybackError, UnknownColorError
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
    ) -> None:
        self._detector = detector
        self._policy = policy
        self._motion_map = motion_map
        self._player = player
        self._clock = clock
        self._detection_sink = detection_sink
        self._enabled = True

        self._player.on_finished(self._handle_motion_finished)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        """Resume acting on detections."""
        if not self._enabled:
            self._enabled = True
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
        try:
            motion = self._motion_map.motion_for(color)
            duration_ms = self._motion_map.duration_for(color)
        except UnknownColorError as exc:
            # The policy already moved to PLAYING; release it, otherwise
            # an unmapped colour would wedge the pipeline forever.
            logger.warning("%s", exc)
            self._policy.motion_finished(self._clock())
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason=str(exc),
            )

        try:
            logger.info("playing %r for %s block", motion, color)
            self._player.play(motion, duration_ms=duration_ms)
        except MotionPlaybackError as exc:
            logger.error("could not play %r: %s", motion, exc)
            self._policy.motion_finished(self._clock())
            return PickDecision(
                should_play=False, color=color, detection=decision.detection,
                reason=f"playback failed: {exc}",
            )

        return decision

    def _handle_motion_finished(self, motion_name: str, success: bool) -> None:
        if success:
            logger.info("motion %r finished", motion_name)
        else:
            logger.warning("motion %r did not complete successfully", motion_name)
        self._policy.motion_finished(self._clock())
