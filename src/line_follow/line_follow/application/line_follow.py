"""Turn line detections into planar steering commands."""

from dataclasses import replace
from typing import Callable, Optional

from ..domain.detection import LineDetection
from ..domain.pid import PID
from ..domain.ports import LineDetector
from ..domain.steering import SteeringCommand


class LineFollowUseCase:
    """Apply PID steering and lost-line behavior to each camera frame."""

    def __init__(
        self,
        detector: LineDetector,
        pid: PID,
        *,
        cruise_speed: float,
        max_angular_speed: float,
        min_speed_scale: float = 0.4,
        lost_line_timeout_s: float = 0.4,
        clock: Callable[[], float],
    ) -> None:
        if cruise_speed < 0:
            raise ValueError(
                f"cruise_speed must be non-negative, got {cruise_speed}"
            )
        if max_angular_speed <= 0:
            raise ValueError(
                f"max_angular_speed must be positive, got {max_angular_speed}"
            )
        if not 0 < min_speed_scale <= 1:
            raise ValueError(
                f"min_speed_scale must be in (0, 1], got {min_speed_scale}"
            )
        if lost_line_timeout_s < 0:
            raise ValueError(
                "lost_line_timeout_s must be non-negative, "
                f"got {lost_line_timeout_s}"
            )

        self._detector = detector
        self._pid = pid
        self._cruise_speed = cruise_speed
        self._max_angular_speed = max_angular_speed
        self._min_speed_scale = min_speed_scale
        self._lost_line_timeout_s = lost_line_timeout_s
        self._clock = clock
        self._last_time: Optional[float] = None
        self._last_seen_time: Optional[float] = None
        self._last_detection: Optional[LineDetection] = None
        self._last_command = SteeringCommand(0.0, 0.0, line_found=False)

    @property
    def last_detection(self) -> Optional[LineDetection]:
        """Detection computed by the most recent :meth:`process_frame` call."""
        return self._last_detection

    def reset(self) -> None:
        """Clear timing, coasting, and PID state for a fresh acquisition."""
        self._last_time = None
        self._last_seen_time = None
        self._last_command = SteeringCommand(0.0, 0.0, line_found=False)
        self._pid.reset()

    def process_frame(self, frame, frame_width: float) -> SteeringCommand:
        """Compute one steering command from a camera frame."""
        now = self._clock()
        dt = now - self._last_time if self._last_time is not None else 0.0
        self._last_time = now

        detection = self._detector.detect(frame)
        self._last_detection = detection

        if detection is not None:
            self._last_seen_time = now
            error = (frame_width / 2.0) - detection.center_x
            angular_z = self._pid.update(error, dt)
            speed_scale = max(
                self._min_speed_scale,
                1.0 - abs(angular_z) / self._max_angular_speed,
            )
            linear_x = self._cruise_speed * speed_scale
            command = SteeringCommand(
                linear_x,
                angular_z,
                line_found=True,
            )
            self._last_command = command
            return command

        elapsed = (
            None
            if self._last_seen_time is None
            else now - self._last_seen_time
        )
        if (
            elapsed is None
            or elapsed < 0.0
            or elapsed > self._lost_line_timeout_s
        ):
            self._pid.reset()
            return SteeringCommand(0.0, 0.0, line_found=False)

        return replace(self._last_command, line_found=False)
