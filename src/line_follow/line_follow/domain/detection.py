"""Line-detection domain values."""

from dataclasses import dataclass
import math

from .errors import InvalidLineConfigError


@dataclass(frozen=True)
class RoiBand:
    """One weighted horizontal image region used to find the line."""

    y_min: int
    y_max: int
    x_min: int
    x_max: int
    weight: float

    def __post_init__(self) -> None:
        for field_name in ("y_min", "y_max", "x_min", "x_max"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise InvalidLineConfigError(
                    f"{field_name} must be an integer, got {value!r}"
                )
            if value < 0:
                raise InvalidLineConfigError(
                    f"{field_name} must not be negative, got {value}"
                )
        if self.y_min >= self.y_max:
            raise InvalidLineConfigError(
                f"y_min ({self.y_min}) must be less than y_max ({self.y_max})"
            )
        if self.x_min >= self.x_max:
            raise InvalidLineConfigError(
                f"x_min ({self.x_min}) must be less than x_max ({self.x_max})"
            )
        if isinstance(self.weight, bool) or not isinstance(
            self.weight, (int, float)
        ):
            raise InvalidLineConfigError(
                f"weight must be a number, got {self.weight!r}"
            )
        if not math.isfinite(float(self.weight)) or not 0.0 <= self.weight <= 1.0:
            raise InvalidLineConfigError(
                f"weight must be in 0..1, got {self.weight}"
            )


@dataclass(frozen=True)
class LineDetection:
    """Weighted horizontal position of a line in full-frame pixels."""

    center_x: float
    confidence: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.center_x):
            raise ValueError(f"center_x must be finite, got {self.center_x}")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError(
                f"confidence must be in 0..1, got {self.confidence}"
            )
