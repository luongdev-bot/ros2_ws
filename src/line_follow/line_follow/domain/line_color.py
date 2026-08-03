"""Line colour expressed as a box in OpenCV 8-bit LAB space."""

from dataclasses import dataclass
from typing import Tuple

from .errors import InvalidLineConfigError

#: A LAB triple as OpenCV stores it.
LabTriple = Tuple[int, int, int]

_CHANNEL_NAMES = ("L", "a", "b")


@dataclass(frozen=True)
class LineColorRange:
    """Inclusive LAB bounds for the single line colour."""

    lab_min: LabTriple
    lab_max: LabTriple
    min_area_px: int = 80

    def __post_init__(self) -> None:
        for bound, label in ((self.lab_min, "lab_min"), (self.lab_max, "lab_max")):
            try:
                channel_count = len(bound)
            except TypeError:
                raise InvalidLineConfigError(
                    f"{label} must have 3 channels, got a non-sequence value"
                ) from None
            if channel_count != 3:
                raise InvalidLineConfigError(
                    f"{label} must have 3 channels, got {channel_count}"
                )
            for channel, value in zip(_CHANNEL_NAMES, bound):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise InvalidLineConfigError(
                        f"{label}.{channel} must be an integer, got {value!r}"
                    )
                if not 0 <= value <= 255:
                    raise InvalidLineConfigError(
                        f"{label}.{channel} = {value} is outside 0..255"
                    )

        for channel, low, high in zip(_CHANNEL_NAMES, self.lab_min, self.lab_max):
            if low > high:
                raise InvalidLineConfigError(
                    f"{channel} lower bound {low} exceeds upper bound {high}"
                )

        if isinstance(self.min_area_px, bool) or not isinstance(
            self.min_area_px, int
        ):
            raise InvalidLineConfigError(
                f"min_area_px must be an integer, got {self.min_area_px!r}"
            )
        if self.min_area_px < 0:
            raise InvalidLineConfigError(
                f"min_area_px must not be negative, got {self.min_area_px}"
            )
