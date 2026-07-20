"""What the camera found: one coloured block in one frame."""

from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class BlockDetection:
    """A single coloured block located in image space.

    Coordinates are pixels in the **full-resolution** frame, even though
    the mask is computed on a downscaled copy — the adapter maps them
    back before constructing this. ``angle_deg`` is the rotation of the
    minimum-area rectangle, in the range OpenCV returns (-90, 0].
    """

    color: str
    center_x: float
    center_y: float
    width: float
    height: float
    angle_deg: float
    area_px: float

    def __post_init__(self) -> None:
        if self.width < 0 or self.height < 0:
            raise ValueError(
                f"detection of {self.color} has negative size "
                f"({self.width} x {self.height})"
            )
        if self.area_px < 0:
            raise ValueError(f"detection of {self.color} has negative area {self.area_px}")

    def offset_from(self, x: float, y: float) -> "tuple[float, float]":
        """Vector from a reference point (usually the image centre) to this block.

        The pick policy uses this to decide whether the block is close
        enough to the frame centre for the stored action group - which
        replays a fixed trajectory - to actually reach it.
        """
        return (self.center_x - x, self.center_y - y)


def largest(detections: Sequence[BlockDetection]) -> Optional[BlockDetection]:
    """The biggest detection, or None if there were none.

    Biggest is a decent proxy for closest, which is the block the arm
    should deal with first.
    """
    if not detections:
        return None
    return max(detections, key=lambda d: d.area_px)
