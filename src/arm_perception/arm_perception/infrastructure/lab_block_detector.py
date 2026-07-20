"""OpenCV LAB-threshold block detector.

The pipeline mirrors the one Hiwonder's `automatic_pick.py` uses on the
real JetRover — downscale, blur, LAB threshold, open (erode+dilate),
largest contour, minAreaRect — because it is known to work on this
robot's camera. Differences here: every colour is scanned in one pass,
results come back as domain objects, and geometry is mapped back to
full-resolution pixels so downstream consumers never need to know a
downscale happened.
"""

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from ..domain.color_range import ColorPalette, ColorRange
from ..domain.detection import BlockDetection
from ..domain.ports import BlockDetector

#: Working resolution for masking. Small enough to stay real-time on the
#: Jetson, large enough that a 4 cm block is still tens of pixels across.
DEFAULT_PROC_SIZE = (320, 240)


class LabBlockDetector(BlockDetector):
    """Finds the largest blob of each configured colour in a BGR frame."""

    def __init__(
        self,
        palette: ColorPalette,
        *,
        proc_size: Tuple[int, int] = DEFAULT_PROC_SIZE,
        blur_ksize: int = 3,
        morph_ksize: int = 3,
    ) -> None:
        if proc_size[0] <= 0 or proc_size[1] <= 0:
            raise ValueError(f"proc_size must be positive, got {proc_size}")
        if blur_ksize % 2 == 0 or blur_ksize < 1:
            raise ValueError(f"blur_ksize must be a positive odd number, got {blur_ksize}")
        if morph_ksize < 1:
            raise ValueError(f"morph_ksize must be >= 1, got {morph_ksize}")

        self._palette = palette
        self._proc_size = proc_size
        self._blur_ksize = blur_ksize
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_ksize, morph_ksize)
        )

    @property
    def palette(self) -> ColorPalette:
        return self._palette

    def detect(self, frame: np.ndarray) -> Sequence[BlockDetection]:
        if frame is None or frame.size == 0:
            return []

        frame_h, frame_w = frame.shape[:2]
        scale_x = frame_w / self._proc_size[0]
        scale_y = frame_h / self._proc_size[1]

        small = cv2.resize(frame, self._proc_size, interpolation=cv2.INTER_NEAREST)
        blurred = cv2.GaussianBlur(small, (self._blur_ksize, self._blur_ksize), 3)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

        detections: List[BlockDetection] = []
        for color_range in self._palette:
            detection = self._detect_one(lab, color_range, scale_x, scale_y)
            if detection is not None:
                detections.append(detection)
        return detections

    def sample_lab(self, frame: np.ndarray, x: int, y: int) -> Tuple[int, int, int]:
        """LAB value of one full-resolution pixel.

        Exposed for threshold tuning: click a block, read its LAB, widen
        the configured range around it.
        """
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        pixel = lab[int(y), int(x)]
        return (int(pixel[0]), int(pixel[1]), int(pixel[2]))

    def _detect_one(
        self,
        lab: np.ndarray,
        color_range: ColorRange,
        scale_x: float,
        scale_y: float,
    ) -> Optional[BlockDetection]:
        mask = cv2.inRange(
            lab, np.array(color_range.lab_min, dtype=np.uint8),
            np.array(color_range.lab_max, dtype=np.uint8),
        )
        # Opening: drop salt noise without shrinking the block itself.
        eroded = cv2.erode(mask, self._morph_kernel)
        opened = cv2.dilate(eroded, self._morph_kernel)

        # findContours returns (contours, hierarchy) on OpenCV 4 but
        # (image, contours, hierarchy) on 3.x; [-2] picks contours on both.
        contours = cv2.findContours(
            opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        if not contours:
            return None

        biggest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(biggest))
        if area < color_range.min_area_px:
            return None

        (cx, cy), (w, h), angle = cv2.minAreaRect(biggest)

        return BlockDetection(
            color=color_range.name,
            center_x=float(cx) * scale_x,
            center_y=float(cy) * scale_y,
            width=float(w) * scale_x,
            height=float(h) * scale_y,
            angle_deg=float(angle),
            # Area scales with both axes, so the full-res area is the
            # processing-frame area times each scale factor.
            area_px=area * scale_x * scale_y,
        )


def draw_detections(
    frame: np.ndarray, detections: Sequence[BlockDetection]
) -> np.ndarray:
    """Annotate a copy of ``frame`` with boxes and labels, for the debug topic."""
    annotated = frame.copy()
    for det in detections:
        box = cv2.boxPoints(
            ((det.center_x, det.center_y), (det.width, det.height), det.angle_deg)
        )
        cv2.drawContours(annotated, [np.intp(box)], -1, (0, 255, 255), 2)
        cv2.circle(
            annotated, (int(det.center_x), int(det.center_y)), 4, (0, 255, 255), -1
        )
        cv2.putText(
            annotated,
            f"{det.color} {int(det.area_px)}px",
            (int(det.center_x) - 40, int(det.center_y) - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated
