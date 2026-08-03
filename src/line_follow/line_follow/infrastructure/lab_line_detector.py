"""OpenCV LAB-threshold detector for a line in weighted ROI bands."""

from typing import Optional, Sequence

import cv2
import numpy as np

from ..domain.detection import LineDetection, RoiBand
from ..domain.line_color import LineColorRange
from ..domain.ports import LineDetector


class LabLineDetector(LineDetector):
    """Find and combine the largest line blob in each configured ROI band."""

    def __init__(
        self,
        color: LineColorRange,
        rois: Sequence[RoiBand],
        *,
        use_cuda: bool = False,
    ) -> None:
        if not rois:
            raise ValueError("rois must define at least one ROI band")
        self._color = color
        self._rois = tuple(rois)
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (3, 3)
        )
        self._lab_min = np.array(color.lab_min, dtype=np.uint8)
        self._lab_max = np.array(color.lab_max, dtype=np.uint8)
        self._use_cuda = use_cuda
        if self._use_cuda:
            try:
                cuda_device_count = cv2.cuda.getCudaEnabledDeviceCount()
            except Exception as exc:
                raise ValueError(
                    "use_cuda=True but no CUDA device is available "
                    "(cv2 built without CUDA support, or the CUDA-enabled "
                    "cv2 build is not on PYTHONPATH)"
                ) from exc
            if cuda_device_count <= 0:
                raise ValueError(
                    "use_cuda=True but no CUDA device is available "
                    "(cv2 built without CUDA support, or the CUDA-enabled "
                    "cv2 build is not on PYTHONPATH)"
                )
            self._cuda_morph_filter = cv2.cuda.createMorphologyFilter(
                cv2.MORPH_OPEN, cv2.CV_8UC1, self._morph_kernel
            )

    @property
    def rois(self) -> Sequence[RoiBand]:
        """Configured ROI bands, in their original order."""
        return self._rois

    def detect(self, frame: np.ndarray) -> Optional[LineDetection]:
        if frame is None or frame.size == 0:
            return None

        weighted_center_sum = 0.0
        hit_weight_sum = 0.0
        hit_count = 0

        for band in self._rois:
            crop = frame[
                band.y_min:band.y_max,
                band.x_min:band.x_max,
            ]
            if crop.size == 0:
                continue

            if self._use_cuda:
                gpu_crop = cv2.cuda_GpuMat()
                gpu_crop.upload(crop)
                gpu_lab = cv2.cuda.cvtColor(gpu_crop, cv2.COLOR_BGR2LAB)
                gpu_mask = cv2.cuda.inRange(
                    gpu_lab, self._color.lab_min, self._color.lab_max
                )
                opened = self._cuda_morph_filter.apply(gpu_mask).download()
            else:
                lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
                mask = cv2.inRange(lab, self._lab_min, self._lab_max)
                # Opening: remove isolated threshold noise while retaining the line.
                eroded = cv2.erode(mask, self._morph_kernel)
                opened = cv2.dilate(eroded, self._morph_kernel)

            contours = cv2.findContours(
                opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )[-2]
            candidates = [
                (cv2.contourArea(contour), contour)
                for contour in contours
                if cv2.contourArea(contour) >= self._color.min_area_px
            ]
            if not candidates:
                continue

            _, biggest = max(candidates, key=lambda pair: pair[0])
            x, _, width, _ = cv2.boundingRect(biggest)
            center_x = band.x_min + x + width / 2.0

            hit_count += 1
            weighted_center_sum += center_x * band.weight
            hit_weight_sum += band.weight

        if hit_count == 0 or hit_weight_sum <= 0.0:
            return None

        return LineDetection(
            center_x=weighted_center_sum / hit_weight_sum,
            confidence=hit_count / len(self._rois),
        )


def draw_debug(
    frame: np.ndarray,
    rois: Sequence[RoiBand],
    detection: Optional[LineDetection],
) -> np.ndarray:
    """Annotate a copy of ``frame`` with ROI bands and the line estimate."""
    annotated = frame.copy()
    for band in rois:
        cv2.rectangle(
            annotated,
            (band.x_min, band.y_min),
            (band.x_max, band.y_max),
            (0, 255, 255),
            2,
        )

    if detection is not None and rois:
        strongest_band = max(rois, key=lambda band: band.weight)
        center_y = (strongest_band.y_min + strongest_band.y_max) // 2
        center = (int(round(detection.center_x)), center_y)
        cv2.circle(annotated, center, 5, (0, 255, 255), -1)
        cv2.putText(
            annotated,
            f"line {detection.confidence:.0%}",
            (center[0] - 40, center[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated
