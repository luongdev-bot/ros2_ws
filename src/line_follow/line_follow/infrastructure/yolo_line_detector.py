"""Ultralytics YOLO segmentation detector for a camera line."""

import os
from typing import Optional

import cv2
import numpy as np

from ..domain.detection import LineDetection
from ..domain.ports import LineDetector


class YoloLineDetector(LineDetector):
    """Find a line from the highest-confidence YOLO segmentation instance."""

    def __init__(
        self,
        model_path: str,
        *,
        confidence_threshold: float = 0.5,
        device: str = "0",
        imgsz: int = 640,
    ) -> None:
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"line-follow YOLO model not found: {model_path}"
            )
        if not 0.0 < confidence_threshold <= 1.0:
            raise ValueError(
                "confidence_threshold must be in (0.0, 1.0], "
                f"got {confidence_threshold}"
            )
        if isinstance(imgsz, bool) or not isinstance(imgsz, int) or imgsz <= 0:
            raise ValueError(f"imgsz must be a positive integer, got {imgsz}")

        # Keep torch and ultralytics optional for the package's system-Python
        # LAB node. This detector is instantiated only by the YOLO node.
        import torch  # noqa: F401
        from ultralytics import YOLO

        self._model = YOLO(model_path)
        self._confidence_threshold = confidence_threshold
        self._device = device
        self._imgsz = imgsz
        self._last_polygon: Optional[np.ndarray] = None

    @property
    def last_polygon(self) -> Optional[np.ndarray]:
        """Polygon from the most recent :meth:`detect` call, if any."""
        return self._last_polygon

    def detect(self, frame: np.ndarray) -> Optional[LineDetection]:
        if frame is None or frame.size == 0:
            self._last_polygon = None
            return None

        results = self._model.predict(
            source=frame,
            conf=self._confidence_threshold,
            device=self._device,
            imgsz=self._imgsz,
            verbose=False,
        )
        result = results[0]
        if result.masks is None or len(result.masks.xy) == 0:
            self._last_polygon = None
            return None

        confidences = result.boxes.conf.detach().cpu().numpy()
        best = int(confidences.argmax())
        polygon = result.masks.xy[best]
        if polygon.size == 0:
            self._last_polygon = None
            return None

        self._last_polygon = polygon
        moments = cv2.moments(polygon)
        if abs(moments["m00"]) > 1e-6:
            center_x = float(moments["m10"] / moments["m00"])
        else:
            center_x = float(polygon[:, 0].mean())
        confidence = float(min(max(confidences[best], 0.0), 1.0))
        return LineDetection(center_x=center_x, confidence=confidence)


def draw_debug(
    frame: np.ndarray,
    detection: Optional[LineDetection],
    polygon: Optional[np.ndarray],
) -> np.ndarray:
    """Annotate a copy of ``frame`` with the YOLO line estimate."""
    annotated = frame.copy()
    if polygon is not None and polygon.size > 0:
        cv2.polylines(
            annotated,
            [polygon.astype(np.int32)],
            isClosed=True,
            color=(0, 255, 255),
            thickness=2,
        )

    if detection is not None:
        center = (int(round(detection.center_x)), frame.shape[0] // 2)
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
