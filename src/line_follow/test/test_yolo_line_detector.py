"""YOLO line detector validation and optional end-to-end tests."""

import math
from pathlib import Path

import cv2
import pytest

from line_follow.domain.detection import LineDetection
from line_follow.infrastructure.yolo_line_detector import YoloLineDetector


def test_rejects_a_missing_model_before_importing_ultralytics(tmp_path):
    missing_model = tmp_path / "missing.pt"

    with pytest.raises(FileNotFoundError, match="line-follow YOLO model not found"):
        YoloLineDetector(str(missing_model))


@pytest.mark.parametrize("confidence_threshold", [0.0, -0.1, 1.1])
def test_rejects_out_of_range_confidence_before_importing_ultralytics(
    tmp_path, confidence_threshold
):
    model_path = tmp_path / "model.pt"
    model_path.touch()

    with pytest.raises(ValueError, match="confidence_threshold"):
        YoloLineDetector(
            str(model_path), confidence_threshold=confidence_threshold
        )


@pytest.mark.parametrize("imgsz", [0, -1, True, 640.0])
def test_rejects_non_positive_or_non_integer_imgsz_before_importing_ultralytics(
    tmp_path, imgsz
):
    model_path = tmp_path / "model.pt"
    model_path.touch()

    with pytest.raises(ValueError, match="imgsz"):
        YoloLineDetector(str(model_path), imgsz=imgsz)


def test_detects_real_image_when_ultralytics_is_available():
    pytest.importorskip("ultralytics")

    model_path = (
        Path(__file__).resolve().parents[1]
        / "models"
        / "line_yolo11n_seg.pt"
    )
    image_path = Path(
        "/home/luong/line AI/dataset/line/images/"
        "img_0000_line_20260728_221909.jpg"
    )
    image = cv2.imread(str(image_path))
    if image is None:
        pytest.skip(f"dataset image is unavailable: {image_path}")

    detection = YoloLineDetector(str(model_path), device="cpu").detect(image)

    assert detection is not None
    assert isinstance(detection, LineDetection)
    assert 0.0 <= detection.confidence <= 1.0
    assert math.isfinite(detection.center_x)
    assert 0.0 <= detection.center_x < image.shape[1]
