"""LAB line detector tests against synthetic BGR frames."""

from unittest.mock import patch

import cv2
import numpy as np
import pytest

from line_follow.domain.detection import RoiBand
from line_follow.domain.line_color import LineColorRange
from line_follow.infrastructure.lab_line_detector import (
    LabLineDetector,
    draw_debug,
)

BGR_BLACK = (0, 0, 0)
BGR_ASYMMETRIC = (40, 60, 180)


def frame_with(patches, size=(100, 120)):
    """Grey canvas with filled rectangles: [(bgr, x0, y0, x1, y1), ...]."""
    frame = np.full((size[0], size[1], 3), 128, dtype=np.uint8)
    for bgr, x0, y0, x1, y1 in patches:
        frame[y0:y1, x0:x1] = bgr
    return frame


def black_range():
    return LineColorRange(
        lab_min=(0, 120, 120),
        lab_max=(20, 136, 136),
        min_area_px=20,
    )


def asymmetric_range():
    return LineColorRange(
        # BGR_ASYMMETRIC converts to LAB (109, 176, 166); RGB channel order
        # instead produces (81, 164, 62), outside this deliberately narrow range.
        lab_min=(100, 170, 160),
        lab_max=(115, 182, 172),
        min_area_px=20,
    )


def test_combines_band_centers_by_weights_of_hitting_bands():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=0, x_max=120, weight=0.75),
        RoiBand(y_min=10, y_max=40, x_min=0, x_max=120, weight=0.25),
    ]
    detector = LabLineDetector(black_range(), bands)
    frame = frame_with([
        (BGR_BLACK, 10, 55, 30, 85),   # center x ~= 20, near band
        (BGR_BLACK, 70, 15, 90, 35),   # center x ~= 80, far band
    ])

    detection = detector.detect(frame)

    assert detection is not None
    assert detection.center_x == pytest.approx(35.0, abs=2.0)
    assert detection.confidence == pytest.approx(1.0)


def test_center_is_mapped_back_to_full_frame_x_and_confidence_counts_hits():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=40, x_max=110, weight=0.6),
        RoiBand(y_min=10, y_max=40, x_min=0, x_max=120, weight=0.4),
    ]
    detector = LabLineDetector(black_range(), bands)
    frame = frame_with([
        (BGR_BLACK, 70, 55, 90, 85),
    ])

    detection = detector.detect(frame)

    assert detection is not None
    assert detection.center_x == pytest.approx(80.0, abs=2.0)
    assert detection.confidence == pytest.approx(0.5)


def test_returns_none_when_no_band_contains_target_colour():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=0, x_max=120, weight=1.0),
    ]
    detector = LabLineDetector(black_range(), bands)

    assert detector.detect(frame_with([])) is None


def test_cuda_requires_an_available_device():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=0, x_max=120, weight=1.0),
    ]

    with patch('cv2.cuda.getCudaEnabledDeviceCount', return_value=0):
        with pytest.raises(ValueError, match='use_cuda=True but no CUDA device'):
            LabLineDetector(black_range(), bands, use_cuda=True)


@pytest.mark.skipif(
    cv2.cuda.getCudaEnabledDeviceCount() == 0,
    reason='no CUDA device available',
)
def test_cuda_matches_cpu_detection():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=0, x_max=120, weight=0.75),
        RoiBand(y_min=10, y_max=40, x_min=0, x_max=120, weight=0.25),
    ]
    frame = frame_with([
        (BGR_ASYMMETRIC, 10, 55, 30, 85),
        (BGR_ASYMMETRIC, 70, 15, 90, 35),
    ])

    cpu_detection = LabLineDetector(asymmetric_range(), bands).detect(frame)
    cuda_detection = LabLineDetector(
        asymmetric_range(), bands, use_cuda=True
    ).detect(frame)

    assert cpu_detection is not None
    assert cuda_detection is not None
    assert cuda_detection.center_x == pytest.approx(
        # Shared contour extraction operates on the same binary mask; permit
        # only half a pixel for backend-specific morphology rounding.
        cpu_detection.center_x, abs=0.5
    )
    assert cuda_detection.confidence == pytest.approx(
        cpu_detection.confidence
    )


def test_debug_annotation_does_not_mutate_source():
    bands = [
        RoiBand(y_min=50, y_max=90, x_min=0, x_max=120, weight=1.0),
    ]
    detector = LabLineDetector(black_range(), bands)
    frame = frame_with([(BGR_BLACK, 50, 55, 70, 85)])
    original = frame.copy()

    annotated = draw_debug(frame, bands, detector.detect(frame))

    assert np.array_equal(frame, original)
    assert not np.array_equal(annotated, frame)
