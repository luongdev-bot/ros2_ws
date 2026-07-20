"""Detector and colour-range tests against synthetic frames."""

import numpy as np
import pytest

from arm_perception.domain.color_range import ColorPalette, ColorRange
from arm_perception.domain.errors import InvalidColorRangeError
from arm_perception.infrastructure.lab_block_detector import (
    LabBlockDetector,
    draw_detections,
)

# Pure BGR primaries, well inside the ranges below.
BGR_RED = (0, 0, 255)
BGR_BLUE = (255, 0, 0)


def palette():
    return ColorPalette.of(
        ColorRange("red", (40, 150, 140), (255, 255, 255), min_area_px=50),
        ColorRange("blue", (20, 120, 0), (255, 215, 105), min_area_px=50),
    )


def frame_with(patches, size=(480, 640)):
    """Grey canvas with filled rectangles: [(bgr, x0, y0, x1, y1), ...]."""
    img = np.full((size[0], size[1], 3), 128, dtype=np.uint8)
    for bgr, x0, y0, x1, y1 in patches:
        img[y0:y1, x0:x1] = bgr
    return img


class TestColorRangeValidation:
    def test_rejects_inverted_bounds(self):
        with pytest.raises(InvalidColorRangeError, match="exceeds upper bound"):
            ColorRange("red", (200, 0, 0), (100, 255, 255))

    def test_rejects_out_of_gamut_values(self):
        with pytest.raises(InvalidColorRangeError, match="outside 0..255"):
            ColorRange("red", (0, 0, 0), (300, 255, 255))

    def test_rejects_wrong_channel_count(self):
        with pytest.raises(InvalidColorRangeError, match="3 channels"):
            ColorRange("red", (0, 0), (255, 255, 255))

    def test_rejects_empty_name_and_negative_area(self):
        with pytest.raises(InvalidColorRangeError):
            ColorRange("", (0, 0, 0), (255, 255, 255))
        with pytest.raises(InvalidColorRangeError):
            ColorRange("red", (0, 0, 0), (255, 255, 255), min_area_px=-1)

    def test_contains_is_inclusive_at_the_bounds(self):
        rng = ColorRange("red", (10, 20, 30), (200, 210, 220))
        assert rng.contains((10, 20, 30))
        assert rng.contains((200, 210, 220))
        assert not rng.contains((9, 20, 30))

    def test_palette_rejects_key_name_mismatch(self):
        with pytest.raises(InvalidColorRangeError, match="does not match"):
            ColorPalette({"crimson": ColorRange("red", (0, 0, 0), (255, 255, 255))})

    def test_empty_palette_is_rejected(self):
        with pytest.raises(InvalidColorRangeError):
            ColorPalette({})


class TestDetection:
    def test_finds_a_red_patch(self):
        detector = LabBlockDetector(palette())
        frame = frame_with([(BGR_RED, 280, 200, 360, 280)])

        detections = {d.color: d for d in detector.detect(frame)}

        assert "red" in detections
        assert detections["red"].center_x == pytest.approx(320, abs=15)
        assert detections["red"].center_y == pytest.approx(240, abs=15)

    def test_separates_two_colours_in_one_frame(self):
        detector = LabBlockDetector(palette())
        frame = frame_with([
            (BGR_RED, 60, 200, 140, 280),
            (BGR_BLUE, 500, 200, 580, 280),
        ])

        found = {d.color: d for d in detector.detect(frame)}

        assert set(found) == {"red", "blue"}
        assert found["red"].center_x < found["blue"].center_x

    def test_grey_frame_yields_nothing(self):
        detector = LabBlockDetector(palette())
        assert detector.detect(frame_with([])) == []

    def test_patch_below_min_area_is_ignored(self):
        strict = ColorPalette.of(
            ColorRange("red", (40, 150, 140), (255, 255, 255), min_area_px=100000)
        )
        detector = LabBlockDetector(strict)

        assert detector.detect(frame_with([(BGR_RED, 300, 230, 320, 250)])) == []

    def test_geometry_is_reported_in_full_resolution_pixels(self):
        # Processing runs at 320x240; a patch at x~600 in a 640-wide frame
        # must come back near 600, not near 300.
        detector = LabBlockDetector(palette(), proc_size=(320, 240))
        frame = frame_with([(BGR_RED, 560, 400, 620, 460)])

        detection = detector.detect(frame)[0]

        assert detection.center_x == pytest.approx(590, abs=20)
        assert detection.center_y == pytest.approx(430, abs=20)

    def test_area_scales_with_both_axes(self):
        detector = LabBlockDetector(palette(), proc_size=(320, 240))
        # 160x160 px in a 640x480 frame -> 80x80 at processing size.
        frame = frame_with([(BGR_RED, 240, 160, 400, 320)])

        detection = detector.detect(frame)[0]

        assert detection.area_px == pytest.approx(160 * 160, rel=0.2)

    def test_empty_and_none_frames_are_handled(self):
        detector = LabBlockDetector(palette())
        assert detector.detect(None) == []
        assert detector.detect(np.empty((0, 0, 3), dtype=np.uint8)) == []

    def test_rejects_bad_construction(self):
        with pytest.raises(ValueError):
            LabBlockDetector(palette(), proc_size=(0, 240))
        with pytest.raises(ValueError):
            LabBlockDetector(palette(), blur_ksize=4)
        with pytest.raises(ValueError):
            LabBlockDetector(palette(), morph_ksize=0)

    def test_sample_lab_reads_a_pixel(self):
        detector = LabBlockDetector(palette())
        frame = frame_with([(BGR_RED, 300, 220, 340, 260)])

        lab = detector.sample_lab(frame, 320, 240)

        assert len(lab) == 3
        assert palette()["red"].contains(lab)


class TestDebugRendering:
    def test_draw_does_not_mutate_the_source_frame(self):
        detector = LabBlockDetector(palette())
        frame = frame_with([(BGR_RED, 280, 200, 360, 280)])
        original = frame.copy()

        draw_detections(frame, detector.detect(frame))

        assert np.array_equal(frame, original)

    def test_draw_marks_the_frame_when_there_is_a_detection(self):
        detector = LabBlockDetector(palette())
        frame = frame_with([(BGR_RED, 280, 200, 360, 280)])

        annotated = draw_detections(frame, detector.detect(frame))

        assert not np.array_equal(annotated, frame)
