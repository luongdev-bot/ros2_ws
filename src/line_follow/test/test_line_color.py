"""Line-colour validation tests."""

import pytest

from line_follow.domain.errors import InvalidLineConfigError
from line_follow.domain.line_color import LineColorRange


class TestLineColorRangeValidation:
    def test_rejects_inverted_bounds(self):
        with pytest.raises(InvalidLineConfigError, match="exceeds upper bound"):
            LineColorRange((200, 0, 0), (100, 255, 255))

    def test_rejects_out_of_gamut_values(self):
        with pytest.raises(InvalidLineConfigError, match="outside 0..255"):
            LineColorRange((0, 0, 0), (300, 255, 255))

    def test_rejects_wrong_channel_count(self):
        with pytest.raises(InvalidLineConfigError, match="3 channels"):
            LineColorRange((0, 0), (255, 255, 255))

    def test_rejects_negative_minimum_area(self):
        with pytest.raises(InvalidLineConfigError, match="must not be negative"):
            LineColorRange((0, 0, 0), (255, 255, 255), min_area_px=-1)
