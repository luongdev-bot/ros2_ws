"""Config parsing — malformed input must fail loudly, not silently."""

import pytest
import yaml

from arm_perception.domain.errors import InvalidColorRangeError, UnknownColorError
from arm_perception.infrastructure.config_loader import load_color_pick_config

GOOD = {
    "colors": {
        "red": {
            "lab_min": [40, 150, 140],
            "lab_max": [255, 255, 255],
            "min_area_px": 200,
            "motion": "pick",
            "duration_ms": 0,
        },
        "blue": {
            "lab_min": [20, 120, 0],
            "lab_max": [255, 215, 105],
            "motion": "place_left",
            "duration_ms": 1500,
        },
    }
}


def write(tmp_path, data):
    path = tmp_path / "color_pick.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_loads_palette_and_bindings(tmp_path):
    palette, motion_map = load_color_pick_config(write(tmp_path, GOOD))

    assert set(palette.names()) == {"red", "blue"}
    assert motion_map.motion_for("red") == "pick"
    assert motion_map.duration_for("blue") == 1500


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_color_pick_config(str(tmp_path / "nope.yaml"))


def test_empty_motion_is_rejected(tmp_path):
    """An empty string is a typo; skipping it would hide the bug."""
    data = {"colors": {"red": dict(GOOD["colors"]["red"], motion="")}}

    with pytest.raises(InvalidColorRangeError, match="empty 'motion'"):
        load_color_pick_config(write(tmp_path, data))


def test_whitespace_motion_is_rejected(tmp_path):
    data = {"colors": {"red": dict(GOOD["colors"]["red"], motion="   ")}}

    with pytest.raises(InvalidColorRangeError, match="empty 'motion'"):
        load_color_pick_config(write(tmp_path, data))


def test_omitting_motion_makes_a_colour_detect_only(tmp_path):
    red_only = {k: v for k, v in GOOD["colors"]["red"].items() if k != "motion"}
    data = {"colors": {"red": red_only, "blue": GOOD["colors"]["blue"]}}

    palette, motion_map = load_color_pick_config(write(tmp_path, data))

    assert "red" in palette.names()          # still detected
    with pytest.raises(UnknownColorError):   # but never triggers a motion
        motion_map.motion_for("red")


def test_config_with_no_motion_at_all_is_rejected(tmp_path):
    red_only = {k: v for k, v in GOOD["colors"]["red"].items() if k != "motion"}
    data = {"colors": {"red": red_only}}

    with pytest.raises(InvalidColorRangeError, match="nothing could ever be picked"):
        load_color_pick_config(write(tmp_path, data))


def test_negative_and_non_integer_duration_are_rejected(tmp_path):
    negative = {"colors": {"red": dict(GOOD["colors"]["red"], duration_ms=-5)}}
    with pytest.raises(InvalidColorRangeError, match="negative 'duration_ms'"):
        load_color_pick_config(write(tmp_path, negative))

    bogus = {"colors": {"red": dict(GOOD["colors"]["red"], duration_ms="soon")}}
    with pytest.raises(InvalidColorRangeError, match="non-integer 'duration_ms'"):
        load_color_pick_config(write(tmp_path, bogus))


def test_missing_lab_bounds_are_reported_by_name(tmp_path):
    data = {"colors": {"red": {"lab_max": [255, 255, 255], "motion": "pick"}}}

    with pytest.raises(InvalidColorRangeError, match="lab_min"):
        load_color_pick_config(write(tmp_path, data))


def test_inverted_bounds_are_rejected(tmp_path):
    data = {"colors": {"red": dict(GOOD["colors"]["red"], lab_min=[250, 150, 140],
                                   lab_max=[100, 255, 255])}}

    with pytest.raises(InvalidColorRangeError):
        load_color_pick_config(write(tmp_path, data))


def test_empty_or_missing_colors_section_is_rejected(tmp_path):
    with pytest.raises(InvalidColorRangeError, match="non-empty 'colors'"):
        load_color_pick_config(write(tmp_path, {"colors": {}}))
    with pytest.raises(InvalidColorRangeError, match="non-empty 'colors'"):
        load_color_pick_config(write(tmp_path, {"something_else": 1}))


def test_shipped_config_is_valid():
    """The config we actually install must load."""
    from ament_index_python.packages import get_package_share_directory
    import os

    path = os.path.join(
        get_package_share_directory("arm_perception"), "config", "color_pick.yaml"
    )
    palette, motion_map = load_color_pick_config(path)

    assert len(palette) >= 1
    for color in palette.names():
        assert motion_map.motion_for(color)


def test_non_string_motion_is_rejected(tmp_path):
    """`motion: 0` is a typo, not an action group named "0"."""
    for bogus in (0, [1, 2], {"a": 1}, True):
        data = {"colors": {"red": dict(GOOD["colors"]["red"], motion=bogus)}}
        with pytest.raises(InvalidColorRangeError, match="non-string 'motion'"):
            load_color_pick_config(write(tmp_path, data))


def test_non_integer_min_area_is_reported_as_a_config_error(tmp_path):
    data = {"colors": {"red": dict(GOOD["colors"]["red"], min_area_px="big")}}

    with pytest.raises(InvalidColorRangeError, match="non-integer 'min_area_px'"):
        load_color_pick_config(write(tmp_path, data))
