"""Config parsing must reject malformed input with path-rich errors."""

import pytest
import yaml

from line_follow.domain.detection import RoiBand
from line_follow.domain.errors import InvalidLineConfigError
from line_follow.domain.line_color import LineColorRange
from line_follow.domain.pid import PidGains
from line_follow.infrastructure.config_loader import load_line_follow_config

GOOD = {
    "color": {
        "lab_min": [0, 120, 120],
        "lab_max": [20, 136, 136],
        "min_area_px": 40,
    },
    "rois": [
        {
            "y_min": 50,
            "y_max": 90,
            "x_min": 0,
            "x_max": 120,
            "weight": 1.0,
        },
    ],
    "pid": {"kp": 0.01, "ki": 0.0, "kd": 0.001},
}


def write(tmp_path, data):
    path = tmp_path / "line_follow.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return str(path)


def test_valid_config_round_trips_to_domain_values(tmp_path):
    path = write(tmp_path, GOOD)

    color, rois, gains = load_line_follow_config(path)

    assert color == LineColorRange(
        (0, 120, 120),
        (20, 136, 136),
        min_area_px=40,
    )
    assert rois == [RoiBand(50, 90, 0, 120, 1.0)]
    assert gains == PidGains(kp=0.01, ki=0.0, kd=0.001)


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_line_follow_config(str(tmp_path / "missing.yaml"))


@pytest.mark.parametrize(
    "data",
    [
        {"rois": GOOD["rois"], "pid": GOOD["pid"]},
        {"color": [], "rois": GOOD["rois"], "pid": GOOD["pid"]},
        {
            "color": {"lab_max": [20, 136, 136], "min_area_px": 40},
            "rois": GOOD["rois"],
            "pid": GOOD["pid"],
        },
        {
            "color": dict(GOOD["color"], min_area_px=1.5),
            "rois": GOOD["rois"],
            "pid": GOOD["pid"],
        },
    ],
)
def test_malformed_color_names_path_in_error(tmp_path, data):
    path = write(tmp_path, data)

    with pytest.raises(InvalidLineConfigError, match=str(path)):
        load_line_follow_config(path)


@pytest.mark.parametrize(
    "data",
    [
        {"color": GOOD["color"], "pid": GOOD["pid"]},
        {"color": GOOD["color"], "rois": {}, "pid": GOOD["pid"]},
        {
            "color": GOOD["color"],
            "rois": [{"y_min": 0}],
            "pid": GOOD["pid"],
        },
        {
            "color": GOOD["color"],
            "rois": [dict(GOOD["rois"][0], weight=1.5)],
            "pid": GOOD["pid"],
        },
    ],
)
def test_malformed_rois_name_path_in_error(tmp_path, data):
    path = write(tmp_path, data)

    with pytest.raises(InvalidLineConfigError, match=str(path)):
        load_line_follow_config(path)


@pytest.mark.parametrize(
    "data",
    [
        {"color": GOOD["color"], "rois": GOOD["rois"]},
        {"color": GOOD["color"], "rois": GOOD["rois"], "pid": []},
        {
            "color": GOOD["color"],
            "rois": GOOD["rois"],
            "pid": {"kp": 0.01, "ki": 0.0},
        },
        {
            "color": GOOD["color"],
            "rois": GOOD["rois"],
            "pid": {"kp": "fast", "ki": 0.0, "kd": 0.0},
        },
    ],
)
def test_malformed_pid_names_path_in_error(tmp_path, data):
    path = write(tmp_path, data)

    with pytest.raises(InvalidLineConfigError, match=str(path)):
        load_line_follow_config(path)
