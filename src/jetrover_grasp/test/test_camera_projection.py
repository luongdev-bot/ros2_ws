"""Tests for pure pinhole-camera projection."""

import math

import numpy as np
import pytest

from jetrover_grasp.domain.camera_projection import (
    deproject_pixel,
    intrinsics_from_camera_info_k,
)


IMAGE_WIDTH = 640
FX = (IMAGE_WIDTH / 2.0) / math.tan(1.089 / 2.0)
FY = FX
CX = 320.0
CY = 240.0


def test_principal_point_deprojects_along_optical_axis():
    depth = 0.75

    point = deproject_pixel(CX, CY, depth, FX, FY, CX, CY)

    np.testing.assert_allclose(point, np.array([0.0, 0.0, depth]))


def test_off_center_pixel_matches_hand_computed_point():
    u = CX + 80.0
    v = CY - 40.0
    depth = 1.25
    expected = np.array(
        [
            80.0 * depth / FX,
            -40.0 * depth / FY,
            depth,
        ]
    )

    point = deproject_pixel(u, v, depth, FX, FY, CX, CY)

    np.testing.assert_allclose(point, expected)


def test_intrinsics_are_parsed_from_row_major_camera_matrix():
    camera_matrix = [
        FX,
        0.0,
        CX,
        0.0,
        FY,
        CY,
        0.0,
        0.0,
        1.0,
    ]

    intrinsics = intrinsics_from_camera_info_k(camera_matrix)

    np.testing.assert_allclose(intrinsics, (FX, FY, CX, CY))


@pytest.mark.parametrize("index", [0, 4])
def test_non_positive_intrinsic_focal_lengths_raise(index):
    camera_matrix = [
        FX,
        0.0,
        CX,
        0.0,
        FY,
        CY,
        0.0,
        0.0,
        1.0,
    ]
    camera_matrix[index] = -camera_matrix[index]

    with pytest.raises(ValueError):
        intrinsics_from_camera_info_k(camera_matrix)


@pytest.mark.parametrize(
    "overrides",
    [
        {"fx": 0.0},
        {"fx": -FX},
        {"depth": 0.0},
        {"depth": -1.0},
        {"depth": np.nan},
    ],
)
def test_invalid_projection_inputs_raise_value_error(overrides):
    arguments = {
        "u": CX,
        "v": CY,
        "depth": 1.0,
        "fx": FX,
        "fy": FY,
        "cx": CX,
        "cy": CY,
    }
    arguments.update(overrides)

    with pytest.raises(ValueError):
        deproject_pixel(**arguments)
