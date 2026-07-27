"""Tests for pure robust depth sampling."""

import numpy as np
import pytest

from jetrover_grasp.domain.depth_sampling import sample_depth


def test_patch_median_ignores_non_positive_and_non_finite_values():
    depth_image = np.full((5, 5), 50.0, dtype=np.float32)
    depth_image[1:4, 1:4] = [
        [0.0, np.nan, 1.0],
        [np.inf, 2.0, 4.0],
        [-np.inf, 6.0, 100.0],
    ]

    depth = sample_depth(depth_image, 2, 2, window=3)

    assert depth == pytest.approx(4.0)


def test_all_invalid_depths_return_none():
    depth_image = np.array(
        [
            [0.0, np.nan, np.inf],
            [-np.inf, -1.0, 0.0],
            [np.nan, 0.0, -2.0],
        ],
        dtype=np.float32,
    )

    assert sample_depth(depth_image, 1, 1, window=3) is None


@pytest.mark.parametrize(
    ("u", "v"),
    [
        (-1, 2),
        (5, 2),
        (2, -1),
        (2, 5),
    ],
)
def test_out_of_bounds_pixel_returns_none(u, v):
    depth_image = np.ones((5, 5), dtype=np.float32)

    assert sample_depth(depth_image, u, v) is None


@pytest.mark.parametrize("window", [4, 0, -3])
def test_invalid_window_is_rejected(window):
    depth_image = np.ones((5, 5), dtype=np.float32)

    with pytest.raises(ValueError):
        sample_depth(depth_image, 2, 2, window=window)
