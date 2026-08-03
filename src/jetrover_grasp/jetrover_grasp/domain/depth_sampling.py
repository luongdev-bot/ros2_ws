"""Pure robust depth-image sampling."""

import numpy as np


def sample_depth(depth_image, u, v, window=5) -> float | None:
    """Return a nearest-surface robust depth around a pixel, or ``None``.

    ``depth_image`` must be two-dimensional and contain depths in metres.
    Invalid depths are non-finite or non-positive values.
    """
    if (
        isinstance(window, (bool, np.bool_))
        or not isinstance(window, (int, np.integer))
        or window <= 0
        or window % 2 == 0
    ):
        raise ValueError("window must be a positive odd integer")

    image = np.asarray(depth_image)
    if image.ndim != 2:
        raise ValueError("depth_image must be a 2-D array")

    try:
        pixel_u = float(u)
        pixel_v = float(v)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(pixel_u) or not np.isfinite(pixel_v):
        return None

    height, width = image.shape
    if pixel_u < 0 or pixel_u >= width or pixel_v < 0 or pixel_v >= height:
        return None

    center_u = int(round(pixel_u))
    center_v = int(round(pixel_v))
    if center_u < 0 or center_u >= width or center_v < 0 or center_v >= height:
        return None

    radius = window // 2
    u_start = max(0, center_u - radius)
    u_stop = min(width, center_u + radius + 1)
    v_start = max(0, center_v - radius)
    v_stop = min(height, center_v + radius + 1)
    patch = image[v_start:v_stop, u_start:u_stop]

    valid_depths = patch[np.isfinite(patch) & (patch > 0)]
    if valid_depths.size == 0:
        return None
    return float(np.percentile(valid_depths, 20))
