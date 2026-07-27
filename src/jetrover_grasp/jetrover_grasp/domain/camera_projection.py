"""Pure pinhole-camera projection helpers."""

import numpy as np


def deproject_pixel(u, v, depth, fx, fy, cx, cy) -> np.ndarray:
    """Return a pixel's 3D position in the ROS camera optical frame.

    The returned axes follow the ROS optical convention: X right, Y down,
    and Z forward.
    """
    try:
        values = np.asarray((u, v, depth, fx, fy, cx, cy), dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("projection inputs must be finite numbers") from exc

    if values.shape != (7,):
        raise ValueError("projection inputs must be scalar numbers")
    if not np.all(np.isfinite(values)):
        raise ValueError("projection inputs must be finite")
    u, v, depth, fx, fy, cx, cy = values
    if depth <= 0:
        raise ValueError("depth must be positive")
    if fx == 0:
        raise ValueError("fx must be non-zero")
    if fx < 0:
        raise ValueError("fx must be positive")
    if fy == 0:
        raise ValueError("fy must be non-zero")
    if fy < 0:
        raise ValueError("fy must be positive")

    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return np.asarray((x, y, depth), dtype=float)


def intrinsics_from_camera_info_k(k) -> tuple[float, float, float, float]:
    """Extract ``(fx, fy, cx, cy)`` from a row-major CameraInfo K matrix."""
    try:
        matrix = np.asarray(k, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "camera intrinsic matrix K must contain 9 numbers"
        ) from exc

    if matrix.size != 9:
        raise ValueError("camera intrinsic matrix K must contain 9 elements")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("camera intrinsic matrix K must be finite")

    fx = float(matrix[0])
    fy = float(matrix[4])
    cx = float(matrix[2])
    cy = float(matrix[5])
    if fx <= 0:
        raise ValueError("K[0] (fx) must be positive")
    if fy <= 0:
        raise ValueError("K[4] (fy) must be positive")
    return fx, fy, cx, cy
