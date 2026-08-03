"""Pure frame transforms for mobile-base approach planning."""

import math

from .base_control import Pose2D


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        try:
            finite = math.isfinite(value)
        except TypeError as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not finite:
            raise ValueError(f"{name} must be finite")


def base_to_odom(
    point_base_xy,
    odom_pose: Pose2D,
) -> tuple[float, float]:
    """Transform a point from ``base_footprint`` into the odom frame."""
    bx, by = point_base_xy
    _require_finite(
        base_x=bx,
        base_y=by,
        odom_x=odom_pose.x,
        odom_y=odom_pose.y,
        odom_yaw=odom_pose.yaw,
    )

    cos_yaw = math.cos(odom_pose.yaw)
    sin_yaw = math.sin(odom_pose.yaw)
    return (
        odom_pose.x + bx * cos_yaw - by * sin_yaw,
        odom_pose.y + bx * sin_yaw + by * cos_yaw,
    )


def base_goal_to_center_target(
    block_base_xy,
    odom_pose: Pose2D,
    sweet_spot_base_xy,
    goal_yaw: float = 0.0,
) -> Pose2D:
    """Return the odom base goal that places a target at the sweet spot."""
    sx, sy = sweet_spot_base_xy
    _require_finite(
        sweet_spot_x=sx,
        sweet_spot_y=sy,
        goal_yaw=goal_yaw,
    )

    target_x, target_y = base_to_odom(block_base_xy, odom_pose)
    cos_yaw = math.cos(goal_yaw)
    sin_yaw = math.sin(goal_yaw)
    goal_x = target_x - (sx * cos_yaw - sy * sin_yaw)
    goal_y = target_y - (sx * sin_yaw + sy * cos_yaw)
    return Pose2D(goal_x, goal_y, goal_yaw)


__all__ = [
    "base_goal_to_center_target",
    "base_to_odom",
]
