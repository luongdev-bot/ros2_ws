"""Pure planar feedback control for a holonomic mobile base."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the odometry frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class BaseGains:
    """Proportional gains, limits, and goal tolerances."""

    kp_lin: float = 1.2
    kp_ang: float = 2.0
    max_lin: float = 0.25
    max_ang: float = 1.0
    pos_tol: float = 0.04
    yaw_tol: float = 0.08
    min_lin: float = 0.0


def _require_finite(**values: float) -> None:
    for name, value in values.items():
        try:
            finite = math.isfinite(value)
        except TypeError as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not finite:
            raise ValueError(f"{name} must be finite")


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(value, limit))


def wrap_angle(a: float) -> float:
    """Wrap an angle in radians to the interval [-pi, pi]."""
    _require_finite(angle=a)
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def body_velocity_command(
    current: Pose2D,
    goal: Pose2D,
    gains: BaseGains,
) -> tuple[float, float, float, bool]:
    """Return a saturated body-frame velocity command and goal status."""
    _require_finite(
        current_x=current.x,
        current_y=current.y,
        current_yaw=current.yaw,
        goal_x=goal.x,
        goal_y=goal.y,
        goal_yaw=goal.yaw,
        kp_lin=gains.kp_lin,
        kp_ang=gains.kp_ang,
        max_lin=gains.max_lin,
        max_ang=gains.max_ang,
        pos_tol=gains.pos_tol,
        yaw_tol=gains.yaw_tol,
        min_lin=gains.min_lin,
    )

    ex = goal.x - current.x
    ey = goal.y - current.y
    eyaw = wrap_angle(goal.yaw - current.yaw)
    reached = (
        math.hypot(ex, ey) < gains.pos_tol
        and abs(eyaw) < gains.yaw_tol
    )
    if reached:
        return 0.0, 0.0, 0.0, True

    cos_yaw = math.cos(current.yaw)
    sin_yaw = math.sin(current.yaw)
    bx = ex * cos_yaw + ey * sin_yaw
    by = -ex * sin_yaw + ey * cos_yaw

    vx = _clamp(gains.kp_lin * bx, gains.max_lin)
    vy = _clamp(gains.kp_lin * by, gains.max_lin)
    wz = _clamp(gains.kp_ang * eyaw, gains.max_ang)
    return vx, vy, wz, False


def yaw_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """Extract planar yaw from a quaternion in ROS ``(x, y, z, w)`` order."""
    _require_finite(x=x, y=y, z=z, w=w)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


__all__ = [
    "BaseGains",
    "Pose2D",
    "body_velocity_command",
    "wrap_angle",
    "yaw_from_quaternion",
]
