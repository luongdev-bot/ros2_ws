"""Pure pick-and-place planning for the JetRover arm."""

from dataclasses import dataclass

import numpy as np
from jetrover_kinematics import (
    best_ik_for_poses,
    inverse_kinematics,
    top_down_grasp_poses,
)


@dataclass(frozen=True)
class GraspWaypoint:
    """One arm and gripper target in an ordered grasp plan."""

    label: str
    joint_positions: tuple | None
    gripper_position: float | None
    settle_time_s: float


@dataclass(frozen=True)
class GraspConfig:
    """Tunable heights, gripper targets, and timing for a grasp plan."""

    # A 0.08 m approach and lift keep the red corner block reachable.
    approach_height: float = 0.08
    grasp_z_offset: float = 0.0
    lift_height: float = 0.08
    place_height: float = 0.12
    gripper_open: float = -1.00
    gripper_closed: float = 0.30
    # Home must keep the arm-mounted camera aimed at the block row.
    q_home: tuple = (1.5708, -0.8378, 2.0316, 1.1938, 0.0)
    position_error_threshold: float = 0.02
    move_settle_s: float = 3.0
    grasp_settle_s: float = 2.0
    n_yaw: int = 4


def _validate_xyz(value, name):
    """Return a finite three-element position vector."""
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{name} must be a finite sequence of length 3"
        ) from exc
    if vector.shape != (3,):
        raise ValueError(f"{name} must be a finite sequence of length 3")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite sequence of length 3")
    return vector


def _joint_tuple(joints):
    """Convert an IK result's joints to an immutable tuple of Python floats."""
    try:
        vector = np.asarray(joints, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("IK solution must contain five joint angles") from exc
    if vector.shape != (5,) or not np.all(np.isfinite(vector)):
        raise ValueError("IK solution must contain five finite joint angles")
    return tuple(float(angle) for angle in vector)


def _home_tuple(config):
    """Convert and validate the configured home position."""
    return _joint_tuple(config.q_home)


def _solve(target_xyz, z_offset, previous_joints, config):
    """Solve one top-down target, seeded from the previous arm solution."""
    poses = top_down_grasp_poses(
        target_xyz,
        z_offset=z_offset,
        n_rotations=config.n_yaw,
    )
    result = best_ik_for_poses(
        poses,
        q0=previous_joints,
        rest_posture=config.q_home,
        position_error_threshold=config.position_error_threshold,
    )
    if result is None:
        return None
    return _joint_tuple(result["q"])


def _solve_place(target_xyz, previous_joints, config):
    """Solve the bin target with position-only inverse kinematics."""
    result = inverse_kinematics(
        target_xyz,
        q0=previous_joints,
        rest_posture=config.q_home,
    )
    if result is None:
        return None
    position_error = float(result["position_error"])
    if (
        not result["success"]
        or not np.isfinite(position_error)
        or position_error >= float(config.position_error_threshold)
    ):
        return None
    return _joint_tuple(result["q"])


def plan_pick_and_place(
    block_xyz,
    bin_xyz,
    config=GraspConfig(),
) -> list[GraspWaypoint] | None:
    """Build an eight-waypoint top-down pick-and-place plan.

    ``block_xyz`` and ``bin_xyz`` are expected in the arm base frame.  A
    required IK failure makes the complete plan unreachable and returns
    ``None``.
    """
    # Validate both inputs up front to retain the combined planner's
    # historical error behavior even when the pick target is unreachable.
    block_position = _validate_xyz(block_xyz, "block_xyz")
    bin_position = _validate_xyz(bin_xyz, "bin_xyz")

    pick = plan_pick(block_position, config)
    if pick is None:
        return None

    # Passing the lift solution into the place planner preserves the
    # continuous IK seed across the pick/place seam.
    place = plan_place(
        bin_position,
        config,
        q_seed=pick[-1].joint_positions,
    )
    if place is None:
        return None
    return pick + place


def plan_pick(
    block_xyz,
    config=GraspConfig(),
) -> list[GraspWaypoint] | None:
    """Build the five-waypoint top-down pick portion of a plan.

    The approach, descend, and lift solves are seeded continuously from the
    preceding solved arm pose.  A failed required pick solve returns ``None``.
    """
    block_position = _validate_xyz(block_xyz, "block_xyz")
    home_joints = _home_tuple(config)
    move_settle = float(config.move_settle_s)
    grasp_settle = float(config.grasp_settle_s)
    gripper_open = float(config.gripper_open)
    gripper_closed = float(config.gripper_closed)

    waypoints = [
        GraspWaypoint(
            label="home",
            joint_positions=home_joints,
            gripper_position=gripper_open,
            settle_time_s=move_settle,
        )
    ]

    solved_joints = _solve(
        block_position,
        config.approach_height,
        home_joints,
        config,
    )
    if solved_joints is None:
        return None
    waypoints.append(
        GraspWaypoint(
            label="approach",
            joint_positions=solved_joints,
            gripper_position=gripper_open,
            settle_time_s=move_settle,
        )
    )

    solved_joints = _solve(
        block_position,
        config.grasp_z_offset,
        solved_joints,
        config,
    )
    if solved_joints is None:
        return None
    waypoints.append(
        GraspWaypoint(
            label="descend",
            joint_positions=solved_joints,
            gripper_position=gripper_open,
            settle_time_s=move_settle,
        )
    )

    waypoints.append(
        GraspWaypoint(
            label="close",
            joint_positions=None,
            gripper_position=gripper_closed,
            settle_time_s=grasp_settle,
        )
    )

    solved_joints = _solve(
        block_position,
        config.grasp_z_offset + config.lift_height,
        solved_joints,
        config,
    )
    if solved_joints is None:
        return None
    waypoints.append(
        GraspWaypoint(
            label="lift",
            joint_positions=solved_joints,
            gripper_position=gripper_closed,
            settle_time_s=move_settle,
        )
    )
    return waypoints


def plan_place(
    bin_xyz,
    config=GraspConfig(),
    q_seed=None,
) -> list[GraspWaypoint] | None:
    """Build the three-waypoint place portion of a plan.

    ``q_seed`` is the arm solution from the preceding pick when composing a
    complete plan.  When omitted, the configured home pose is used so this
    function remains independently usable.
    """
    bin_position = _validate_xyz(bin_xyz, "bin_xyz")
    home_joints = _home_tuple(config)
    previous_joints = (
        home_joints if q_seed is None else _joint_tuple(q_seed)
    )
    move_settle = float(config.move_settle_s)
    grasp_settle = float(config.grasp_settle_s)
    gripper_open = float(config.gripper_open)
    gripper_closed = float(config.gripper_closed)

    place_position = bin_position + np.array(
        [0.0, 0.0, config.place_height],
        dtype=float,
    )
    solved_joints = _solve_place(place_position, previous_joints, config)
    if solved_joints is None:
        return None

    return [
        GraspWaypoint(
            label="to_bin",
            joint_positions=solved_joints,
            gripper_position=gripper_closed,
            settle_time_s=move_settle,
        ),
        GraspWaypoint(
            label="release",
            joint_positions=None,
            gripper_position=gripper_open,
            settle_time_s=grasp_settle,
        ),
        GraspWaypoint(
            label="home",
            joint_positions=home_joints,
            gripper_position=gripper_open,
            settle_time_s=move_settle,
        ),
    ]


def plan_is_reachable(
    block_xyz,
    bin_xyz,
    config=GraspConfig(),
) -> bool:
    """Return whether a complete pick-and-place plan can be solved."""
    return plan_pick_and_place(block_xyz, bin_xyz, config) is not None


__all__ = [
    "GraspConfig",
    "GraspWaypoint",
    "plan_is_reachable",
    "plan_pick",
    "plan_pick_and_place",
    "plan_place",
]
