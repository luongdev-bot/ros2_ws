"""Numerical inverse kinematics for the JetRover arm."""

import numpy as np
from scipy.optimize import least_squares

from .arm_geometry import (
    JOINT1_TRANSLATION,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
)
from .forward_kinematics import forward_kinematics


_DEFAULT_Q0 = np.array([0.0, -0.5, 1.0, 0.5, 0.0])
_FALLBACK_POSITION_ERROR = 1e-3


def _vector(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f'{name} must contain exactly three values')
    if not np.all(np.isfinite(vector)):
        raise ValueError(f'{name} must contain only finite values')
    return vector


def _positive_finite(value, name):
    number = float(value)
    if not np.isfinite(number) or number <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return number


def _normalized_axis(value, name):
    axis = _vector(value, name)
    with np.errstate(over='ignore', invalid='ignore'):
        axis_norm = np.linalg.norm(axis)
    if not np.isfinite(axis_norm) or axis_norm == 0.0:
        raise ValueError(f'{name} must have a finite, non-zero norm')
    return axis / axis_norm


def _normalized_rotation(value):
    rotation = np.asarray(value, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('target_rotation must be a 3x3 matrix')
    if not np.all(np.isfinite(rotation)):
        raise ValueError('target_rotation must contain only finite values')

    with np.errstate(over='ignore', invalid='ignore'):
        column_norms = np.linalg.norm(rotation, axis=0)
    if np.any(~np.isfinite(column_norms)) or np.any(column_norms == 0.0):
        raise ValueError(
            'target_rotation columns must have finite, non-zero norms'
        )

    normalized = rotation / column_norms
    if not np.allclose(normalized.T @ normalized, np.eye(3), atol=1e-6):
        raise ValueError('target_rotation must have orthogonal columns')
    if not np.isclose(np.linalg.det(normalized), 1.0, atol=1e-6):
        raise ValueError('target_rotation must be right-handed')
    return normalized


def _initial_joints(q0):
    if q0 is None:
        return _DEFAULT_Q0.copy()

    joints = np.asarray(q0, dtype=float)
    if joints.shape != (5,):
        raise ValueError('q0 must contain exactly five joint angles')
    if not np.all(np.isfinite(joints)):
        raise ValueError('q0 must contain only finite joint angles')
    if np.any(joints < JOINT_LIMITS_LOWER):
        raise ValueError('q0 must be within the lower joint limits')
    if np.any(joints > JOINT_LIMITS_UPPER):
        raise ValueError('q0 must be within the upper joint limits')
    return joints


def _rest_joints(rest_posture):
    if rest_posture is None:
        return None

    joints = np.asarray(rest_posture, dtype=float)
    if joints.shape != (5,):
        raise ValueError(
            'rest_posture must contain exactly five joint angles'
        )
    if not np.all(np.isfinite(joints)):
        raise ValueError(
            'rest_posture must contain only finite joint angles'
        )
    return joints


def _fallback_initial_joints(target_position, q5):
    """Yield radial-branch seeds with straight and bent elbow postures."""
    target_offset = target_position[:2] - JOINT1_TRANSLATION[:2]
    primary_q1 = -np.arctan2(target_offset[1], target_offset[0])
    opposite_q1 = primary_q1 + np.pi

    arm_profiles = (
        np.array([0.0, 0.0, 0.0]),
        np.array([0.6, 0.6, 0.6]),
        np.array([-0.6, -0.6, -0.6]),
    )
    for q1 in (primary_q1, opposite_q1):
        wrapped_q1 = (q1 + np.pi) % (2.0 * np.pi) - np.pi
        for arm_profile in arm_profiles:
            seed = np.concatenate(([wrapped_q1], arm_profile, [q5]))
            yield np.clip(seed, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)


def _z_axis_error(tool_rotation, target_axis):
    cosine = np.clip(
        np.dot(tool_rotation[:, 2], target_axis),
        -1.0,
        1.0,
    )
    return float(np.arccos(cosine))


def _rotation_error(tool_rotation, target_rotation):
    relative_rotation = target_rotation.T @ tool_rotation
    cosine = np.clip(
        (np.trace(relative_rotation) - 1.0) / 2.0,
        -1.0,
        1.0,
    )
    return float(np.arccos(cosine))


def inverse_kinematics(
        target_position,
        q0=None,
        target_z_axis=None,
        target_rotation=None,
        orientation_weight=0.3,
        position_tol=2e-3,
        orientation_tol=0.05,
        rest_posture=None,
        regularization_weight=0.02):
    """Solve for joint angles that reach a target position and orientation.

    ``target_rotation`` constrains the tool x- and z-axes and takes precedence
    over ``target_z_axis``. Supplying only ``target_z_axis`` retains the
    position-plus-tool-axis behavior. ``rest_posture`` adds a soft joint-space
    preference without changing the FK-based geometric success criteria.
    """
    target = _vector(target_position, 'target_position')
    initial_joints = _initial_joints(q0)
    rest_joints = _rest_joints(rest_posture)

    target_frame = None
    target_axis = None
    if target_rotation is not None:
        target_frame = _normalized_rotation(target_rotation)
    elif target_z_axis is not None:
        target_axis = _normalized_axis(target_z_axis, 'target_z_axis')

    weight = float(orientation_weight)
    if not np.isfinite(weight) or weight < 0.0:
        raise ValueError('orientation_weight must be finite and non-negative')
    posture_weight = float(regularization_weight)
    if not np.isfinite(posture_weight) or posture_weight < 0.0:
        raise ValueError(
            'regularization_weight must be finite and non-negative'
        )
    position_tolerance = _positive_finite(position_tol, 'position_tol')
    orientation_tolerance = _positive_finite(
        orientation_tol,
        'orientation_tol',
    )
    orientation_constrained = (
        target_frame is not None or target_axis is not None
    )

    def geometric_residual(joints):
        pose = forward_kinematics(joints)
        position_residual = pose[:3, 3] - target
        residuals = [position_residual]
        if target_frame is not None:
            orientation_residual = weight * (
                pose[:3, (0, 2)] - target_frame[:, (0, 2)]
            ).ravel()
            residuals.append(orientation_residual)
        elif target_axis is not None:
            orientation_residual = weight * (pose[:3, 2] - target_axis)
            residuals.append(orientation_residual)
        return np.concatenate(residuals)

    def residual(joints):
        residuals = [geometric_residual(joints)]
        if rest_joints is not None:
            residuals.append(posture_weight * (joints - rest_joints))
        return np.concatenate(residuals)

    def solve(seed, objective=residual):
        return least_squares(
            objective,
            seed,
            bounds=(JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER),
            xtol=1e-12,
            ftol=1e-12,
        )

    def metrics(candidate):
        pose = forward_kinematics(candidate.x)
        position_error = float(np.linalg.norm(pose[:3, 3] - target))
        if target_frame is not None:
            orientation_error = _rotation_error(
                pose[:3, :3],
                target_frame,
            )
        elif target_axis is not None:
            orientation_error = _z_axis_error(pose[:3, :3], target_axis)
        else:
            orientation_error = None

        orientation_ok = (
            not orientation_constrained
            or orientation_error < orientation_tolerance
        )
        success = position_error < position_tolerance and orientation_ok
        return position_error, orientation_error, success

    def candidate_key(candidate):
        position_error, orientation_error, success = metrics(candidate)
        if success:
            return (0, position_error, orientation_error or 0.0)

        position_ratio = position_error / position_tolerance
        orientation_ratio = 0.0
        if orientation_constrained:
            orientation_ratio = orientation_error / orientation_tolerance
        return (
            1,
            max(position_ratio, orientation_ratio),
            position_error,
            orientation_error or 0.0,
        )

    initial_result = solve(initial_joints)
    candidates = [initial_result]
    initial_metrics = metrics(initial_result)
    position_needs_fallback = (
        initial_metrics[0] >= _FALLBACK_POSITION_ERROR
    )
    orientation_needs_fallback = (
        orientation_constrained
        and initial_metrics[1] >= orientation_tolerance
    )
    if position_needs_fallback or orientation_needs_fallback:
        candidates.extend(
            solve(seed)
            for seed in _fallback_initial_joints(
                target,
                initial_joints[4],
            )
        )

    result = min(candidates, key=candidate_key)
    position_error, orientation_error, success = metrics(result)
    if rest_joints is not None and not success:
        # The posture penalty selects a natural local branch. Polish only a
        # tolerance-missing result on that branch so geometric accuracy remains
        # authoritative rather than accepting the penalty's small compromise.
        polished_result = solve(result.x, objective=geometric_residual)
        if candidate_key(polished_result) < candidate_key(result):
            result = polished_result
            position_error, orientation_error, success = metrics(result)

    return {
        'q': result.x,
        'position_error': position_error,
        'orientation_error': orientation_error,
        'success': success,
    }


__all__ = ['inverse_kinematics']
