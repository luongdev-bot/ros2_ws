"""Top-down grasp-pose generation and inverse-kinematics selection."""

import numpy as np

from .inverse_kinematics import inverse_kinematics


def _position_vector(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f'{name} must contain exactly three values')
    if not np.all(np.isfinite(vector)):
        raise ValueError(f'{name} must contain only finite values')
    return vector


def top_down_grasp_poses(block_xyz, z_offset=0.0, n_rotations=4):
    """Return evenly yawed, vertically downward grasp target transforms.

    Every pose has its tool z-axis fixed at ``(0, 0, -1)``. Rotating the
    horizontal tool axes in equal steps supplies equivalent wrist-yaw choices.
    """
    block_position = _position_vector(block_xyz, 'block_xyz')
    offset = float(z_offset)
    if not np.isfinite(offset):
        raise ValueError('z_offset must be finite')
    if not isinstance(n_rotations, int) or isinstance(n_rotations, bool):
        raise ValueError('n_rotations must be a positive integer')
    if n_rotations <= 0:
        raise ValueError('n_rotations must be a positive integer')

    target_position = block_position + np.array([0.0, 0.0, offset])
    poses = []
    for index in range(n_rotations):
        yaw = 2.0 * np.pi * index / n_rotations
        cosine = np.cos(yaw)
        sine = np.sin(yaw)

        pose = np.eye(4)
        pose[:3, 0] = [cosine, sine, 0.0]
        pose[:3, 1] = [sine, -cosine, 0.0]
        pose[:3, 2] = [0.0, 0.0, -1.0]
        pose[:3, 3] = target_position
        poses.append(pose)
    return poses


def best_ik_for_poses(
        target_poses,
        q0=None,
        rest_posture=None,
        position_error_threshold=0.02,
        orientation_tol=0.05):
    """Return the best tolerance-qualified IK result, or ``None``.

    Acceptance uses the raw position and orientation errors, independently of
    the inverse solver's internal position-success tolerance. A supplied rest
    posture ranks qualified poses by joint-space distance; otherwise the raw
    position error is used.
    """
    threshold = float(position_error_threshold)
    if not np.isfinite(threshold) or threshold <= 0.0:
        raise ValueError(
            'position_error_threshold must be finite and positive'
        )
    orientation_tolerance = float(orientation_tol)
    if (
            not np.isfinite(orientation_tolerance)
            or orientation_tolerance <= 0.0):
        raise ValueError('orientation_tol must be finite and positive')

    best_result = None
    best_result_key = None
    for pose_index, target_pose in enumerate(target_poses):
        pose = np.asarray(target_pose, dtype=float)
        if pose.shape != (4, 4):
            raise ValueError('each target pose must be a 4x4 matrix')
        if not np.all(np.isfinite(pose)):
            raise ValueError('target poses must contain only finite values')

        result = inverse_kinematics(
            target_position=pose[:3, 3],
            q0=q0,
            target_rotation=pose[:3, :3],
            orientation_tol=orientation_tolerance,
            rest_posture=rest_posture,
        )
        if result['position_error'] >= threshold:
            continue
        if result['orientation_error'] >= orientation_tolerance:
            continue

        result = dict(result)
        result['pose_index'] = pose_index
        if rest_posture is None:
            result_key = (result['position_error'],)
        else:
            posture_distance = float(np.linalg.norm(
                result['q'] - np.asarray(rest_posture, dtype=float)
            ))
            result_key = (posture_distance, result['position_error'])
        if best_result_key is None or result_key < best_result_key:
            best_result = result
            best_result_key = result_key
    return best_result


__all__ = ['best_ik_for_poses', 'top_down_grasp_poses']
