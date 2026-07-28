"""Tests for top-down JetRover grasp-pose generation."""

import numpy as np
import pytest

from jetrover_kinematics import (
    best_ik_for_poses,
    forward_kinematics,
    inverse_kinematics,
    top_down_grasp_poses,
)


_IK_INITIAL_JOINTS = np.array([0.0, -0.5, 1.0, 0.5, 0.0])
_DOWNWARD_AXIS = np.array([0.0, 0.0, -1.0])


def test_top_down_poses_are_right_handed_and_evenly_yawed():
    """Generated poses must preserve position and point the tool downward."""
    block_position = np.array([0.025, -0.245, 0.085])
    poses = top_down_grasp_poses(
        block_position,
        z_offset=0.01,
        n_rotations=4,
    )

    assert len(poses) == 4
    for pose in poses:
        np.testing.assert_allclose(
            pose[:3, 3],
            block_position + np.array([0.0, 0.0, 0.01]),
            atol=1e-12,
        )
        np.testing.assert_allclose(pose[:3, 2], _DOWNWARD_AXIS, atol=1e-12)
        np.testing.assert_allclose(
            pose[:3, :3].T @ pose[:3, :3],
            np.eye(3),
            atol=1e-12,
        )
        np.testing.assert_allclose(
            np.linalg.det(pose[:3, :3]),
            1.0,
            atol=1e-12,
        )

    expected_x_axes = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    np.testing.assert_allclose(
        np.array([pose[:3, 0] for pose in poses]),
        expected_x_axes,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    'block_position',
    [
        np.array([0.025, -0.245, 0.085]),
        np.array([-0.055, -0.245, 0.085]),
        np.array([0.105, -0.245, 0.085]),
    ],
    ids=['blue', 'green', 'yellow'],
)
def test_reachable_blocks_have_strict_top_down_solutions(block_position):
    """The blue, green, and yellow simulation blocks are top-down reachable."""
    result = best_ik_for_poses(
        top_down_grasp_poses(block_position),
        q0=_IK_INITIAL_JOINTS,
    )

    assert result is not None
    assert result['position_error'] < 3e-3
    assert result['orientation_error'] < 0.05
    assert result['pose_index'] in range(4)
    reached_pose = forward_kinematics(result['q'])
    assert np.linalg.norm(reached_pose[:3, 2] - _DOWNWARD_AXIS) < 0.05


def test_blue_block_regularization_prefers_a_natural_top_down_posture():
    """Regularization must retain accuracy while preferring natural joints."""
    poses = top_down_grasp_poses([0.025, -0.245, 0.085])
    rest_posture = np.array([1.57, 1.25, 0.70, 1.20, 0.0])

    unregularized = best_ik_for_poses(
        poses,
        q0=_IK_INITIAL_JOINTS,
    )
    regularized = best_ik_for_poses(
        poses,
        q0=_IK_INITIAL_JOINTS,
        rest_posture=rest_posture,
    )

    assert unregularized is not None
    assert regularized is not None
    assert regularized['position_error'] < 3e-3
    assert (
        np.linalg.norm(regularized['q'] - rest_posture)
        < np.linalg.norm(unregularized['q'] - rest_posture)
    )


def test_red_block_documents_strict_vertical_workspace_limit():
    """The red simulation block is outside the strict top-down workspace."""
    red_block_position = np.array([-0.135, -0.245, 0.085])
    result = best_ik_for_poses(
        top_down_grasp_poses(red_block_position),
        q0=_IK_INITIAL_JOINTS,
        position_error_threshold=3e-3,
    )

    # With all five URDF limits corrected to +/-2.09 rad, the strict
    # top-down target remains outside the 3 mm workspace.
    assert result is None


def test_four_top_down_yaws_drive_distinct_wrist_angles():
    """Full pose IK must preserve the four meaningful wrist-yaw choices."""
    poses = top_down_grasp_poses([0.025, -0.245, 0.085])
    results = [
        inverse_kinematics(
            pose[:3, 3],
            q0=_IK_INITIAL_JOINTS,
            target_rotation=pose[:3, :3],
        )
        for pose in poses
    ]

    assert all(result['success'] for result in results)
    rounded_q5 = {round(float(result['q'][4]), 12) for result in results}
    assert len(rounded_q5) == 4


def test_loose_position_threshold_does_not_use_internal_success_flag():
    """A caller threshold above 2 mm must gate using the raw errors."""
    poses = top_down_grasp_poses([-0.135, -0.245, 0.085])
    result = best_ik_for_poses(
        poses,
        q0=_IK_INITIAL_JOINTS,
        position_error_threshold=0.04,
    )

    assert result is not None
    assert 2e-3 < result['position_error'] < 0.04
    assert result['orientation_error'] < 0.05
    assert not result['success']
