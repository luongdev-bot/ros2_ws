"""Tests for JetRover forward kinematics."""

import numpy as np
import pytest

from jetrover_kinematics import BASE_HEIGHT, forward_kinematics
from jetrover_kinematics.domain.arm_geometry import (
    END_EFFECTOR_TRANSLATION,
    JOINT1_TRANSLATION,
    JOINT2_TRANSLATION,
    JOINT3_TRANSLATION,
    JOINT4_TRANSLATION,
    JOINT5_TRANSLATION,
)


def _translation(offset):
    transform = np.eye(4)
    transform[:3, 3] = offset
    return transform


def _rotation_y(angle):
    cosine = np.cos(angle)
    sine = np.sin(angle)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ]
    return transform


def _rotation_z(angle):
    cosine = np.cos(angle)
    sine = np.sin(angle)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return transform


def _independent_end_effector_position(q):
    q1, q2, q3, q4, q5 = q
    transform = _translation([0.0, 0.0, BASE_HEIGHT])
    transform = transform @ _translation(JOINT1_TRANSLATION)
    transform = transform @ _rotation_z(-q1)
    transform = transform @ _translation(JOINT2_TRANSLATION)
    transform = transform @ _rotation_y(q2)
    transform = transform @ _translation(JOINT3_TRANSLATION)
    transform = transform @ _rotation_y(q3)
    transform = transform @ _translation(JOINT4_TRANSLATION)
    transform = transform @ _rotation_y(q4)
    transform = transform @ _translation(JOINT5_TRANSLATION)
    transform = transform @ _rotation_z(-q5)
    transform = transform @ _translation(END_EFFECTOR_TRANSLATION)
    return transform[:3, 3]


def test_zero_joints_match_verified_end_effector_position():
    """The home transform must match the URDF-derived chain."""
    pose = forward_kinematics(np.zeros(5))

    np.testing.assert_allclose(
        pose[:3, 3],
        np.array([0.0251328, 0.0, 0.620703]),
        atol=1e-7,
    )
    np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-12)


@pytest.mark.parametrize(
    'joints',
    [
        np.array([0.4, -0.7, 0.5, 1.1, -0.3]),
        np.array([-1.2, 0.9, -0.4, -0.8, 1.5]),
    ],
)
def test_nonzero_joints_match_independent_transform_chain(joints):
    """Non-zero FK must retain the URDF transform order and axis signs."""
    expected_position = _independent_end_effector_position(joints)

    np.testing.assert_allclose(
        forward_kinematics(joints)[:3, 3],
        expected_position,
        atol=1e-12,
    )
