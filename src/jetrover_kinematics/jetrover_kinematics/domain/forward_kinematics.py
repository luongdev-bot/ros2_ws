"""Forward kinematics for the JetRover arm."""

import numpy as np

from .arm_geometry import (
    BASE_HEIGHT,
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
    return np.array([
        [cosine, 0.0, sine, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sine, 0.0, cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _rotation_z(angle):
    cosine = np.cos(angle)
    sine = np.sin(angle)
    return np.array([
        [cosine, -sine, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _joint_vector(q):
    joints = np.asarray(q, dtype=float)
    if joints.shape != (5,):
        raise ValueError('q must contain exactly five joint angles')
    if not np.all(np.isfinite(joints)):
        raise ValueError('q must contain only finite joint angles')
    return joints


def forward_kinematics(q):
    """Return the base_footprint-to-end_effector_link transform.

    Joint angles are in radians. The translation-before-rotation order and
    joint-axis signs match the JetRover_Mecanum simulation URDF exactly.
    """
    q1, q2, q3, q4, q5 = _joint_vector(q)

    transform = _translation(np.array([0.0, 0.0, BASE_HEIGHT]))
    transform = (
        transform
        @ _translation(JOINT1_TRANSLATION)
        @ _rotation_z(-q1)
    )
    transform = (
        transform
        @ _translation(JOINT2_TRANSLATION)
        @ _rotation_y(q2)
    )
    transform = (
        transform
        @ _translation(JOINT3_TRANSLATION)
        @ _rotation_y(q3)
    )
    transform = (
        transform
        @ _translation(JOINT4_TRANSLATION)
        @ _rotation_y(q4)
    )
    transform = (
        transform
        @ _translation(JOINT5_TRANSLATION)
        @ _rotation_z(-q5)
    )
    transform = transform @ _translation(END_EFFECTOR_TRANSLATION)
    return transform


__all__ = ['forward_kinematics']
