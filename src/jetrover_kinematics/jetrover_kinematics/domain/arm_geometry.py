"""URDF-derived geometry for the JetRover 5-DOF arm."""

import numpy as np


# Translations are in metres and are applied before the associated rotation.
# JetRover_Mecanum height; Tank is 0.127 m and Ackermann is about 0.115487 m.
BASE_HEIGHT = 0.11609108
JOINT1_TRANSLATION = np.array([0.0251328, 0.0, 0.07740269])
JOINT2_TRANSLATION = np.array([0.0, 0.0, 0.03386480])
JOINT3_TRANSLATION = np.array([0.0, 0.0, 0.12941645])
JOINT4_TRANSLATION = np.array([0.0, 0.0, 0.12944463])
JOINT5_TRANSLATION = np.array([0.0, 0.0, 0.05448333])
END_EFFECTOR_TRANSLATION = np.array([0.0, 0.0, 0.08])

JOINT_LIMITS_LOWER = np.array([-2.09] * 5)
JOINT_LIMITS_UPPER = np.array([2.09] * 5)

__all__ = [
    'BASE_HEIGHT',
    'END_EFFECTOR_TRANSLATION',
    'JOINT1_TRANSLATION',
    'JOINT2_TRANSLATION',
    'JOINT3_TRANSLATION',
    'JOINT4_TRANSLATION',
    'JOINT5_TRANSLATION',
    'JOINT_LIMITS_LOWER',
    'JOINT_LIMITS_UPPER',
]
