"""Tests for JetRover inverse kinematics."""

import numpy as np
import pytest

from jetrover_kinematics import (
    BASE_HEIGHT,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    forward_kinematics,
    inverse_kinematics,
)


def test_position_only_round_trip_for_random_reachable_targets():
    """IK must recover every deterministic reachable FK position."""
    np.testing.assert_array_equal(JOINT_LIMITS_LOWER, [-2.09] * 5)
    np.testing.assert_array_equal(JOINT_LIMITS_UPPER, [2.09] * 5)
    random_generator = np.random.default_rng(20260723)
    joint_samples = random_generator.uniform(
        -2.09,
        2.09,
        size=(50, 5),
    )

    failures = []
    for source_joints in joint_samples:
        target_position = forward_kinematics(source_joints)[:3, 3]
        result = inverse_kinematics(target_position)
        if result['position_error'] >= 1e-3:
            failures.append(result['position_error'])

        assert result['q'].shape == (5,)
        assert np.all(result['q'] >= JOINT_LIMITS_LOWER)
        assert np.all(result['q'] <= JOINT_LIMITS_UPPER)

    assert not failures, f'round-trip position errors: {failures}'


def test_full_orientation_controls_success_and_reports_angle():
    """Full-orientation success must require both geometric tolerances."""
    source_joints = np.array([0.4, 0.6, -0.3, 0.9, -0.7])
    source_pose = forward_kinematics(source_joints)

    success = inverse_kinematics(
        source_pose[:3, 3],
        q0=source_joints,
        target_rotation=source_pose[:3, :3],
    )
    assert success['success']
    assert success['position_error'] < 2e-3
    assert success['orientation_error'] < 0.05

    quarter_turn = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    failure = inverse_kinematics(
        source_pose[:3, 3],
        q0=source_joints,
        target_rotation=source_pose[:3, :3] @ quarter_turn,
        orientation_weight=0.0,
    )
    assert failure['position_error'] < 2e-3
    assert failure['orientation_error'] > 0.05
    assert not failure['success']


def test_z_axis_constraint_reports_axis_angle():
    """The back-compatible z-axis constraint must contribute to success."""
    source_joints = np.array([-0.5, 0.8, 0.2, 0.7, 0.4])
    source_pose = forward_kinematics(source_joints)

    result = inverse_kinematics(
        source_pose[:3, 3],
        q0=source_joints,
        target_z_axis=source_pose[:3, 2],
    )

    assert result['success']
    assert result['orientation_error'] < 0.05


def test_unconstrained_orientation_error_is_none():
    """Position-only IK must explicitly report no orientation constraint."""
    result = inverse_kinematics([0.0251328, 0.0, 0.620703])

    assert result['orientation_error'] is None


@pytest.mark.parametrize(
    'keyword, value',
    [
        ('target_z_axis', np.array([1e308, 1e308, 1e308])),
        ('target_rotation', np.full((3, 3), 1e308)),
    ],
)
def test_non_finite_normalization_norms_raise(keyword, value):
    """Finite components whose normalization overflows must be rejected."""
    with pytest.raises(ValueError, match='finite, non-zero norm'):
        inverse_kinematics(
            [0.1, 0.0, BASE_HEIGHT],
            **{keyword: value},
        )


def test_non_finite_target_position_raises():
    """Position validation must reject non-finite components."""
    with pytest.raises(ValueError, match='only finite values'):
        inverse_kinematics([np.inf, 0.0, BASE_HEIGHT])


@pytest.mark.parametrize(
    'rest_posture, message',
    [
        ([0.0] * 4, 'exactly five'),
        ([0.0, 0.0, np.nan, 0.0, 0.0], 'only finite'),
    ],
)
def test_invalid_rest_posture_raises(rest_posture, message):
    """Posture regularization must reject malformed reference joints."""
    with pytest.raises(ValueError, match=message):
        inverse_kinematics(
            [0.1, 0.0, BASE_HEIGHT],
            rest_posture=rest_posture,
        )


def test_public_exports_are_importable():
    """Every declared top-level API export must resolve."""
    import jetrover_kinematics

    for name in jetrover_kinematics.__all__:
        assert getattr(jetrover_kinematics, name) is not None
