"""Round-trip tests for the .d6a SQLite storage."""

import sqlite3

import pytest

from arm_motion.domain.errors import (
    InvalidMotionError,
    MotionAlreadyExistsError,
    MotionNotFoundError,
)
from arm_motion.domain.joint_spec import JointKind, JointSpec
from arm_motion.domain.motion import Motion, MotionStep
from arm_motion.domain.pose import Pose
from arm_motion.domain.robot_profile import build_profile
from arm_motion.domain.servo_scale import ServoScale
from arm_motion.infrastructure.d6a_repository import D6aMotionRepository


@pytest.fixture
def profile():
    joints = [
        JointSpec(f'joint{i}', lower=-2.09, upper=2.09) for i in range(1, 6)
    ] + [
        JointSpec(
            'r_joint', lower=-1.57, upper=1.57, kind=JointKind.GRIPPER,
            group='gripper', open_position=-1.0, closed_position=0.3,
        )
    ]
    scales = [
        ServoScale(i, f'joint{i}', 0, 1000, -2.0943951, 2.0943951)
        for i in range(1, 6)
    ] + [ServoScale(10, 'r_joint', 0, 1000, -1.57, 1.57)]
    return build_profile(joints, scales)


@pytest.fixture
def repository(tmp_path, profile):
    return D6aMotionRepository(tmp_path, profile)


def make_motion(profile, name='pick_init', pulses=(500, 300, 300, 215, 500, 500)):
    pose = profile.pose_from_pulses(
        dict(zip([j.name for j in profile.joints], pulses))
    )
    return Motion(
        name=name,
        steps=(
            MotionStep(pose, 1000),
            MotionStep(pose, 200),
            MotionStep(pose, 1500),
        ),
        description='test group',
    )


class TestRoundTrip:

    def test_save_then_load(self, repository, profile):
        motion = make_motion(profile)
        repository.save(motion)

        loaded = repository.load('pick_init')
        assert loaded.name == 'pick_init'
        assert loaded.description == 'test group'
        assert [s.duration_ms for s in loaded.steps] == [1000, 200, 1500]

    def test_revolute_positions_survive_the_round_trip(self, repository, profile):
        pulses = (15, 180, 215, 245, 650, 0)
        motion = make_motion(profile, pulses=pulses)
        repository.save(motion)

        loaded = repository.load(motion.name)
        reloaded = profile.pulses_from_pose(loaded.steps[0].pose)
        assert [reloaded[f'joint{i}'] for i in range(1, 6)] == list(pulses[:5])

    def test_gripper_pulses_snap_to_a_detent(self, repository, profile):
        """A half-open gripper value in a file must not survive as-is."""
        # Pulse 400 is ~0.256 rad — between open (-1.0) and closed (0.3).
        motion = make_motion(profile, pulses=(500, 500, 500, 500, 500, 400))
        repository.save(motion)

        loaded = repository.load(motion.name)
        position = loaded.steps[0].pose['r_joint']
        assert position == pytest.approx(0.3), 'should snap to the closed detent'
        # And the stored motion must pass strict validation.
        profile.validate_pose(loaded.steps[0].pose)

    def test_file_uses_the_hiwonder_table_layout(self, repository, profile):
        repository.save(make_motion(profile))
        path = repository.path_for('pick_init')

        conn = sqlite3.connect(str(path))
        try:
            columns = [
                row[1]
                for row in conn.execute('PRAGMA table_info(ActionGroup)').fetchall()
            ]
            rows = conn.execute(
                'SELECT [Index], Time, Servo1, Servo6 FROM ActionGroup ORDER BY [Index]'
            ).fetchall()
        finally:
            conn.close()

        assert columns[:2] == ['Index', 'Time']
        assert columns[2:] == [f'Servo{i}' for i in range(1, 7)]
        assert rows[0][0] == 1
        assert rows[0][1] == 1000

    def test_reads_a_file_without_motioninfo(self, repository, profile, tmp_path):
        """A file written by the original editor has no MotionInfo table."""
        path = tmp_path / 'legacy.d6a'
        conn = sqlite3.connect(str(path))
        try:
            with conn:
                conn.execute(
                    'CREATE TABLE ActionGroup ('
                    '[Index] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, Time INT, '
                    'Servo1 INT, Servo2 INT, Servo3 INT, Servo4 INT, '
                    'Servo5 INT, Servo6 INT)'
                )
                conn.execute(
                    'INSERT INTO ActionGroup VALUES (1, 1000, 500, 500, 300, 215, '
                    '500, 500)'
                )
        finally:
            conn.close()

        loaded = repository.load('legacy')
        assert loaded.name == 'legacy'
        assert len(loaded) == 1
        assert loaded.steps[0].duration_ms == 1000
        pulses = profile.pulses_from_pose(loaded.steps[0].pose)
        assert pulses['joint1'] == 500
        assert pulses['joint4'] == 215


class TestLibraryOperations:

    def test_overwrite_is_required_to_replace(self, repository, profile):
        repository.save(make_motion(profile))
        with pytest.raises(MotionAlreadyExistsError):
            repository.save(make_motion(profile))
        repository.save(make_motion(profile), overwrite=True)

    def test_load_missing_raises(self, repository):
        with pytest.raises(MotionNotFoundError):
            repository.load('nope')

    def test_delete(self, repository, profile):
        repository.save(make_motion(profile))
        assert repository.exists('pick_init')
        repository.delete('pick_init')
        assert not repository.exists('pick_init')
        with pytest.raises(MotionNotFoundError):
            repository.delete('pick_init')

    def test_list(self, repository, profile):
        repository.save(make_motion(profile, name='a'))
        repository.save(make_motion(profile, name='b'))
        assert sorted(m.name for m in repository.list()) == ['a', 'b']

    def test_list_skips_unreadable_files(self, repository, profile, tmp_path):
        repository.save(make_motion(profile, name='good'))
        (tmp_path / 'broken.d6a').write_bytes(b'not a database')
        assert [m.name for m in repository.list()] == ['good']

    def test_path_traversal_is_rejected(self, repository):
        for bad in ('../escape', 'sub/dir', '..'):
            with pytest.raises(InvalidMotionError):
                repository.path_for(bad)

    def test_save_rejects_out_of_range_pose(self, repository, profile):
        # Bypass the profile helpers to build a deliberately illegal pose.
        bad_pose = Pose({j.name: 0.0 for j in profile.joints} | {'joint1': 99.0})
        motion = Motion('bad', (MotionStep(bad_pose, 1000),))
        with pytest.raises(InvalidMotionError):
            repository.save(motion)

    def test_name_comes_from_the_filename_not_the_metadata(
        self, repository, profile, tmp_path
    ):
        """A hand-edited MotionInfo.name must not shadow the real filename."""
        repository.save(make_motion(profile, name='real_name'))
        conn = sqlite3.connect(str(tmp_path / 'real_name.d6a'))
        try:
            with conn:
                conn.execute(
                    "UPDATE MotionInfo SET Value = 'lie' WHERE Key = 'name'"
                )
        finally:
            conn.close()

        assert repository.load('real_name').name == 'real_name'
        assert [m.name for m in repository.list()] == ['real_name']

    def test_list_ignores_symlinks_pointing_outside(
        self, repository, profile, tmp_path
    ):
        outside = tmp_path.parent / 'outside.d6a'
        repository.save(make_motion(profile, name='inside'))
        (tmp_path / 'inside.d6a').rename(tmp_path / 'inside.d6a')
        # Build a real motion file outside the library, then link it in.
        D6aMotionRepository(outside.parent, profile).save(
            make_motion(profile, name='outside')
        )
        (tmp_path / 'linked.d6a').symlink_to(outside)

        assert [m.name for m in repository.list()] == ['inside']

    def test_failed_save_leaves_no_temp_files(self, repository, profile):
        bad_pose = Pose({j.name: 0.0 for j in profile.joints} | {'joint1': 99.0})
        with pytest.raises(InvalidMotionError):
            repository.save(Motion('bad', (MotionStep(bad_pose, 1000),)))
        assert list(repository.directory.glob('*.tmp')) == []
