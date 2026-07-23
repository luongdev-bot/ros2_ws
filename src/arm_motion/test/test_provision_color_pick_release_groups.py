"""Tests for the operator script that derives colour-pick release groups."""

import importlib.util
import sqlite3
import sys
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SCRIPT_PATH = WORKSPACE / 'scripts' / 'provision_color_pick_release_groups.py'
ORIGINAL_NAMES = (
    'place_left',
    'place_center',
    'place_center1',
    'place_right',
)


@pytest.fixture(scope='module')
def provisioner():
    spec = importlib.util.spec_from_file_location(
        'provision_color_pick_release_groups', SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_original(
    path,
    servo1,
    gripper_pulses=(540, 540, 540, 400),
):
    rows = (
        (1, 1000, servo1, 650, 15, 280, 500, gripper_pulses[0]),
        (2, 1500, servo1, 210, 353, 217, 500, gripper_pulses[1]),
        (3, 500, servo1, 210, 353, 217, 500, gripper_pulses[2]),
        (4, 1500, servo1, 650, 15, 215, 500, gripper_pulses[3]),
    )
    with sqlite3.connect(str(path)) as connection:
        connection.execute(
            'CREATE TABLE ActionGroup ('
            '[Index] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, '
            'Time INT, Servo1 INT, Servo2 INT, Servo3 INT, Servo4 INT, '
            'Servo5 INT, Servo6 INT)'
        )
        connection.executemany(
            'INSERT INTO ActionGroup '
            '([Index], Time, Servo1, Servo2, Servo3, Servo4, Servo5, Servo6) '
            'VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
            rows,
        )
        connection.execute(
            'CREATE TABLE MotionInfo ('
            'Key TEXT PRIMARY KEY NOT NULL, Value TEXT)'
        )
        connection.executemany(
            'INSERT INTO MotionInfo (Key, Value) VALUES (?, ?)',
            (('name', path.stem), ('description', 'operator-taught original')),
        )


def _make_originals(
    library,
    *,
    gripper_pulses=(540, 540, 540, 400),
):
    pulses = (600, 500, 400, 400)
    for name, servo1 in zip(ORIGINAL_NAMES, pulses):
        _write_original(
            library / f'{name}.d6a',
            servo1,
            gripper_pulses=gripper_pulses,
        )


def _rows(path):
    with sqlite3.connect(str(path)) as connection:
        return connection.execute(
            'SELECT [Index], Time, Servo1, Servo2, Servo3, Servo4, '
            'Servo5, Servo6 FROM ActionGroup ORDER BY [Index]'
        ).fetchall()


def _metadata(path):
    with sqlite3.connect(str(path)) as connection:
        return connection.execute(
            'SELECT Key, Value FROM MotionInfo ORDER BY Key'
        ).fetchall()


def _profile(provisioner):
    return provisioner.build_robot_profile(
        provisioner.load_yaml(provisioner.ROBOT_CONFIG)
    )


def _open_pulse(provisioner):
    profile = _profile(provisioner)
    gripper = profile.gripper_joints()[0]
    return profile.scale(gripper.name).to_pulse(gripper.open_position)


def _assert_final_r_joint_is_open(provisioner, library, name):
    repository = provisioner.D6aMotionRepository(
        library, _profile(provisioner)
    )
    motion = repository.load(f'{name}_release')
    assert motion.steps[-1].pose['r_joint'] == pytest.approx(-1.0)


def test_provisions_exact_final_step_open_derivatives(provisioner, tmp_path):
    _make_originals(tmp_path)
    source_bytes = {
        name: (tmp_path / f'{name}.d6a').read_bytes()
        for name in ORIGINAL_NAMES
    }
    open_pulse = _open_pulse(provisioner)

    results = provisioner.provision_release_groups(tmp_path)

    assert [result.name for result in results] == [
        f'{name}_release' for name in ORIGINAL_NAMES
    ]
    assert all(result.created for result in results)
    assert all(result.changed_steps == 1 for result in results)
    for name in ORIGINAL_NAMES:
        source = tmp_path / f'{name}.d6a'
        release = tmp_path / f'{name}_release.d6a'
        expected = _rows(source)
        expected[-1] = expected[-1][:-1] + (open_pulse,)
        assert _rows(release) == expected
        assert _metadata(release) == _metadata(source)
        assert source.read_bytes() == source_bytes[name]
        _assert_final_r_joint_is_open(provisioner, tmp_path, name)


def test_editor_normalised_originals_still_end_open(provisioner, tmp_path):
    _make_originals(tmp_path, gripper_pulses=(596, 596, 596, 596))
    open_pulse = _open_pulse(provisioner)

    results = provisioner.provision_release_groups(tmp_path)

    assert all(result.changed_steps == 1 for result in results)
    for name in ORIGINAL_NAMES:
        source_rows = _rows(tmp_path / f'{name}.d6a')
        release_rows = _rows(tmp_path / f'{name}_release.d6a')
        assert [row[-1] for row in source_rows] == [596, 596, 596, 596]
        assert release_rows[:-1] == source_rows[:-1]
        assert release_rows[-1][:-1] == source_rows[-1][:-1]
        assert release_rows[-1][-1] == open_pulse
        _assert_final_r_joint_is_open(provisioner, tmp_path, name)


def test_stray_non_final_400_is_not_opened(provisioner, tmp_path):
    _make_originals(tmp_path, gripper_pulses=(596, 400, 596, 596))
    open_pulse = _open_pulse(provisioner)

    provisioner.provision_release_groups(tmp_path)

    for name in ORIGINAL_NAMES:
        source_rows = _rows(tmp_path / f'{name}.d6a')
        release_rows = _rows(tmp_path / f'{name}_release.d6a')
        assert release_rows[1][-1] == 400
        assert release_rows[:-1] == source_rows[:-1]
        assert release_rows[-1][:-1] == source_rows[-1][:-1]
        assert release_rows[-1][-1] == open_pulse
        _assert_final_r_joint_is_open(provisioner, tmp_path, name)


def test_existing_files_are_kept_unless_forced(provisioner, tmp_path):
    _make_originals(tmp_path)
    provisioner.provision_release_groups(tmp_path)
    release = tmp_path / 'place_left_release.d6a'
    release.write_bytes(b'operator edit that must not be overwritten')

    results = provisioner.provision_release_groups(tmp_path)

    assert not any(result.created for result in results)
    assert release.read_bytes() == b'operator edit that must not be overwritten'

    forced = provisioner.provision_release_groups(tmp_path, overwrite=True)
    assert all(result.created for result in forced)
    assert _rows(release)[-1][-1] == _open_pulse(provisioner)
    _assert_final_r_joint_is_open(provisioner, tmp_path, 'place_left')


def test_missing_original_fails_before_writing_outputs(provisioner, tmp_path):
    _make_originals(tmp_path)
    missing = tmp_path / 'place_center1.d6a'
    missing.unlink()

    with pytest.raises(provisioner.ProvisioningError, match=str(missing)):
        provisioner.provision_release_groups(tmp_path)

    assert not list(tmp_path.glob('*_release.d6a'))


def test_symlink_destination_is_rejected_before_overwrite(provisioner, tmp_path):
    _make_originals(tmp_path)
    original_bytes = {
        name: (tmp_path / f'{name}.d6a').read_bytes()
        for name in ORIGINAL_NAMES
    }
    destination = tmp_path / 'place_left_release.d6a'
    destination.symlink_to('place_left.d6a')

    with pytest.raises(
        provisioner.ProvisioningError,
        match=r'place_left_release\.d6a is a symlink.*original action group',
    ):
        provisioner.provision_release_groups(tmp_path, overwrite=True)

    assert destination.is_symlink()
    assert destination.resolve() == tmp_path / 'place_left.d6a'
    for name in ORIGINAL_NAMES:
        assert (tmp_path / f'{name}.d6a').read_bytes() == original_bytes[name]
    assert not any(
        (tmp_path / f'{name}_release.d6a').exists()
        for name in ORIGINAL_NAMES[1:]
    )
