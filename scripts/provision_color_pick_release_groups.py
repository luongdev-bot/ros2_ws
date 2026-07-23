#!/usr/bin/env python3
"""Provision colour-pick place groups that end with the gripper open.

``arm_motion`` defines the gripper's OPEN detent in the robot profile and
maps that joint position back to the pulse stored in a ``.d6a`` file.  Each
``place_*_release.d6a`` is a deterministic copy of its ``place_*.d6a``
original with only the final step's gripper pulse set to that profile-derived
OPEN pulse.  Step count, indexes, timings, every earlier gripper value, the
other servo values, and any additive metadata are preserved exactly.

The arm_motion repository owns path and schema validation.  The one raw SQL
update is intentional: loading and saving through the domain model would snap
*every* gripper pulse to a detent and would no longer preserve the source rows.
"""

import argparse
import math
import os
import shutil
import sqlite3
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple


WORKSPACE = Path(__file__).resolve().parents[1]
ARM_MOTION_SOURCE = WORKSPACE / "src" / "arm_motion"
if str(ARM_MOTION_SOURCE) not in sys.path:
    sys.path.insert(0, str(ARM_MOTION_SOURCE))

from arm_motion.domain.errors import MotionAlreadyExistsError  # noqa: E402
from arm_motion.domain.joint_spec import GripperCommand  # noqa: E402
from arm_motion.infrastructure.config_loader import (  # noqa: E402
    build_robot_profile,
    load_yaml,
)
from arm_motion.infrastructure.d6a_repository import (  # noqa: E402
    D6aMotionRepository,
)


ORIGINAL_NAMES: Tuple[str, ...] = (
    "place_left",
    "place_center",
    "place_center1",
    "place_right",
)
RELEASE_SUFFIX = "_release"
ROBOT_CONFIG = WORKSPACE / "src" / "arm_motion" / "config" / "jetrover_arm.yaml"


class ProvisioningError(RuntimeError):
    """The release groups could not be safely provisioned."""


@dataclass(frozen=True)
class ProvisionResult:
    """Outcome for one release group."""

    name: str
    path: Path
    changed_steps: int
    created: bool


@dataclass(frozen=True)
class _GripperEncoding:
    """The profile-backed ``.d6a`` representation of the OPEN detent."""

    joint_name: str
    column: str
    column_index: int
    open_position: float
    open_pulse: int
    action_columns: Tuple[str, ...]


def _gripper_encoding(profile) -> _GripperEncoding:
    """Derive the gripper column and OPEN pulse from ``arm_motion``."""
    grippers = profile.gripper_joints()
    if len(grippers) != 1:
        raise ProvisioningError(
            "colour-pick provisioning requires exactly one gripper joint; "
            f"the robot profile defines {len(grippers)}"
        )

    gripper = grippers[0]
    slot = profile.joint_names.index(gripper.name) + 1
    column = f"Servo{slot}"
    action_columns = ("Index", "Time") + tuple(
        f"Servo{index}" for index in range(1, len(profile.joints) + 1)
    )
    open_position = float(gripper.gripper_position(GripperCommand.OPEN))
    open_pulse = profile.scale(gripper.name).to_pulse(open_position)
    round_trip = profile.pose_from_pulses(
        {gripper.name: open_pulse}
    )[gripper.name]
    if not math.isclose(
        round_trip, open_position, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ProvisioningError(
            f"profile pulse {open_pulse} for {gripper.name} does not map "
            f"back to its OPEN detent {open_position:.2f}"
        )
    return _GripperEncoding(
        joint_name=gripper.name,
        column=column,
        column_index=action_columns.index(column),
        open_position=open_position,
        open_pulse=open_pulse,
        action_columns=action_columns,
    )


def _action_rows(
    path: Path, action_columns: Sequence[str]
) -> List[Tuple[int, ...]]:
    """Read the fields this derivation promises to preserve."""
    columns = ", ".join(f"[{column}]" for column in action_columns)
    try:
        with sqlite3.connect(str(path)) as connection:
            rows = connection.execute(
                f"SELECT {columns} FROM ActionGroup ORDER BY [Index]"
            ).fetchall()
    except sqlite3.DatabaseError as exc:
        raise ProvisioningError(
            f"invalid action group {path}: expected ActionGroup with "
            "Index, Time, and all robot-profile Servo columns"
        ) from exc
    if not rows:
        raise ProvisioningError(f"invalid action group {path}: no motion steps")
    return [tuple(int(value) for value in row) for row in rows]


def _open_final_step(
    path: Path,
    source_rows: Sequence[Tuple[int, ...]],
    encoding: _GripperEncoding,
) -> int:
    """Set only the final indexed row to OPEN; return the values changed."""
    final_row = source_rows[-1]
    if final_row[encoding.column_index] == encoding.open_pulse:
        return 0

    try:
        with sqlite3.connect(str(path)) as connection:
            cursor = connection.execute(
                f"UPDATE ActionGroup SET [{encoding.column}] = ? "
                "WHERE [Index] = ?",
                (encoding.open_pulse, final_row[0]),
            )
    except sqlite3.DatabaseError as exc:
        raise ProvisioningError(
            f"could not update staged action group {path}: {exc}"
        ) from exc
    if cursor.rowcount != 1:
        raise ProvisioningError(
            f"could not identify the final step of staged action group "
            f"{path.name} by Index={final_row[0]}"
        )
    return 1


def _verify_derivation(
    source_rows: Sequence[Tuple[int, ...]],
    destination_path: Path,
    changed_steps: int,
    encoding: _GripperEncoding,
) -> None:
    """Prove that only the final step's gripper encoding can differ."""
    destination_rows = _action_rows(
        destination_path, encoding.action_columns
    )
    expected_rows = list(source_rows)
    final_row = list(expected_rows[-1])
    final_row[encoding.column_index] = encoding.open_pulse
    expected_rows[-1] = tuple(final_row)
    if destination_rows != expected_rows:
        raise ProvisioningError(
            f"generated action group {destination_path.name} changed fields "
            f"other than the final step's {encoding.column} value"
        )
    expected_changes = int(
        source_rows[-1][encoding.column_index] != encoding.open_pulse
    )
    if changed_steps != expected_changes:
        raise ProvisioningError(
            f"generated action group {destination_path.name} changed "
            f"{changed_steps} values; expected {expected_changes}"
        )
    if destination_rows[-1][encoding.column_index] != encoding.open_pulse:
        raise ProvisioningError(
            f"generated action group {destination_path.name} does not end "
            "with the gripper OPEN"
        )


def _require_loaded_final_step_open(
    motion,
    encoding: _GripperEncoding,
) -> None:
    """Reject a staged group unless arm_motion loads its final step as OPEN."""
    final_position = motion.steps[-1].pose[encoding.joint_name]
    if not math.isclose(
        final_position,
        encoding.open_position,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ProvisioningError(
            f"generated action group {motion.name}.d6a ends with "
            f"{encoding.joint_name}={final_position:.2f}, not the profile's "
            f"OPEN detent {encoding.open_position:.2f}"
        )


def _destination_path(
    repository: D6aMotionRepository,
    release_name: str,
    original_paths: Sequence[Path],
) -> Path:
    """Return the literal library destination, rejecting symlink aliases."""
    library = repository.directory.resolve()
    literal = repository.directory / f"{release_name}.d6a"
    intended = library / f"{release_name}.d6a"

    if literal.is_symlink():
        try:
            target = literal.resolve(strict=False)
        except RuntimeError:
            target = literal
        if target in original_paths:
            detail = f"; it resolves to original action group {target}"
        else:
            detail = f"; it resolves to {target}"
        raise ProvisioningError(
            f"release destination {literal} is a symlink{detail}. "
            "Refusing to replace it"
        )

    try:
        resolved = repository.path_for(release_name)
    except Exception as exc:
        raise ProvisioningError(
            f"unsafe release destination {literal}: {exc}"
        ) from exc
    if resolved in original_paths:
        raise ProvisioningError(
            f"release destination {literal} resolves to original action "
            f"group {resolved}; refusing to replace it"
        )
    if resolved != intended:
        raise ProvisioningError(
            f"release destination {literal} resolves outside its intended "
            f"file ({resolved}); refusing to replace it"
        )
    return intended


def provision_release_groups(
    library_dir: Path,
    *,
    overwrite: bool = False,
    robot_config: Path = ROBOT_CONFIG,
) -> Sequence[ProvisionResult]:
    """Create all four release groups from originals in ``library_dir``.

    Existing release groups are reported and left untouched unless
    ``overwrite`` is true.  All originals are checked before any destination
    is written, so a missing or invalid source cannot leave a partial set.
    """
    library_dir = Path(library_dir).expanduser()
    if not library_dir.is_dir():
        raise ProvisioningError(f"action-group library not found: {library_dir}")

    profile = build_robot_profile(load_yaml(Path(robot_config)))
    encoding = _gripper_encoding(profile)
    repository = D6aMotionRepository(library_dir, profile)
    sources = {
        name: repository.path_for(name)
        for name in ORIGINAL_NAMES
    }
    missing = [path for path in sources.values() if not path.is_file()]
    if missing:
        formatted = "\n".join(f"  - {path}" for path in missing)
        raise ProvisioningError(
            "missing original action group(s); no release groups were written:\n"
            f"{formatted}"
        )

    # Reuse the authoritative repository reader to reject corrupt, foreign,
    # or empty source files before staging any output.
    source_rows = {}
    for name in ORIGINAL_NAMES:
        try:
            repository.load(name).require_steps()
            source_rows[name] = _action_rows(
                sources[name], encoding.action_columns
            )
        except Exception as exc:
            raise ProvisioningError(
                f"cannot load original action group {sources[name]}: {exc}"
            ) from exc

    results: List[ProvisionResult] = []
    to_generate = []
    original_paths = tuple(sources.values())
    for original_name in ORIGINAL_NAMES:
        release_name = original_name + RELEASE_SUFFIX
        destination = _destination_path(
            repository, release_name, original_paths
        )
        if destination.exists() and not overwrite:
            results.append(
                ProvisionResult(release_name, destination, 0, created=False)
            )
        else:
            to_generate.append((original_name, release_name, destination))

    if not to_generate:
        return tuple(results)

    with tempfile.TemporaryDirectory(
        dir=str(library_dir), prefix=".color-pick-release-"
    ) as staging_dir:
        staging = Path(staging_dir)
        staged_repository = D6aMotionRepository(staging, profile)
        staged = []

        for original_name, release_name, destination in to_generate:
            source = sources[original_name]
            staged_path = staged_repository.path_for(release_name)
            shutil.copyfile(source, staged_path)
            changed_steps = _open_final_step(
                staged_path, source_rows[original_name], encoding
            )
            _verify_derivation(
                source_rows[original_name],
                staged_path,
                changed_steps,
                encoding,
            )
            try:
                motion = staged_repository.load(release_name)
                motion.require_steps()
                _require_loaded_final_step_open(motion, encoding)
            except Exception as exc:
                raise ProvisioningError(
                    f"generated action group {staged_path} failed arm_motion "
                    f"validation: {exc}"
                ) from exc
            staged.append(
                (release_name, staged_path, destination, changed_steps)
            )

        for release_name, staged_path, destination, changed_steps in staged:
            try:
                if overwrite:
                    os.replace(staged_path, destination)
                else:
                    repository._create_exclusively(
                        staged_path, destination, release_name
                    )
            except MotionAlreadyExistsError as exc:
                raise ProvisioningError(str(exc)) from exc
            results.append(
                ProvisionResult(
                    release_name,
                    destination,
                    changed_steps,
                    created=True,
                )
            )

    order = {name + RELEASE_SUFFIX: index for index, name in enumerate(ORIGINAL_NAMES)}
    return tuple(sorted(results, key=lambda result: order[result.name]))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate colour-pick _release action groups from place groups."
    )
    parser.add_argument(
        "--library-dir",
        type=Path,
        default=Path(
            os.environ.get("ARM_MOTION_LIBRARY_DIR", "~/ActionGroups")
        ).expanduser(),
        help="action-group library (default: ARM_MOTION_LIBRARY_DIR or ~/ActionGroups)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="replace existing _release files; required after re-teaching originals",
    )
    return parser


def main(argv: Sequence[str] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        results = provision_release_groups(
            args.library_dir,
            overwrite=args.force,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    created = 0
    for result in results:
        if result.created:
            created += 1
            print(
                f"created {result.path} "
                f"(final gripper set to the profile's OPEN detent; "
                f"{result.changed_steps} value(s) changed)"
            )
        else:
            print(
                f"kept {result.path} (already exists; use --force to replace)"
            )
    if created == 0:
        print("All colour-pick release groups already exist; nothing changed.")
    else:
        print(f"Provisioned {created} colour-pick release group(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
