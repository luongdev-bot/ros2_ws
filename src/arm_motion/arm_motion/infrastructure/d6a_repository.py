"""SQLite-backed action-group storage — one ``.d6a`` file per motion.

The ``ActionGroup`` table mirrors the Hiwonder editor's layout one-to-one, so
files written here open in ``arm_pc`` and vice versa::

    ActionGroup([Index] INTEGER PRIMARY KEY AUTOINCREMENT, Time INT,
                Servo1 INT, Servo2 INT, ... ServoN INT)

``Servo1..ServoN`` are *positional*: Servo1 is the first joint of the robot
profile, ServoN the last. For the JetRover profile that is
Servo1..Servo5 = joint1..joint5 (bus ID:1..ID:5) and Servo6 = r_joint (ID:10).

A second table carries metadata the original format has no room for. It is
additive, so it does not disturb readers that only know ``ActionGroup``::

    MotionInfo(Key TEXT PRIMARY KEY, Value TEXT)
        name, description, updated_at, schema_version, servo_ids, joint_names

Servo **pulses** (0..1000) are the stored truth, exactly as the editor table
shows them; radians are derived on load through the
:class:`~arm_motion.domain.robot_profile.RobotProfile`. Positions therefore
round-trip at servo resolution — the resolution the hardware actually has.
"""

import json
import os
import sqlite3
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from ..domain.errors import (
    InvalidMotionError,
    MotionAlreadyExistsError,
    MotionNotFoundError,
)
from ..domain.motion import Motion, MotionStep, validate_motion_name
from ..domain.ports import MotionRepository
from ..domain.robot_profile import RobotProfile

SCHEMA_VERSION = "1"
FILE_SUFFIX = ".d6a"


class D6aMotionRepository(MotionRepository):
    """Stores motions as ``.d6a`` SQLite files inside one directory."""

    def __init__(self, directory: os.PathLike, profile: RobotProfile):
        self._dir = Path(directory).expanduser()
        self._profile = profile
        self._dir.mkdir(parents=True, exist_ok=True)

    @property
    def directory(self) -> Path:
        return self._dir

    @property
    def _servo_columns(self) -> List[str]:
        return [f"Servo{i}" for i in range(1, len(self._profile.joints) + 1)]

    def path_for(self, name: str) -> Path:
        """Resolve a motion name to its file path, rejecting traversal."""
        validate_motion_name(name)
        path = (self._dir / f"{name}{FILE_SUFFIX}").resolve()
        # validate_motion_name already blocks separators; this is belt-and-braces
        # so a crafted name can never escape the library directory.
        if path.parent != self._dir.resolve():
            raise InvalidMotionError(f"motion name '{name}' escapes the library")
        return path

    # ------------------------------------------------------------------
    # MotionRepository
    # ------------------------------------------------------------------
    def exists(self, name: str) -> bool:
        return self.path_for(name).is_file()

    def save(self, motion: Motion, *, overwrite: bool = False) -> Motion:
        motion.require_steps()
        path = self.path_for(motion.name)
        if path.exists() and not overwrite:
            raise MotionAlreadyExistsError(
                f"motion '{motion.name}' already exists; pass overwrite to replace it"
            )

        # Reject anything out of range before touching the disk.
        for index, step in enumerate(motion.steps):
            try:
                self._profile.validate_pose(step.pose)
            except Exception as exc:
                raise InvalidMotionError(f"step {index + 1}: {exc}") from exc

        updated_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
        stored = Motion(
            name=motion.name,
            steps=motion.steps,
            description=motion.description,
            updated_at=updated_at,
        )

        # Write to a temp file in the same directory, then atomically replace,
        # so an interrupted save cannot leave a half-written action group.
        fd, tmp_name = tempfile.mkstemp(
            dir=str(self._dir), prefix=f".{motion.name}.", suffix=".tmp"
        )
        os.close(fd)
        tmp_path = Path(tmp_name)
        try:
            self._write(tmp_path, stored)
            if overwrite:
                os.replace(tmp_path, path)
            else:
                self._create_exclusively(tmp_path, path, motion.name)
        finally:
            tmp_path.unlink(missing_ok=True)
        return stored

    def load(self, name: str) -> Motion:
        path = self.path_for(name)
        if not path.is_file():
            raise MotionNotFoundError(f"no motion named '{name}' in {self._dir}")
        return self._read(path)

    def list(self) -> List[Motion]:
        motions: List[Motion] = []
        library = self._dir.resolve()
        for path in sorted(self._dir.glob(f"*{FILE_SUFFIX}")):
            # A symlink could point anywhere; only list files that really live
            # in the library directory.
            if path.is_symlink() or not path.is_file():
                continue
            if path.resolve().parent != library:
                continue
            try:
                motions.append(self._read(path))
            except Exception:
                # A corrupt or foreign file must not break the whole listing.
                continue
        return motions

    def delete(self, name: str) -> None:
        path = self.path_for(name)
        if not path.is_file():
            raise MotionNotFoundError(f"no motion named '{name}' in {self._dir}")
        path.unlink()

    @staticmethod
    def _create_exclusively(tmp_path: Path, path: Path, name: str) -> None:
        """Publish ``tmp_path`` as ``path``, failing if ``path`` exists.

        ``os.link`` makes the existence check and the create a single atomic
        step, so a concurrent writer cannot slip in between them. Filesystems
        without hard links (FAT/exFAT, some network mounts) fall back to a
        check-then-rename, which is the best that can be done there.
        """
        try:
            os.link(tmp_path, path)
            return
        except FileExistsError as exc:
            raise MotionAlreadyExistsError(
                f"motion '{name}' already exists; pass overwrite to replace it"
            ) from exc
        except OSError:
            # Hard links unsupported on this filesystem.
            pass

        if path.exists():
            raise MotionAlreadyExistsError(
                f"motion '{name}' already exists; pass overwrite to replace it"
            )
        os.replace(tmp_path, path)

    # ------------------------------------------------------------------
    # SQLite plumbing
    # ------------------------------------------------------------------
    @staticmethod
    def _connect(path: Path) -> sqlite3.Connection:
        return sqlite3.connect(str(path))

    def _write(self, path: Path, motion: Motion) -> None:
        columns = self._servo_columns
        joints = list(self._profile.joints)
        conn = self._connect(path)
        try:
            with conn:
                conn.execute("DROP TABLE IF EXISTS ActionGroup")
                conn.execute("DROP TABLE IF EXISTS MotionInfo")
                column_ddl = ", ".join(f"{col} INT" for col in columns)
                conn.execute(
                    "CREATE TABLE ActionGroup ("
                    "[Index] INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL, "
                    f"Time INT, {column_ddl})"
                )
                conn.execute(
                    "CREATE TABLE MotionInfo ("
                    "Key TEXT PRIMARY KEY NOT NULL, Value TEXT)"
                )
                conn.executemany(
                    "INSERT INTO MotionInfo (Key, Value) VALUES (?, ?)",
                    [
                        ("schema_version", SCHEMA_VERSION),
                        ("name", motion.name),
                        ("description", motion.description or ""),
                        ("updated_at", motion.updated_at or ""),
                        (
                            "servo_ids",
                            json.dumps(
                                [self._profile.scale(j.name).servo_id for j in joints]
                            ),
                        ),
                        (
                            "joint_names",
                            json.dumps([j.name for j in joints]),
                        ),
                    ],
                )

                placeholders = ", ".join("?" for _ in range(len(columns) + 2))
                insert_sql = (
                    f"INSERT INTO ActionGroup ([Index], Time, {', '.join(columns)}) "
                    f"VALUES ({placeholders})"
                )
                rows = []
                for index, step in enumerate(motion.steps, start=1):
                    pulses = self._profile.pulses_from_pose(step.pose)
                    rows.append(
                        [index, step.duration_ms]
                        + [int(pulses[j.name]) for j in joints]
                    )
                conn.executemany(insert_sql, rows)
        finally:
            conn.close()

    def _read(self, path: Path) -> Motion:
        conn = self._connect(path)
        try:
            info = self._read_info(conn)
            steps = self._read_steps(conn, info)
            # The filename is authoritative: it is what load()/delete() key on.
            # A stale or hand-edited MotionInfo.name must not shadow it.
            return Motion(
                name=path.stem,
                steps=tuple(steps),
                description=info.get("description", ""),
                updated_at=info.get("updated_at") or None,
            )
        finally:
            conn.close()

    @staticmethod
    def _read_info(conn: sqlite3.Connection) -> Dict[str, str]:
        try:
            rows = conn.execute("SELECT Key, Value FROM MotionInfo").fetchall()
        except sqlite3.DatabaseError:
            # A plain arm_pc file has no MotionInfo table — that is fine.
            return {}
        return {str(k): ("" if v is None else str(v)) for k, v in rows}

    def _read_steps(
        self, conn: sqlite3.Connection, info: Dict[str, str]
    ) -> List[MotionStep]:
        try:
            cursor = conn.execute("SELECT * FROM ActionGroup ORDER BY [Index]")
            column_names = [d[0] for d in cursor.description]
            rows = cursor.fetchall()
        except sqlite3.DatabaseError as exc:
            raise InvalidMotionError(f"unreadable action group: {exc}") from exc

        servo_columns = [c for c in column_names if c.lower().startswith("servo")]
        joint_for_column = self._map_columns_to_joints(servo_columns, info)

        time_index = self._column_index(column_names, "Time")
        if time_index is None:
            raise InvalidMotionError("action group has no Time column")

        steps: List[MotionStep] = []
        for row in rows:
            pulses: Dict[str, int] = {}
            for column, joint_name in joint_for_column.items():
                value = row[self._column_index(column_names, column)]
                if value is None:
                    continue
                pulses[joint_name] = int(value)
            pose = self._profile.fill_missing(self._profile.pose_from_pulses(pulses))
            duration = row[time_index]
            steps.append(MotionStep(pose=pose, duration_ms=int(duration or 0)))
        return steps

    @staticmethod
    def _column_index(column_names: Sequence[str], target: str) -> Optional[int]:
        lowered = [c.lower() for c in column_names]
        try:
            return lowered.index(target.lower())
        except ValueError:
            return None

    def _map_columns_to_joints(
        self, servo_columns: Sequence[str], info: Dict[str, str]
    ) -> Dict[str, str]:
        """Decide which joint each ``ServoN`` column drives.

        Prefers the joint names recorded in ``MotionInfo``; falls back to the
        recorded bus ids; finally falls back to positional order, which is how
        a file written by the original editor must be interpreted.
        """
        stored_joints = self._json_list(info.get("joint_names"))
        stored_ids = self._json_list(info.get("servo_ids"))
        profile_joints = [j.name for j in self._profile.joints]

        mapping: Dict[str, str] = {}
        for position, column in enumerate(sorted(servo_columns, key=_servo_sort_key)):
            joint: Optional[str] = None

            if position < len(stored_joints):
                candidate = str(stored_joints[position])
                if candidate in profile_joints:
                    joint = candidate

            if joint is None and position < len(stored_ids):
                try:
                    joint = self._profile.joint_by_servo_id(int(stored_ids[position])).name
                except Exception:
                    joint = None

            if joint is None and position < len(profile_joints):
                joint = profile_joints[position]

            if joint is not None:
                mapping[column] = joint
        return mapping

    @staticmethod
    def _json_list(raw: Optional[str]) -> List[object]:
        if not raw:
            return []
        try:
            parsed = json.loads(raw)
        except (ValueError, TypeError):
            return []
        return parsed if isinstance(parsed, list) else []


def _servo_sort_key(column: str) -> int:
    """Order ``Servo2`` before ``Servo10`` (lexical sort would not)."""
    digits = "".join(ch for ch in column if ch.isdigit())
    return int(digits) if digits else 0
