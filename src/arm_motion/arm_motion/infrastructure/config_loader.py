"""Build domain objects from YAML configuration."""

from pathlib import Path
from typing import Any, Dict, List, Mapping

import yaml

from ..application.task_motions import TaskMotionMap
from ..domain.joint_spec import JointKind, JointSpec
from ..domain.robot_profile import RobotProfile, build_profile
from ..domain.servo_scale import (
    DEFAULT_MAX_PULSE,
    DEFAULT_MAX_RAD,
    DEFAULT_MIN_PULSE,
    DEFAULT_MIN_RAD,
    ServoScale,
)


class ConfigError(Exception):
    """The configuration file is missing or malformed."""


def load_yaml(path: Path) -> Dict[str, Any]:
    path = Path(path).expanduser()
    if not path.is_file():
        raise ConfigError(f"configuration file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ConfigError(f"{path}: expected a mapping at the top level")
    return dict(data)


def build_robot_profile(config: Mapping[str, Any]) -> RobotProfile:
    """Construct a :class:`RobotProfile` from the ``joints:`` section."""
    raw_joints = config.get("joints")
    if not raw_joints:
        raise ConfigError("configuration has no 'joints' section")
    if not isinstance(raw_joints, list):
        raise ConfigError("'joints' must be a list")

    specs: List[JointSpec] = []
    scales: List[ServoScale] = []

    for entry in raw_joints:
        if not isinstance(entry, Mapping):
            raise ConfigError("each entry of 'joints' must be a mapping")
        try:
            name = str(entry["name"])
            servo_id = int(entry["servo_id"])
        except KeyError as exc:
            raise ConfigError(f"joint entry missing required key: {exc}") from exc

        kind_raw = str(entry.get("kind", JointKind.REVOLUTE.value)).lower()
        try:
            kind = JointKind(kind_raw)
        except ValueError as exc:
            valid = ", ".join(k.value for k in JointKind)
            raise ConfigError(
                f"joint '{name}': unknown kind '{kind_raw}' (valid: {valid})"
            ) from exc

        try:
            lower = float(entry["lower"])
            upper = float(entry["upper"])
        except KeyError as exc:
            raise ConfigError(
                f"joint '{name}' missing limit key: {exc}"
            ) from exc

        try:
            spec = JointSpec(
                name=name,
                lower=lower,
                upper=upper,
                kind=kind,
                jog_step=float(entry.get("jog_step", 0.05)),
                group=str(entry.get("group", "arm")),
                open_position=_optional_float(entry.get("open_position")),
                closed_position=_optional_float(entry.get("closed_position")),
            )
            scale = ServoScale(
                servo_id=servo_id,
                joint_name=name,
                min_pulse=int(entry.get("min_pulse", DEFAULT_MIN_PULSE)),
                max_pulse=int(entry.get("max_pulse", DEFAULT_MAX_PULSE)),
                min_rad=float(entry.get("min_rad", DEFAULT_MIN_RAD)),
                max_rad=float(entry.get("max_rad", DEFAULT_MAX_RAD)),
                invert=bool(entry.get("invert", False)),
            )
        except ValueError as exc:
            raise ConfigError(f"joint '{name}': {exc}") from exc

        specs.append(spec)
        scales.append(scale)

    try:
        return build_profile(specs, scales)
    except ValueError as exc:
        raise ConfigError(str(exc)) from exc


def build_task_motions(config: Mapping[str, Any]) -> TaskMotionMap:
    """Construct the task -> motion mapping from ``task_motions:``."""
    raw = config.get("task_motions") or {}
    if not isinstance(raw, Mapping):
        raise ConfigError("'task_motions' must be a mapping")
    try:
        return TaskMotionMap.from_mapping(raw)
    except Exception as exc:
        raise ConfigError(f"task_motions: {exc}") from exc


def build_controller_groups(config: Mapping[str, Any]) -> Dict[str, str]:
    """Map a joint group name to its FollowJointTrajectory action namespace."""
    raw = config.get("controllers") or {}
    if not isinstance(raw, Mapping):
        raise ConfigError("'controllers' must be a mapping")
    groups = {str(group): str(ns) for group, ns in raw.items()}
    if not groups:
        raise ConfigError("configuration has no 'controllers' section")
    return groups


def _optional_float(value: Any):
    return None if value is None else float(value)
