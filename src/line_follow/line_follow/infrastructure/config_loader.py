"""Build line-following domain values from a YAML file."""

import math
import os
from typing import Any, List, Mapping, Sequence, Tuple

import yaml

from ..domain.detection import RoiBand
from ..domain.errors import InvalidLineConfigError
from ..domain.line_color import LabTriple, LineColorRange
from ..domain.pid import PidGains


def load_line_follow_config(
    path: str,
) -> Tuple[LineColorRange, List[RoiBand], PidGains]:
    """Read ``path`` and return the line colour, ROI bands, and PID gains.

    Raises:
        FileNotFoundError: if the file is missing.
        InvalidLineConfigError: if the contents are malformed.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"line-follow config not found: {path}")

    with open(path, "r", encoding="utf-8") as handle:
        try:
            data = yaml.safe_load(handle)
        except yaml.YAMLError as exc:
            raise InvalidLineConfigError(
                f"{path}: invalid YAML: {exc}"
            ) from exc

    data = data or {}
    if not isinstance(data, dict):
        raise InvalidLineConfigError(
            f"{path}: top level must be a mapping, got {type(data).__name__}"
        )

    color = _load_color(path, data.get("color"))
    rois = _load_rois(path, data.get("rois"))
    gains = _load_pid(path, data.get("pid"))
    return color, rois, gains


def _load_color(path: str, raw: Any) -> LineColorRange:
    if not isinstance(raw, dict):
        raise InvalidLineConfigError(
            f"{path}: expected a 'color' mapping"
        )

    lab_min = _require_lab_triple(path, "color.lab_min", raw, "lab_min")
    lab_max = _require_lab_triple(path, "color.lab_max", raw, "lab_max")
    min_area_px = _require_mapping_int(
        path, "color", raw, "min_area_px"
    )
    try:
        return LineColorRange(
            lab_min=lab_min,
            lab_max=lab_max,
            min_area_px=min_area_px,
        )
    except InvalidLineConfigError as exc:
        raise InvalidLineConfigError(f"{path}: color: {exc}") from exc


def _load_rois(path: str, raw: Any) -> List[RoiBand]:
    if not isinstance(raw, list) or not raw:
        raise InvalidLineConfigError(
            f"{path}: expected a non-empty 'rois' list"
        )

    rois = []
    for index, spec in enumerate(raw):
        label = f"rois[{index}]"
        if not isinstance(spec, dict):
            raise InvalidLineConfigError(
                f"{path}: {label} must be a mapping"
            )

        values = {
            name: _require_mapping_int(path, label, spec, name)
            for name in ("y_min", "y_max", "x_min", "x_max")
        }
        weight = _require_mapping_number(path, label, spec, "weight")
        try:
            rois.append(RoiBand(weight=weight, **values))
        except InvalidLineConfigError as exc:
            raise InvalidLineConfigError(
                f"{path}: {label}: {exc}"
            ) from exc

    if not any(band.weight > 0.0 for band in rois):
        raise InvalidLineConfigError(
            f"{path}: at least one ROI band must have a positive weight"
        )
    return rois


def _load_pid(path: str, raw: Any) -> PidGains:
    if not isinstance(raw, dict):
        raise InvalidLineConfigError(
            f"{path}: expected a 'pid' mapping"
        )
    return PidGains(
        kp=_require_mapping_number(path, "pid", raw, "kp"),
        ki=_require_mapping_number(path, "pid", raw, "ki"),
        kd=_require_mapping_number(path, "pid", raw, "kd"),
    )


def _require_lab_triple(
    path: str,
    label: str,
    mapping: Mapping[str, Any],
    key: str,
) -> LabTriple:
    value = _require_key(path, label.rsplit(".", 1)[0], mapping, key)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise InvalidLineConfigError(
            f"{path}: {label} must be a 3-item sequence"
        )
    if len(value) != 3:
        raise InvalidLineConfigError(
            f"{path}: {label} must have 3 channels, got {len(value)}"
        )
    return tuple(
        _require_int(path, f"{label}[{index}]", channel)
        for index, channel in enumerate(value)
    )


def _require_mapping_int(
    path: str,
    section: str,
    mapping: Mapping[str, Any],
    key: str,
) -> int:
    return _require_int(
        path,
        f"{section}.{key}",
        _require_key(path, section, mapping, key),
    )


def _require_mapping_number(
    path: str,
    section: str,
    mapping: Mapping[str, Any],
    key: str,
) -> float:
    return _require_number(
        path,
        f"{section}.{key}",
        _require_key(path, section, mapping, key),
    )


def _require_key(
    path: str,
    section: str,
    mapping: Mapping[str, Any],
    key: str,
) -> Any:
    if key not in mapping:
        raise InvalidLineConfigError(
            f"{path}: {section} is missing {key!r}"
        )
    return mapping[key]


def _require_int(path: str, label: str, value: Any) -> int:
    if isinstance(value, bool):
        raise InvalidLineConfigError(
            f"{path}: {label} is boolean; expected an integer"
        )
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            raise InvalidLineConfigError(
                f"{path}: {label} is non-integer ({value!r})"
            )
        return int(value)
    if isinstance(value, int):
        return value
    raise InvalidLineConfigError(
        f"{path}: {label} is non-integer ({value!r})"
    )


def _require_number(path: str, label: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidLineConfigError(
            f"{path}: {label} must be a number, got {value!r}"
        )
    result = float(value)
    if not math.isfinite(result):
        raise InvalidLineConfigError(
            f"{path}: {label} must be finite, got {value!r}"
        )
    return result
