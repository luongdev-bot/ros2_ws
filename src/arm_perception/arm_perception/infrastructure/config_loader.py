"""Builds the palette and the colour -> motion map from a YAML file."""

import os
from typing import Any, Dict, Tuple

import yaml

from ..application.color_motion_map import MotionBinding, StaticColorMotionMap
from ..domain.color_range import ColorPalette, ColorRange
from ..domain.errors import InvalidColorRangeError


def load_color_pick_config(
    path: str,
) -> Tuple[ColorPalette, StaticColorMotionMap]:
    """Read ``path`` and return the palette plus its motion bindings.

    Raises:
        FileNotFoundError: if the file is missing.
        InvalidColorRangeError: if the contents are malformed.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"colour config not found: {path}")

    with open(path, "r", encoding="utf-8") as handle:
        try:
            data = yaml.safe_load(handle)
        except yaml.YAMLError as exc:
            # Normalise a parser error into the loader's documented failure
            # so callers only ever have to catch InvalidColorRangeError.
            raise InvalidColorRangeError(f"{path}: invalid YAML: {exc}") from exc
    data = data or {}
    if not isinstance(data, dict):
        raise InvalidColorRangeError(
            f"{path}: top level must be a mapping, got {type(data).__name__}"
        )

    colors = data.get("colors")
    if not isinstance(colors, dict) or not colors:
        raise InvalidColorRangeError(
            f"{path}: expected a non-empty 'colors' mapping"
        )

    ranges = {}
    bindings = {}
    for name, spec in colors.items():
        if not isinstance(spec, dict):
            raise InvalidColorRangeError(f"{path}: colour {name!r} must be a mapping")

        ranges[name] = _build_range(path, name, spec)

        # A colour may legitimately omit 'motion' (detect-only, e.g. while
        # tuning thresholds), but a present-but-empty value is a typo -
        # silently skipping it would leave a colour that never fires with
        # no indication why.
        if "motion" in spec:
            raw_motion = spec["motion"]
            # A motion name is a filename stem. Anything that is not a string
            # (`motion: 0`, `motion: [pick]`) is a typo, not a name to coerce.
            if raw_motion is not None and not isinstance(raw_motion, str):
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} has a non-string 'motion' "
                    f"({raw_motion!r}) - expected an action group name"
                )
            motion = (raw_motion or "").strip()
            if not motion:
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} has an empty 'motion' - "
                    "remove the key to make the colour detect-only"
                )
            duration_ms = _require_int(
                path, name, "duration_ms", spec.get("duration_ms", 0)
            )
            if duration_ms < 0:
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} has a negative 'duration_ms'"
                )

            # Optional second step: after the shared grasp, drop the block in a
            # colour-specific bin. Same 'present-but-empty is a typo' rule as
            # 'motion': omit the key for pick-only, never leave it blank.
            place = None
            place_duration_ms = 0
            if "place" in spec:
                raw_place = spec["place"]
                if raw_place is not None and not isinstance(raw_place, str):
                    raise InvalidColorRangeError(
                        f"{path}: colour {name!r} has a non-string 'place' "
                        f"({raw_place!r}) - expected an action group name"
                    )
                place = (raw_place or "").strip()
                if not place:
                    raise InvalidColorRangeError(
                        f"{path}: colour {name!r} has an empty 'place' - "
                        "remove the key to pick without placing"
                    )
                place_duration_ms = _require_int(
                    path, name, "place_duration_ms", spec.get("place_duration_ms", 0)
                )
                if place_duration_ms < 0:
                    raise InvalidColorRangeError(
                        f"{path}: colour {name!r} has a negative 'place_duration_ms'"
                    )
            elif "place_duration_ms" in spec:
                # A budget with nothing to spend it on is a typo (most likely a
                # forgotten or misspelled 'place'), not a silently-ignored key.
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} sets 'place_duration_ms' without 'place'"
                )

            bindings[name] = MotionBinding(
                motion=motion,
                duration_ms=duration_ms,
                place=place,
                place_duration_ms=place_duration_ms,
            )

    if not bindings:
        raise InvalidColorRangeError(
            f"{path}: no colour defines a 'motion' - nothing could ever be picked"
        )

    return ColorPalette(ranges), StaticColorMotionMap(bindings)


def _build_range(path: str, name: str, spec: Dict[str, Any]) -> ColorRange:
    try:
        lab_min = tuple(int(v) for v in spec["lab_min"])
        lab_max = tuple(int(v) for v in spec["lab_max"])
    except KeyError as exc:
        raise InvalidColorRangeError(
            f"{path}: colour {name!r} is missing {exc.args[0]!r}"
        ) from None
    except (TypeError, ValueError) as exc:
        raise InvalidColorRangeError(
            f"{path}: colour {name!r} has non-integer LAB bounds: {exc}"
        ) from None

    min_area_px = _require_int(path, name, "min_area_px", spec.get("min_area_px", 200))

    raw_max = spec.get("max_area_px")
    max_area_px = (
        None if raw_max is None
        else _require_int(path, name, "max_area_px", raw_max)
    )

    return ColorRange(
        name=name,
        lab_min=lab_min,
        lab_max=lab_max,
        min_area_px=min_area_px,
        max_area_px=max_area_px,
    )


def _require_int(path: str, color: str, field: str, value: Any) -> int:
    """Accept only whole numbers.

    ``int(1.5)`` would silently truncate to 1 and quietly change a motion's
    timing, so a fractional value is a config error rather than something to
    round. ``2.0`` is accepted - YAML writes some integers that way.
    """
    if isinstance(value, bool):
        raise InvalidColorRangeError(
            f"{path}: colour {color!r} has a boolean {field!r}; expected a number"
        )
    if isinstance(value, float):
        if not value.is_integer():
            raise InvalidColorRangeError(
                f"{path}: colour {color!r} has a fractional {field!r} ({value}); "
                "it must be a whole number"
            )
        return int(value)
    if isinstance(value, int):
        return value
    raise InvalidColorRangeError(
        f"{path}: colour {color!r} has a non-integer {field!r} ({value!r})"
    )
