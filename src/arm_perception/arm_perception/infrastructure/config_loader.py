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
        data = yaml.safe_load(handle) or {}

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
            try:
                duration_ms = int(spec.get("duration_ms", 0))
            except (TypeError, ValueError):
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} has a non-integer 'duration_ms'"
                ) from None
            if duration_ms < 0:
                raise InvalidColorRangeError(
                    f"{path}: colour {name!r} has a negative 'duration_ms'"
                )
            bindings[name] = MotionBinding(motion=motion, duration_ms=duration_ms)

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

    try:
        min_area_px = int(spec.get("min_area_px", 200))
    except (TypeError, ValueError):
        # Keep every config fault on one error type, so callers need only
        # catch InvalidColorRangeError.
        raise InvalidColorRangeError(
            f"{path}: colour {name!r} has a non-integer 'min_area_px'"
        ) from None

    return ColorRange(
        name=name,
        lab_min=lab_min,
        lab_max=lab_max,
        min_area_px=min_area_px,
    )
