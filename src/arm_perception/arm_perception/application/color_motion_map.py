"""In-memory colour -> action-group mapping."""

from dataclasses import dataclass
from typing import Mapping, Optional, Sequence, Tuple

from ..domain.errors import UnknownColorError
from ..domain.motion_step import MotionStep
from ..domain.ports import ColorMotionMap


@dataclass(frozen=True)
class MotionBinding:
    """The action group(s) to play for one colour.

    ``motion`` is the (required) first step - the grasp. ``place`` is an
    optional second step run only after the grasp succeeds, so a colour can
    pick a block and then drop it in a colour-specific bin.
    """

    motion: str
    duration_ms: int = 0
    place: Optional[str] = None
    place_duration_ms: int = 0

    def __post_init__(self) -> None:
        if not self.motion or not self.motion.strip():
            raise ValueError("motion name must not be blank")
        if self.duration_ms < 0:
            raise ValueError(f"duration_ms must not be negative, got {self.duration_ms}")
        if self.place is not None:
            if not self.place or not self.place.strip():
                raise ValueError("place name must not be blank when set")
            if self.place_duration_ms < 0:
                raise ValueError(
                    f"place_duration_ms must not be negative, got {self.place_duration_ms}"
                )
        elif self.place_duration_ms:
            # A budget with no place step to spend it on is a mistake, not a
            # value to silently drop.
            raise ValueError(
                f"place_duration_ms ({self.place_duration_ms}) set without place"
            )

    def steps(self) -> Tuple[MotionStep, ...]:
        steps = [MotionStep(self.motion, self.duration_ms)]
        if self.place is not None:
            steps.append(MotionStep(self.place, self.place_duration_ms))
        return tuple(steps)


class StaticColorMotionMap(ColorMotionMap):
    """A fixed mapping, built once from config and never mutated."""

    def __init__(self, bindings: Mapping[str, MotionBinding]) -> None:
        if not bindings:
            raise ValueError("colour -> motion map must not be empty")
        self._bindings = dict(bindings)

    def steps_for(self, color: str) -> Sequence[MotionStep]:
        return self._binding(color).steps()

    def motion_for(self, color: str) -> str:
        return self._binding(color).motion

    def duration_for(self, color: str) -> int:
        return self._binding(color).duration_ms

    def as_dict(self) -> Mapping[str, str]:
        return {color: b.motion for color, b in self._bindings.items()}

    def colors(self):
        return tuple(self._bindings)

    def _binding(self, color: str) -> MotionBinding:
        try:
            return self._bindings[color]
        except KeyError:
            known = ", ".join(sorted(self._bindings)) or "<none>"
            raise UnknownColorError(
                f"no action group mapped for colour {color!r} (known: {known})"
            ) from None
