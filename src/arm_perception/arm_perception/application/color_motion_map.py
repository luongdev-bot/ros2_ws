"""In-memory colour -> action-group mapping."""

from dataclasses import dataclass
from typing import Mapping

from ..domain.errors import UnknownColorError
from ..domain.ports import ColorMotionMap


@dataclass(frozen=True)
class MotionBinding:
    """The action group to play for one colour, and its time budget."""

    motion: str
    duration_ms: int = 0

    def __post_init__(self) -> None:
        if not self.motion:
            raise ValueError("motion name must not be empty")
        if self.duration_ms < 0:
            raise ValueError(f"duration_ms must not be negative, got {self.duration_ms}")


class StaticColorMotionMap(ColorMotionMap):
    """A fixed mapping, built once from config and never mutated."""

    def __init__(self, bindings: Mapping[str, MotionBinding]) -> None:
        if not bindings:
            raise ValueError("colour -> motion map must not be empty")
        self._bindings = dict(bindings)

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
