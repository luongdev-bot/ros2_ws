"""Task -> motion binding.

Lets other nodes ask for a *task* ("slam3d", "nav", "pick") rather than
hard-coding an action-group name and a duration. The binding lives in
``config/task_motions.yaml``.
"""

from dataclasses import dataclass
from typing import Dict, Mapping, Optional

from ..domain.errors import InvalidMotionError


@dataclass(frozen=True)
class TaskBinding:
    """Which motion a task runs, and how long it is allowed to take.

    Attributes:
        motion_name: Action group to play.
        duration_ms: Total budget; 0 means "use the recorded step timings".
    """

    motion_name: str
    duration_ms: int = 0

    def __post_init__(self) -> None:
        if not self.motion_name:
            raise InvalidMotionError("task binding needs a motion name")
        if self.duration_ms < 0:
            raise InvalidMotionError("task binding duration_ms must be >= 0")


class TaskMotionMap:
    """Immutable lookup from task name to :class:`TaskBinding`."""

    def __init__(self, bindings: Mapping[str, TaskBinding]):
        self._bindings: Dict[str, TaskBinding] = dict(bindings)

    def get(self, task: str) -> Optional[TaskBinding]:
        return self._bindings.get(task)

    def require(self, task: str) -> TaskBinding:
        binding = self.get(task)
        if binding is None:
            known = ", ".join(sorted(self._bindings)) or "(none configured)"
            raise InvalidMotionError(
                f"unknown task '{task}'; configured tasks: {known}"
            )
        return binding

    def tasks(self):
        return sorted(self._bindings)

    def __len__(self) -> int:
        return len(self._bindings)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object]) -> "TaskMotionMap":
        """Build from parsed YAML.

        Accepts either shorthand or the explicit form::

            task_motions:
              nav: travel                          # shorthand
              slam3d: {motion: home, duration_ms: 1000}
        """
        bindings: Dict[str, TaskBinding] = {}
        for task, value in (raw or {}).items():
            if isinstance(value, str):
                bindings[str(task)] = TaskBinding(value)
            elif isinstance(value, Mapping):
                motion = value.get("motion") or value.get("motion_name")
                if not motion:
                    raise InvalidMotionError(
                        f"task '{task}' has no 'motion' key"
                    )
                bindings[str(task)] = TaskBinding(
                    str(motion), int(value.get("duration_ms", 0) or 0)
                )
            else:
                raise InvalidMotionError(
                    f"task '{task}' must map to a motion name or a mapping"
                )
        return cls(bindings)
