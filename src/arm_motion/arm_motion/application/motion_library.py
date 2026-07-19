"""Use cases for managing the stored action-group library."""

from dataclasses import dataclass
from typing import List

from ..domain.motion import Motion
from ..domain.ports import MotionRepository


@dataclass(frozen=True)
class SaveMotionUseCase:
    """Persist a motion, optionally replacing an existing one."""

    repository: MotionRepository

    def execute(self, motion: Motion, *, overwrite: bool = False) -> Motion:
        motion.require_steps()
        return self.repository.save(motion, overwrite=overwrite)


@dataclass(frozen=True)
class LoadMotionUseCase:
    """Reopen a stored motion, e.g. to edit it in the editor."""

    repository: MotionRepository

    def execute(self, name: str) -> Motion:
        return self.repository.load(name)


@dataclass(frozen=True)
class ListMotionsUseCase:
    repository: MotionRepository

    def execute(self) -> List[Motion]:
        return sorted(self.repository.list(), key=lambda m: m.name)


@dataclass(frozen=True)
class DeleteMotionUseCase:
    repository: MotionRepository

    def execute(self, name: str) -> None:
        self.repository.delete(name)
