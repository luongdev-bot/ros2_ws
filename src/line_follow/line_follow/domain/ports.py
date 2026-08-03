"""Ports — interfaces the domain owns and infrastructure implements."""

from abc import ABC, abstractmethod
from typing import Any, Optional

from .detection import LineDetection


class LineDetector(ABC):
    """Finds a line's weighted horizontal position in a camera frame."""

    @abstractmethod
    def detect(self, frame: Any) -> Optional[LineDetection]:
        """Return the line estimate in ``frame``, or ``None`` when absent."""
