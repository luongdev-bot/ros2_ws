"""Per-joint capability description — the single source of truth for limits.

Every command in this package passes through :class:`JointSpec`, so a joint
can only ever be asked to do what it is physically capable of:

* a ``REVOLUTE`` joint rotates, bounded by ``[lower, upper]``;
* a ``GRIPPER`` joint only opens and closes — continuous jogging is rejected.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .errors import (
    JointLimitError,
    UnsupportedJointMotionError,
)


class JointKind(str, Enum):
    """What kind of motion a joint is capable of."""

    REVOLUTE = "revolute"
    GRIPPER = "gripper"


class GripperCommand(str, Enum):
    OPEN = "open"
    CLOSE = "close"


@dataclass(frozen=True)
class JointSpec:
    """Immutable description of one controllable joint.

    Attributes:
        name: Joint name as it appears in the URDF / controller config.
        lower: Lower position limit, in radians.
        upper: Upper position limit, in radians.
        kind: Which motions this joint supports.
        jog_step: Default increment for one jog keystroke, in radians.
        group: Controller group this joint belongs to ("arm", "gripper", ...).
        open_position: Gripper only — position meaning "open".
        closed_position: Gripper only — position meaning "closed".
    """

    name: str
    lower: float
    upper: float
    kind: JointKind = JointKind.REVOLUTE
    jog_step: float = 0.05
    group: str = "arm"
    open_position: Optional[float] = None
    closed_position: Optional[float] = None

    def __post_init__(self) -> None:
        if self.lower > self.upper:
            raise ValueError(
                f"joint '{self.name}': lower ({self.lower}) > upper ({self.upper})"
            )
        if self.jog_step <= 0.0:
            raise ValueError(f"joint '{self.name}': jog_step must be > 0")
        if self.kind is JointKind.GRIPPER:
            if self.open_position is None or self.closed_position is None:
                raise ValueError(
                    f"gripper joint '{self.name}' needs both open_position "
                    "and closed_position"
                )
            # Both named positions must themselves respect the limits,
            # otherwise the gripper could be told to exceed them.
            for label, value in (
                ("open_position", self.open_position),
                ("closed_position", self.closed_position),
            ):
                if not self._within(value):
                    raise ValueError(
                        f"gripper joint '{self.name}': {label} ({value}) is "
                        f"outside [{self.lower}, {self.upper}]"
                    )

    # ------------------------------------------------------------------
    # Limit checking
    # ------------------------------------------------------------------
    def _within(self, position: float, tolerance: float = 1e-9) -> bool:
        return self.lower - tolerance <= position <= self.upper + tolerance

    def validate(self, position: float) -> float:
        """Return ``position`` unchanged, or raise if it violates the limits."""
        if not self._within(position):
            raise JointLimitError(self.name, position, self.lower, self.upper)
        return position

    def clamp(self, position: float) -> float:
        """Return ``position`` squeezed into ``[lower, upper]``."""
        return min(max(position, self.lower), self.upper)

    def is_clamped(self, position: float) -> bool:
        """True when ``position`` would be modified by :meth:`clamp`."""
        return not self._within(position)

    # ------------------------------------------------------------------
    # Motion capability
    # ------------------------------------------------------------------
    def jog(self, current: float, delta: float) -> float:
        """Return ``current + delta`` clamped to the limits.

        Raises:
            UnsupportedJointMotionError: if this joint cannot be jogged
                continuously (i.e. it is a gripper).
        """
        if self.kind is not JointKind.REVOLUTE:
            raise UnsupportedJointMotionError(
                f"joint '{self.name}' is a {self.kind.value} joint and cannot "
                "be jogged continuously; use open/close instead"
            )
        return self.clamp(current + delta)

    def gripper_position(self, command: GripperCommand) -> float:
        """Resolve an open/close command to a joint position.

        Raises:
            UnsupportedJointMotionError: if this joint is not a gripper.
        """
        if self.kind is not JointKind.GRIPPER:
            raise UnsupportedJointMotionError(
                f"joint '{self.name}' is not a gripper; open/close does not apply"
            )
        # __post_init__ guarantees both are set for GRIPPER joints.
        return (
            self.open_position
            if command is GripperCommand.OPEN
            else self.closed_position
        )

    # ------------------------------------------------------------------
    # Detents — a gripper is only ever fully open or fully closed
    # ------------------------------------------------------------------
    def detents(self) -> tuple:
        """The discrete positions this joint may hold, or () if continuous."""
        if self.kind is not JointKind.GRIPPER:
            return ()
        return (float(self.open_position), float(self.closed_position))

    def nearest_detent(self, position: float) -> float:
        """Snap a gripper position to whichever of open/closed is closer."""
        detents = self.detents()
        if not detents:
            return position
        return min(detents, key=lambda d: abs(position - d))

    def is_at_detent(self, position: float, tolerance: float = 1e-3) -> bool:
        """True for a continuous joint, or for a gripper at open/closed."""
        detents = self.detents()
        if not detents:
            return True
        return any(abs(position - d) <= tolerance for d in detents)

    def snap(self, position: float) -> float:
        """Coerce a raw number into something this joint may actually hold."""
        if self.kind is JointKind.GRIPPER:
            return self.nearest_detent(position)
        return self.clamp(position)

    def describe_position(self, position: float) -> str:
        """Human-readable rendering of a position, for the editor UI."""
        if self.kind is JointKind.GRIPPER:
            open_dist = abs(position - float(self.open_position))
            close_dist = abs(position - float(self.closed_position))
            label = "OPEN" if open_dist <= close_dist else "CLOSED"
            return f"{label} ({position:+.3f})"
        return f"{position:+.3f} rad"
