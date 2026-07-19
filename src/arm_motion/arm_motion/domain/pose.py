"""A full-arm pose: one position per joint, in radians."""

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Dict, Iterable, Mapping


@dataclass(frozen=True)
class Pose:
    """Immutable snapshot of every joint position, in radians.

    A ``Pose`` carries no validation of its own — validity is defined
    relative to a :class:`~arm_motion.domain.robot_profile.RobotProfile`,
    which owns the limits. Use ``RobotProfile.validate_pose`` /
    ``RobotProfile.clamp_pose``.
    """

    positions: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # Defensive copy, then freeze, so callers cannot mutate a stored pose.
        frozen = MappingProxyType(dict(self.positions))
        object.__setattr__(self, "positions", frozen)

    def __getitem__(self, joint_name: str) -> float:
        return self.positions[joint_name]

    def __contains__(self, joint_name: str) -> bool:
        return joint_name in self.positions

    def __iter__(self):
        return iter(self.positions)

    def joint_names(self) -> Iterable[str]:
        return self.positions.keys()

    def as_dict(self) -> Dict[str, float]:
        """Mutable copy, for adapters that need to build messages."""
        return dict(self.positions)

    def with_joint(self, joint_name: str, position: float) -> "Pose":
        """Return a new pose with one joint changed."""
        updated = dict(self.positions)
        updated[joint_name] = position
        return Pose(updated)

    def subset(self, joint_names: Iterable[str]) -> "Pose":
        """Return a new pose restricted to ``joint_names`` present here."""
        names = [n for n in joint_names if n in self.positions]
        return Pose({n: self.positions[n] for n in names})

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Pose):
            return NotImplemented
        return dict(self.positions) == dict(other.positions)

    def __hash__(self) -> int:
        return hash(tuple(sorted(self.positions.items())))
