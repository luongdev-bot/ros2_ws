"""The robot's controllable joints, their limits, and their servo mapping.

This is the aggregate that enforces "each joint may only do what it is
allowed to do, within the range it is allowed to do it in".
"""

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .errors import (
    IncompletePoseError,
    UnknownJointError,
    UnsupportedJointMotionError,
)
from .joint_spec import GripperCommand, JointKind, JointSpec
from .pose import Pose
from .servo_scale import ServoScale


@dataclass(frozen=True)
class RobotProfile:
    """Ordered set of joints plus their pulse<->radian scaling.

    Attributes:
        joints: Joints in editor display order (base first, gripper last).
        scales: Servo scaling, keyed by joint name.
    """

    joints: Sequence[JointSpec] = field(default_factory=tuple)
    scales: Mapping[str, ServoScale] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "scales", dict(self.scales))

        if not self.joints:
            raise ValueError("robot profile needs at least one joint")

        names = [j.name for j in self.joints]
        duplicates = {n for n in names if names.count(n) > 1}
        if duplicates:
            raise ValueError(
                "duplicate joint names in profile: " + ", ".join(sorted(duplicates))
            )

        missing_scale = [n for n in names if n not in self.scales]
        if missing_scale:
            raise ValueError(
                "joints without a servo scale: " + ", ".join(missing_scale)
            )

        servo_ids = [self.scales[n].servo_id for n in names]
        dup_ids = {i for i in servo_ids if servo_ids.count(i) > 1}
        if dup_ids:
            raise ValueError(
                "duplicate servo ids in profile: "
                + ", ".join(str(i) for i in sorted(dup_ids))
            )

        for name in names:
            scale = self.scales[name]
            if scale.joint_name != name:
                raise ValueError(
                    f"servo scale for '{name}' declares joint_name "
                    f"'{scale.joint_name}'"
                )

    # ------------------------------------------------------------------
    # Lookups
    # ------------------------------------------------------------------
    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    def joint(self, name: str) -> JointSpec:
        for spec in self.joints:
            if spec.name == name:
                return spec
        raise UnknownJointError(
            f"unknown joint '{name}'; known joints: {', '.join(self.joint_names)}"
        )

    def scale(self, name: str) -> ServoScale:
        try:
            return self.scales[name]
        except KeyError as exc:
            raise UnknownJointError(f"unknown joint '{name}'") from exc

    def joint_by_servo_id(self, servo_id: int) -> JointSpec:
        for spec in self.joints:
            if self.scales[spec.name].servo_id == servo_id:
                return spec
        raise UnknownJointError(f"no joint mapped to servo id {servo_id}")

    def groups(self) -> List[str]:
        """Controller groups, in first-appearance order."""
        seen: List[str] = []
        for spec in self.joints:
            if spec.group not in seen:
                seen.append(spec.group)
        return seen

    def joints_in_group(self, group: str) -> List[JointSpec]:
        return [j for j in self.joints if j.group == group]

    def gripper_joints(self) -> List[JointSpec]:
        return [j for j in self.joints if j.kind is JointKind.GRIPPER]

    # ------------------------------------------------------------------
    # Pose validation
    # ------------------------------------------------------------------
    def validate_pose(self, pose: Pose, *, require_complete: bool = True) -> Pose:
        """Raise if any joint is unknown or out of range.

        Args:
            pose: The pose to check.
            require_complete: When true, every joint of the profile must be
                present — a partial pose would leave joints unspecified in
                the trajectory.
        """
        for name in pose.joint_names():
            spec = self.joint(name)
            spec.validate(pose[name])
            # A gripper that only opens and closes must not be asked to sit
            # half-way, however that value reached us (service, foreign file).
            if not spec.is_at_detent(pose[name]):
                raise UnsupportedJointMotionError(
                    f"joint '{name}' only opens and closes; {pose[name]:.4f} is "
                    f"neither open ({spec.open_position}) nor closed "
                    f"({spec.closed_position})"
                )

        if require_complete:
            missing = [n for n in self.joint_names if n not in pose]
            if missing:
                raise IncompletePoseError(
                    "pose is missing joints: " + ", ".join(missing)
                )
        return pose

    def clamp_pose(self, pose: Pose) -> Pose:
        """Squeeze every joint into something it can hold; drop unknown joints.

        Revolute joints are clamped to their limits; grippers are snapped to
        whichever detent (open / closed) is nearer.
        """
        clamped: Dict[str, float] = {}
        for name in pose.joint_names():
            try:
                spec = self.joint(name)
            except UnknownJointError:
                continue
            clamped[name] = spec.snap(pose[name])
        return Pose(clamped)

    def snap_grippers(self, pose: Pose) -> Pose:
        """Force every gripper joint onto its nearest detent."""
        snapped = pose.as_dict()
        for spec in self.gripper_joints():
            if spec.name in snapped:
                snapped[spec.name] = spec.nearest_detent(snapped[spec.name])
        return Pose(snapped)

    def clamped_joints(self, pose: Pose) -> List[str]:
        """Names of joints that :meth:`clamp_pose` would modify."""
        out = []
        for name in pose.joint_names():
            try:
                spec = self.joint(name)
            except UnknownJointError:
                continue
            if spec.is_clamped(pose[name]):
                out.append(name)
        return out

    def home_pose(self) -> Pose:
        """Every joint at its centre pulse — a safe fallback pose.

        Grippers have no meaningful centre, so they land on their nearest
        detent instead (the centre pulse is not a position they can hold).
        """
        positions: Dict[str, float] = {}
        for spec in self.joints:
            scale = self.scale(spec.name)
            centre = (scale.min_pulse + scale.max_pulse) / 2.0
            positions[spec.name] = spec.snap(scale.to_radians(centre))
        return Pose(positions)

    # ------------------------------------------------------------------
    # Pulse <-> radian conversion for whole poses
    # ------------------------------------------------------------------
    def pose_from_pulses(self, pulses: Mapping[str, float]) -> Pose:
        """Build a validated, clamped pose from editor slider values."""
        radians: Dict[str, float] = {}
        for name, pulse in pulses.items():
            spec = self.joint(name)
            scale = self.scale(name)
            # snap() clamps a revolute joint and detents a gripper, so a raw
            # pulse out of a foreign file can never yield an illegal pose.
            radians[name] = spec.snap(scale.to_radians(scale.clamp_pulse(int(pulse))))
        return Pose(radians)

    def pulses_from_pose(self, pose: Pose) -> Dict[str, int]:
        """Convert a pose back into slider values for display."""
        return {
            name: self.scale(name).to_pulse(pose[name])
            for name in pose.joint_names()
            if name in self.scales
        }

    # ------------------------------------------------------------------
    # Single-joint editing, honouring each joint's capability
    # ------------------------------------------------------------------
    def jog(self, pose: Pose, joint_name: str, delta: float) -> Tuple[Pose, bool]:
        """Nudge one revolute joint. Returns the new pose and whether it clamped.

        Raises:
            UnsupportedJointMotionError: if the joint is a gripper.
        """
        spec = self.joint(joint_name)
        current = pose[joint_name] if joint_name in pose else 0.0
        target = spec.jog(current, delta)
        clamped = spec.is_clamped(current + delta)
        return pose.with_joint(joint_name, target), clamped

    def set_joint(
        self, pose: Pose, joint_name: str, position: float
    ) -> Tuple[Pose, bool]:
        """Set one joint absolutely, clamped (revolute) or detented (gripper)."""
        spec = self.joint(joint_name)
        clamped = spec.is_clamped(position)
        return pose.with_joint(joint_name, spec.snap(position)), clamped

    def set_gripper(
        self, pose: Pose, joint_name: str, command: GripperCommand
    ) -> Pose:
        """Open or close a gripper joint.

        Raises:
            UnsupportedJointMotionError: if the joint is not a gripper.
        """
        spec = self.joint(joint_name)
        return pose.with_joint(joint_name, spec.gripper_position(command))

    def fill_missing(self, pose: Pose, fallback: Optional[Pose] = None) -> Pose:
        """Complete a partial pose from ``fallback`` (default: centre pose)."""
        base = fallback if fallback is not None else self.home_pose()
        merged = base.as_dict()
        merged.update(pose.as_dict())
        return Pose({n: merged[n] for n in self.joint_names if n in merged})


def build_profile(
    joints: Iterable[JointSpec], scales: Iterable[ServoScale]
) -> RobotProfile:
    """Convenience constructor keyed on joint name."""
    return RobotProfile(
        joints=tuple(joints),
        scales={s.joint_name: s for s in scales},
    )
