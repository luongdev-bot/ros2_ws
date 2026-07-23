"""Pure grasp state and rigid-transform logic for the Gazebo attacher."""

from dataclasses import dataclass
from enum import Enum
from math import hypot, isfinite, sqrt
from typing import Mapping, Optional, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
_IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)
_NANOSECONDS_PER_SECOND = 1_000_000_000
_MAX_TIMER_PERIOD_NANOSECONDS = (1 << 63) - 1


def timer_period_from_rate(update_rate: float) -> float:
    """Return an rclpy-representable timer period for an update rate."""

    if not isfinite(update_rate) or update_rate <= 0.0:
        raise ValueError('update_rate must be finite and positive')

    timer_period = 1.0 / update_rate
    maximum_period = (
        _MAX_TIMER_PERIOD_NANOSECONDS / _NANOSECONDS_PER_SECOND)
    if not isfinite(timer_period) or timer_period > maximum_period:
        raise ValueError(
            'update_rate produces an unrepresentable timer period')

    timer_period_nanoseconds = int(
        timer_period * _NANOSECONDS_PER_SECOND)
    if not 0 < timer_period_nanoseconds <= _MAX_TIMER_PERIOD_NANOSECONDS:
        raise ValueError(
            'update_rate produces an unrepresentable timer period')
    return timer_period


class TimeRollbackDetector:
    """Track a clock and report when its value moves backwards."""

    def __init__(self) -> None:
        self._last_time: Optional[int] = None

    def observe(self, time_nanoseconds: int) -> bool:
        """Record a clock sample and return whether it rolled backwards."""

        rolled_back = (
            self._last_time is not None
            and time_nanoseconds < self._last_time
        )
        self._last_time = time_nanoseconds
        return rolled_back


@dataclass(frozen=True)
class Pose:
    """ROS-free rigid pose using an ``(x, y, z, w)`` quaternion."""

    position: Vector3
    orientation: Quaternion = _IDENTITY_QUATERNION

    def distance_to(self, other: 'Pose') -> float:
        dx = self.position[0] - other.position[0]
        dy = self.position[1] - other.position[1]
        dz = self.position[2] - other.position[2]
        return sqrt(dx * dx + dy * dy + dz * dz)


def is_valid_pose(pose: Pose) -> bool:
    """Return whether a pose is finite and has a usable orientation."""

    if len(pose.position) != 3 or len(pose.orientation) != 4:
        return False
    if not all(isfinite(value) for value in pose.position + pose.orientation):
        return False
    orientation_norm = hypot(*pose.orientation)
    return isfinite(orientation_norm) and orientation_norm > 1e-12


class GraspAction(Enum):
    """Action requested by one grasp-controller update."""

    IDLE = 'idle'
    GRASP = 'grasp'
    HOLD = 'hold'
    RELEASE = 'release'


@dataclass(frozen=True)
class GraspDecision:
    """Result of evaluating the current gripper and block state."""

    action: GraspAction
    block_name: Optional[str] = None
    target_pose: Optional[Pose] = None


@dataclass(frozen=True)
class _HeldBlock:
    name: str
    pose_in_gripper: Pose


def _quaternion_multiply(first: Quaternion, second: Quaternion) -> Quaternion:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _normalized(quaternion: Quaternion) -> Quaternion:
    norm = hypot(*quaternion)
    if not isfinite(norm):
        raise ValueError('A pose quaternion must be finite')
    if norm <= 1e-12:
        raise ValueError('A pose quaternion cannot have zero length')
    normalized = tuple(component / norm for component in quaternion)
    return normalized  # type: ignore[return-value]


def _rotate(vector: Vector3, quaternion: Quaternion) -> Vector3:
    rotation = _normalized(quaternion)
    inverse = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
    vector_quaternion: Quaternion = (vector[0], vector[1], vector[2], 0.0)
    result = _quaternion_multiply(
        _quaternion_multiply(rotation, vector_quaternion), inverse)
    return result[0], result[1], result[2]


def compose_poses(parent: Pose, child: Pose) -> Pose:
    """Return ``child`` expressed in the coordinate frame of ``parent``."""

    rotated_position = _rotate(child.position, parent.orientation)
    position = (
        parent.position[0] + rotated_position[0],
        parent.position[1] + rotated_position[1],
        parent.position[2] + rotated_position[2],
    )
    orientation = _normalized(
        _quaternion_multiply(parent.orientation, child.orientation))
    return Pose(position=position, orientation=orientation)


def relative_pose(parent: Pose, child: Pose) -> Pose:
    """Return the transform from ``parent`` to ``child``."""

    parent_orientation = _normalized(parent.orientation)
    inverse = (
        -parent_orientation[0],
        -parent_orientation[1],
        -parent_orientation[2],
        parent_orientation[3],
    )
    displacement = (
        child.position[0] - parent.position[0],
        child.position[1] - parent.position[1],
        child.position[2] - parent.position[2],
    )
    return Pose(
        position=_rotate(displacement, inverse),
        orientation=_normalized(
            _quaternion_multiply(inverse, child.orientation)),
    )


class GraspController:
    """Decide when to grasp, hold, and release one nearby block."""

    def __init__(
        self,
        *,
        closed_position: float,
        open_position: float,
        closed_tolerance: float,
        open_tolerance: float,
        grasp_radius: float,
    ) -> None:
        values = (
            closed_position,
            open_position,
            closed_tolerance,
            open_tolerance,
            grasp_radius,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError('Grasp-controller parameters must be finite')
        if closed_position == open_position:
            raise ValueError('Open and closed positions must differ')
        if closed_tolerance < 0.0:
            raise ValueError('Closed-position tolerance cannot be negative')
        if open_tolerance < 0.0:
            raise ValueError('Open-position tolerance cannot be negative')
        if grasp_radius <= 0.0:
            raise ValueError('Grasp radius must be positive')

        self._closed_position = closed_position
        self._open_position = open_position
        self._closed_tolerance = closed_tolerance
        self._open_tolerance = open_tolerance
        self._grasp_radius = grasp_radius
        self._held_block: Optional[_HeldBlock] = None
        self._grasping_armed = True

    @property
    def held_block_name(self) -> Optional[str]:
        """Name of the currently held block, if any."""

        return None if self._held_block is None else self._held_block.name

    def is_closed(self, joint_position: float) -> bool:
        """Return whether the actuated jaw is at its closed endpoint."""

        if not isfinite(joint_position):
            return False
        closed_distance = abs(joint_position - self._closed_position)
        open_distance = abs(joint_position - self._open_position)
        return (
            closed_distance <= self._closed_tolerance
            and closed_distance < open_distance
        )

    def is_open(self, joint_position: float) -> bool:
        """Return whether the actuated jaw is at its open endpoint."""

        if not isfinite(joint_position):
            return False
        open_distance = abs(joint_position - self._open_position)
        closed_distance = abs(joint_position - self._closed_position)
        return (
            open_distance <= self._open_tolerance
            and open_distance < closed_distance
        )

    def observe_joint_position(self, joint_position: float) -> None:
        """Re-arm reset grasping after observing the jaw fully open."""

        if not self._grasping_armed and self.is_open(joint_position):
            self._grasping_armed = True

    def clear_held_state(self) -> None:
        """Forget held state and disarm grasping after a simulation reset."""

        self._held_block = None
        self._grasping_armed = False

    def update(
        self,
        joint_position: float,
        gripper_pose: Pose,
        block_poses: Mapping[str, Pose],
    ) -> GraspDecision:
        """Evaluate one sample and update the single-block grasp state."""

        self.observe_joint_position(joint_position)

        if self._held_block is not None:
            held_block = self._held_block
            if not self.is_closed(joint_position):
                self._held_block = None
                return GraspDecision(
                    action=GraspAction.RELEASE,
                    block_name=held_block.name,
                )

            if not is_valid_pose(gripper_pose):
                return GraspDecision(
                    action=GraspAction.HOLD,
                    block_name=held_block.name,
                )

            return GraspDecision(
                action=GraspAction.HOLD,
                block_name=held_block.name,
                target_pose=compose_poses(
                    gripper_pose, held_block.pose_in_gripper),
            )

        if not self._grasping_armed or not self.is_closed(joint_position):
            return GraspDecision(action=GraspAction.IDLE)
        if not is_valid_pose(gripper_pose):
            return GraspDecision(action=GraspAction.IDLE)

        nearby_blocks = (
            (gripper_pose.distance_to(pose), name, pose)
            for name, pose in block_poses.items()
            if is_valid_pose(pose)
        )
        candidates = [
            candidate
            for candidate in nearby_blocks
            if candidate[0] <= self._grasp_radius
        ]
        if not candidates:
            return GraspDecision(action=GraspAction.IDLE)

        _, block_name, block_pose = min(
            candidates, key=lambda candidate: (candidate[0], candidate[1]))
        self._held_block = _HeldBlock(
            name=block_name,
            pose_in_gripper=relative_pose(gripper_pose, block_pose),
        )
        return GraspDecision(
            action=GraspAction.GRASP,
            block_name=block_name,
            target_pose=block_pose,
        )
