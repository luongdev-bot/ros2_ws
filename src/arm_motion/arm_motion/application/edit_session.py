"""The editor's working buffer: the motion currently being authored.

Holds the slider pose plus the step list, and mediates every edit through
:class:`~arm_motion.domain.robot_profile.RobotProfile` so no out-of-range
value can ever enter a step.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from ..domain.errors import InvalidMotionError
from ..domain.joint_spec import GripperCommand, JointKind
from ..domain.motion import Motion, MotionStep, validate_motion_name
from ..domain.pose import Pose
from ..domain.robot_profile import RobotProfile

DEFAULT_STEP_DURATION_MS = 1000
UNTITLED = "untitled"


@dataclass
class EditSession:
    """Mutable editor state. Single-threaded — owned by the GUI thread."""

    profile: RobotProfile
    motion: Motion = None  # type: ignore[assignment]
    live_pose: Pose = None  # type: ignore[assignment]
    selected_index: int = -1
    step_duration_ms: int = DEFAULT_STEP_DURATION_MS
    dirty: bool = False
    _origin_name: Optional[str] = None

    def __post_init__(self) -> None:
        if self.motion is None:
            self.motion = Motion(name=UNTITLED, steps=())
        if self.live_pose is None:
            self.live_pose = self.profile.home_pose()

    # ------------------------------------------------------------------
    # Slider editing — capability- and limit-aware
    # ------------------------------------------------------------------
    def set_pulse(self, joint_name: str, pulse: int) -> int:
        """Set one joint from a slider value. Returns the applied pulse."""
        spec = self.profile.joint(joint_name)
        scale = self.profile.scale(joint_name)
        safe_pulse = scale.clamp_pulse(pulse)
        radians = spec.clamp(scale.to_radians(safe_pulse))
        self.live_pose = self.live_pose.with_joint(joint_name, radians)
        # Report the pulse that actually corresponds to the applied angle,
        # so a slider dragged past a joint limit snaps back to the limit.
        return scale.to_pulse(radians)

    def jog(self, joint_name: str, steps: int = 1) -> int:
        """Nudge a revolute joint by ``steps`` * its jog_step. Returns pulse."""
        spec = self.profile.joint(joint_name)
        delta = spec.jog_step * steps
        self.live_pose, _ = self.profile.jog(self.live_pose, joint_name, delta)
        return self.profile.scale(joint_name).to_pulse(self.live_pose[joint_name])

    def set_gripper(self, joint_name: str, command: GripperCommand) -> int:
        """Open/close a gripper joint. Returns the resulting pulse."""
        self.live_pose = self.profile.set_gripper(
            self.live_pose, joint_name, command
        )
        return self.profile.scale(joint_name).to_pulse(self.live_pose[joint_name])

    def toggle_gripper(self, joint_name: str) -> Tuple[GripperCommand, int]:
        """Flip a gripper between open and closed."""
        spec = self.profile.joint(joint_name)
        if spec.kind is not JointKind.GRIPPER:
            raise InvalidMotionError(f"joint '{joint_name}' is not a gripper")
        current = self.live_pose[joint_name] if joint_name in self.live_pose else 0.0
        open_pos = float(spec.open_position)
        closed_pos = float(spec.closed_position)
        is_open = abs(current - open_pos) <= abs(current - closed_pos)
        command = GripperCommand.CLOSE if is_open else GripperCommand.OPEN
        return command, self.set_gripper(joint_name, command)

    def live_pulses(self) -> Dict[str, int]:
        """Slider values for the current pose."""
        return self.profile.pulses_from_pose(self.live_pose)

    def adopt_pose(self, pose: Pose) -> None:
        """Take a measured pose (the 'read angle' button) into the sliders."""
        self.live_pose = self.profile.clamp_pose(
            self.profile.fill_missing(pose, self.live_pose)
        )

    # ------------------------------------------------------------------
    # Step list editing
    # ------------------------------------------------------------------
    def _capture(self) -> MotionStep:
        pose = self.profile.validate_pose(
            self.profile.fill_missing(self.live_pose)
        )
        return MotionStep(pose=pose, duration_ms=self.step_duration_ms)

    def add_step(self) -> int:
        """Append the current pose as a new step. Returns its index."""
        self.motion = self.motion.with_step_appended(self._capture())
        self.selected_index = len(self.motion) - 1
        self.dirty = True
        return self.selected_index

    def insert_step(self, index: Optional[int] = None) -> int:
        """Insert the current pose above ``index`` (default: selection)."""
        at = self.selected_index if index is None else index
        if at < 0:
            at = len(self.motion)
        self.motion = self.motion.with_step_inserted(at, self._capture())
        self.selected_index = min(at, len(self.motion) - 1)
        self.dirty = True
        return self.selected_index

    def update_step(self, index: Optional[int] = None) -> int:
        """Overwrite a step with the current pose and duration."""
        at = self._require_selection(index)
        self.motion = self.motion.with_step_replaced(at, self._capture())
        self.dirty = True
        return at

    def delete_step(self, index: Optional[int] = None) -> None:
        at = self._require_selection(index)
        self.motion = self.motion.with_step_removed(at)
        self.selected_index = min(at, len(self.motion) - 1)
        self.dirty = True

    def delete_all(self) -> None:
        self.motion = self.motion._replacing(())
        self.selected_index = -1
        self.dirty = True

    def move_step(self, offset: int, index: Optional[int] = None) -> int:
        at = self._require_selection(index)
        self.motion, self.selected_index = self.motion.with_step_moved(at, offset)
        self.dirty = True
        return self.selected_index

    def select(self, index: int) -> None:
        """Select a step and load its pose/duration into the sliders."""
        if not 0 <= index < len(self.motion):
            self.selected_index = -1
            return
        self.selected_index = index
        step = self.motion.steps[index]
        self.live_pose = step.pose
        self.step_duration_ms = step.duration_ms

    def set_step_duration(self, duration_ms: int) -> None:
        """Set the duration used for newly captured steps."""
        # Validated by MotionStep on capture; keep the raw value here so the
        # spin box can pass through intermediate values while typing.
        self.step_duration_ms = int(duration_ms)

    def _require_selection(self, index: Optional[int]) -> int:
        at = self.selected_index if index is None else index
        if not 0 <= at < len(self.motion):
            raise InvalidMotionError("no step selected")
        return at

    # ------------------------------------------------------------------
    # File-level operations
    # ------------------------------------------------------------------
    def rename(self, name: str) -> None:
        validate_motion_name(name)
        self.motion = self.motion.renamed(name)
        self.dirty = True

    def set_description(self, description: str) -> None:
        self.motion = Motion(
            self.motion.name, self.motion.steps, description, self.motion.updated_at
        )
        self.dirty = True

    def open(self, motion: Motion) -> None:
        """Replace the buffer with a motion loaded from disk."""
        self.motion = motion
        self._origin_name = motion.name
        self.selected_index = 0 if len(motion) else -1
        if len(motion):
            self.live_pose = motion.steps[0].pose
            self.step_duration_ms = motion.steps[0].duration_ms
        self.dirty = False

    def mark_saved(self, motion: Motion) -> None:
        self.motion = motion
        self._origin_name = motion.name
        self.dirty = False

    def is_new_file(self) -> bool:
        """True when saving would create a file rather than update one."""
        return self._origin_name is None or self._origin_name != self.motion.name

    def new(self) -> None:
        self.motion = Motion(name=UNTITLED, steps=())
        self._origin_name = None
        self.selected_index = -1
        self.step_duration_ms = DEFAULT_STEP_DURATION_MS
        self.dirty = False

    # ------------------------------------------------------------------
    # Table rendering
    # ------------------------------------------------------------------
    def table_rows(self) -> List[List[str]]:
        """Rows for the editor table: Index, Time, then one column per servo."""
        rows: List[List[str]] = []
        for i, step in enumerate(self.motion.steps):
            pulses = self.profile.pulses_from_pose(step.pose)
            row = [str(i + 1), str(step.duration_ms)]
            row.extend(str(pulses.get(spec.name, "")) for spec in self.profile.joints)
            rows.append(row)
        return rows

    def column_headers(self) -> List[str]:
        headers = ["Index", "Time"]
        headers.extend(
            f"ID:{self.profile.scale(spec.name).servo_id}"
            for spec in self.profile.joints
        )
        return headers
