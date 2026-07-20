"""The arm_motion_server node.

Exposes the motion library and the player to the rest of the system:

    action  ~/play_motion   (PlayMotion)   - run an action group by name or task
    service ~/save_motion   (SaveMotion)   - write / update a .d6a file
    service ~/load_motion   (LoadMotion)   - read one back
    service ~/list_motions  (ListMotions)
    service ~/delete_motion (DeleteMotion)
    service ~/jog_joint     (JogJoint)     - move one joint, limit-checked
"""

import threading
from pathlib import Path
from typing import List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from arm_motion_interfaces.action import PlayMotion
from arm_motion_interfaces.msg import JointPosition, MotionStep as MotionStepMsg
from arm_motion_interfaces.msg import MotionSummary
from arm_motion_interfaces.srv import (
    DeleteMotion,
    JogJoint,
    ListMotions,
    LoadMotion,
    SaveMotion,
)

from ...application.motion_library import (
    DeleteMotionUseCase,
    ListMotionsUseCase,
    LoadMotionUseCase,
    SaveMotionUseCase,
)
from ...application.play_motion import PlayMotionRequest, PlayMotionUseCase
from ...domain.errors import (
    ArmMotionError,
    MotionCancelledError,
    UnsupportedJointMotionError,
)
from ...domain.joint_spec import GripperCommand, JointKind
from ...domain.motion import Motion, MotionStep
from ...domain.pose import Pose
from ...domain.robot_profile import RobotProfile
from ..config_loader import (
    ConfigError,
    build_controller_groups,
    build_robot_profile,
    build_task_motions,
    load_yaml,
)
from ..d6a_repository import D6aMotionRepository
from .joint_state_listener import JointStateListener
from .jtc_executor import JtcTrajectoryExecutor

DEFAULT_LIBRARY_DIR = "~/ActionGroups"
DEFAULT_JOG_DURATION_MS = 500


class ArmMotionServer(Node):
    """Wires the domain/application layers to ROS."""

    def __init__(self) -> None:
        super().__init__("arm_motion_server")

        share = Path(get_package_share_directory("arm_motion"))
        self.declare_parameter(
            "robot_config", str(share / "config" / "jetrover_arm.yaml")
        )
        self.declare_parameter(
            "task_config", str(share / "config" / "task_motions.yaml")
        )
        self.declare_parameter("library_dir", DEFAULT_LIBRARY_DIR)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("server_wait_timeout", 5.0)
        # Action group the arm always passes through before the requested one.
        # "" disables it. Playing the prelude itself does not re-run it.
        self.declare_parameter("prelude_motion", "home")

        robot_config_path = self.get_parameter("robot_config").value
        task_config_path = self.get_parameter("task_config").value
        library_dir = self.get_parameter("library_dir").value

        try:
            robot_config = load_yaml(Path(robot_config_path))
            self._profile: RobotProfile = build_robot_profile(robot_config)
            controllers = build_controller_groups(robot_config)
            task_motions = build_task_motions(load_yaml(Path(task_config_path)))
        except ConfigError as exc:
            self.get_logger().fatal(f"configuration error: {exc}")
            raise

        self._repository = D6aMotionRepository(library_dir, self._profile)
        self.get_logger().info(
            f"action-group library: {self._repository.directory}"
        )

        # Reentrant so a long-running PlayMotion goal does not block the
        # services (in particular a cancel arriving mid-motion).
        self._action_group = ReentrantCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        self._joint_states = JointStateListener(
            self,
            self._profile,
            topic=self.get_parameter("joint_states_topic").value,
            callback_group=service_group,
        )
        self._executor_adapter = JtcTrajectoryExecutor(
            self,
            self._profile,
            controllers,
            callback_group=self._action_group,
            server_wait_timeout_s=float(
                self.get_parameter("server_wait_timeout").value
            ),
        )

        prelude = str(self.get_parameter("prelude_motion").value or "").strip()
        self._play = PlayMotionUseCase(
            repository=self._repository,
            executor=self._executor_adapter,
            profile=self._profile,
            task_motions=task_motions,
            prelude_motion=prelude,
        )
        if prelude:
            self.get_logger().info(
                f"every motion runs through '{prelude}' first"
            )
        self._save = SaveMotionUseCase(self._repository)
        self._load = LoadMotionUseCase(self._repository)
        self._list = ListMotionsUseCase(self._repository)
        self._delete = DeleteMotionUseCase(self._repository)

        # Only one PlayMotion goal may be in flight at a time; the executor
        # itself is owner-scoped, so cancels target the right execution.
        self._play_lock = threading.Lock()

        self._play_server = ActionServer(
            self,
            PlayMotion,
            "~/play_motion",
            execute_callback=self._on_play,
            goal_callback=self._on_play_goal,
            cancel_callback=self._on_play_cancel,
            callback_group=self._action_group,
        )
        self.create_service(
            SaveMotion, "~/save_motion", self._on_save, callback_group=service_group
        )
        self.create_service(
            LoadMotion, "~/load_motion", self._on_load, callback_group=service_group
        )
        self.create_service(
            ListMotions, "~/list_motions", self._on_list, callback_group=service_group
        )
        self.create_service(
            DeleteMotion,
            "~/delete_motion",
            self._on_delete,
            callback_group=service_group,
        )
        self.create_service(
            JogJoint, "~/jog_joint", self._on_jog, callback_group=service_group
        )

        self.get_logger().info(
            "arm_motion_server ready: "
            f"{len(self._profile.joints)} joints, "
            f"{len(task_motions)} task bindings"
        )

    # ------------------------------------------------------------------
    # PlayMotion action
    # ------------------------------------------------------------------
    def _on_play_goal(self, goal_request) -> GoalResponse:
        if bool(goal_request.motion_name) == bool(goal_request.task):
            self.get_logger().warn(
                "rejecting goal: set exactly one of motion_name / task"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_play_cancel(self, goal_handle) -> CancelResponse:
        # Cancel the executor only if this very goal is the one driving the
        # arm. Owner-scoped so a goal still queued behind a jog cannot abort
        # that jog; its own `is_cancel_requested` stops it when it starts.
        self._executor_adapter.cancel(owner=bytes(goal_handle.goal_id.uuid))
        return CancelResponse.ACCEPT

    def _on_play(self, goal_handle):
        request = goal_handle.request
        result = PlayMotion.Result()

        if not self._play_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "another motion is already running"
            return result

        try:
            play_request = PlayMotionRequest(
                motion_name=request.motion_name,
                task=request.task,
                duration_ms=int(request.duration_ms),
            )

            def on_progress(index: int, count: int) -> None:
                feedback = PlayMotion.Feedback()
                feedback.step_index = int(index)
                feedback.step_count = int(count)
                feedback.progress = float(index) / count if count else 0.0
                goal_handle.publish_feedback(feedback)

            outcome = self._play.execute(
                play_request,
                on_progress=on_progress,
                should_cancel=goal_handle.is_cancel_requested,
                owner=bytes(goal_handle.goal_id.uuid),
            )

            goal_handle.succeed()
            result.success = True
            result.steps_executed = outcome.steps_executed
            result.message = (
                f"played '{outcome.motion_name}' "
                f"({outcome.steps_executed} steps, "
                f"{outcome.total_duration_ms} ms)"
            )
            self.get_logger().info(result.message)

        except MotionCancelledError as exc:
            goal_handle.canceled()
            result.success = False
            result.message = str(exc)
            self.get_logger().warn(f"play cancelled: {exc}")
        except ArmMotionError as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self.get_logger().error(f"play failed: {exc}")
        except Exception as exc:  # noqa: BLE001 - never kill the executor thread
            goal_handle.abort()
            result.success = False
            result.message = f"unexpected error: {exc}"
            self.get_logger().error(result.message)
        finally:
            self._play_lock.release()

        return result

    # ------------------------------------------------------------------
    # Library services
    # ------------------------------------------------------------------
    def _on_save(self, request, response):
        try:
            motion = Motion(
                name=request.name,
                steps=tuple(self._step_from_msg(s) for s in request.steps),
                description=request.description,
            )
            stored = self._save.execute(motion, overwrite=request.overwrite)
            response.success = True
            response.message = (
                f"saved '{stored.name}' ({len(stored)} steps) to "
                f"{self._repository.path_for(stored.name)}"
            )
            self.get_logger().info(response.message)
        except ArmMotionError as exc:
            response.success = False
            response.message = str(exc)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"unexpected error: {exc}"
        return response

    def _on_load(self, request, response):
        try:
            motion = self._load.execute(request.name)
            response.success = True
            response.description = motion.description
            response.steps = [self._step_to_msg(s) for s in motion.steps]
            response.message = f"loaded '{motion.name}' ({len(motion)} steps)"
        except ArmMotionError as exc:
            response.success = False
            response.message = str(exc)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"unexpected error: {exc}"
        return response

    def _on_list(self, _request, response):
        summaries: List[MotionSummary] = []
        for motion in self._list.execute():
            summary = MotionSummary()
            summary.name = motion.name
            summary.description = motion.description
            summary.step_count = len(motion)
            summary.total_duration_ms = motion.total_duration_ms
            summary.updated_at = motion.updated_at or ""
            summaries.append(summary)
        response.motions = summaries
        return response

    def _on_delete(self, request, response):
        try:
            self._delete.execute(request.name)
            response.success = True
            response.message = f"deleted '{request.name}'"
            self.get_logger().info(response.message)
        except ArmMotionError as exc:
            response.success = False
            response.message = str(exc)
        return response

    # ------------------------------------------------------------------
    # Jog service
    # ------------------------------------------------------------------
    def _on_jog(self, request, response):
        try:
            spec = self._profile.joint(request.joint_name)
            base = self._joint_states.current_pose()
            if base is None:
                response.success = False
                response.message = (
                    "no /joint_states received yet; is the controller running?"
                )
                return response
            # Clamp the measured pose to limits before we build on it: a joint
            # reading (from a mis-calibrated servo, say) must never be forwarded
            # into the trajectory just because it is not the joint being jogged.
            base = self._profile.clamp_pose(self._profile.fill_missing(base))

            if spec.kind is JointKind.GRIPPER:
                pose, clamped = self._jog_gripper(base, request, spec)
            else:
                pose, clamped = self._jog_revolute(base, request, spec)

            if self._executor_adapter.is_busy():
                response.success = False
                response.message = (
                    "a motion is currently running; stop it before jogging"
                )
                return response

            duration_ms = int(request.duration_ms) or DEFAULT_JOG_DURATION_MS
            motion = Motion(
                name="jog",
                steps=(MotionStep(pose=pose, duration_ms=duration_ms),),
            )
            self._executor_adapter.execute(
                motion, owner="jog", acquire_timeout_s=1.0
            )

            response.success = True
            response.resulting_position = float(pose[spec.name])
            response.clamped = clamped
            response.message = (
                f"{spec.name} -> {spec.describe_position(pose[spec.name])}"
                + (" (clamped to limit)" if clamped else "")
            )
        except UnsupportedJointMotionError as exc:
            response.success = False
            response.message = str(exc)
        except ArmMotionError as exc:
            response.success = False
            response.message = str(exc)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"unexpected error: {exc}"
        return response

    def _jog_gripper(self, base: Pose, request, spec):
        command_raw = (request.gripper_command or "").strip().lower()
        if not command_raw:
            raise UnsupportedJointMotionError(
                f"joint '{spec.name}' is a gripper; set gripper_command to "
                "'open' or 'close'"
            )
        try:
            command = GripperCommand(command_raw)
        except ValueError as exc:
            raise UnsupportedJointMotionError(
                f"unknown gripper_command '{request.gripper_command}'; "
                "expected 'open' or 'close'"
            ) from exc
        return self._profile.set_gripper(base, spec.name, command), False

    def _jog_revolute(self, base: Pose, request, spec):
        if request.gripper_command:
            raise UnsupportedJointMotionError(
                f"joint '{spec.name}' is not a gripper; open/close does not apply"
            )
        if request.absolute:
            return self._profile.set_joint(base, spec.name, float(request.position))
        return self._profile.jog(base, spec.name, float(request.delta))

    # ------------------------------------------------------------------
    # Message conversion
    # ------------------------------------------------------------------
    def _step_from_msg(self, msg: MotionStepMsg) -> MotionStep:
        pose = Pose({p.name: float(p.position) for p in msg.positions})
        # A caller can only send a number for a gripper, so interpret it as
        # "whichever of open/closed you meant" rather than rejecting it.
        # Revolute joints are still validated strictly against their limits.
        pose = self._profile.snap_grippers(self._profile.fill_missing(pose))
        pose = self._profile.validate_pose(pose)
        return MotionStep(pose=pose, duration_ms=int(msg.duration_ms))

    @staticmethod
    def _step_to_msg(step: MotionStep) -> MotionStepMsg:
        msg = MotionStepMsg()
        msg.duration_ms = int(step.duration_ms)
        msg.positions = [
            JointPosition(name=name, position=float(step.pose[name]))
            for name in sorted(step.pose.joint_names())
        ]
        return msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = ArmMotionServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
