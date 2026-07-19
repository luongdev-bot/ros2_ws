"""Bridges the Qt editor to ROS without the GUI thread ever touching rclpy.

The editor owns its own node (rather than calling the server's services) so
slider drags reach Gazebo with no round-trip. rclpy spins on a background
thread; every blocking call the GUI makes is dispatched to a worker.
"""

import threading
from pathlib import Path
from typing import Callable, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ..application.motion_library import (
    DeleteMotionUseCase,
    ListMotionsUseCase,
    LoadMotionUseCase,
    SaveMotionUseCase,
)
from ..domain.errors import MotionCancelledError
from ..domain.motion import Motion, MotionStep
from ..domain.pose import Pose
from ..domain.ports import CancelCheck, ProgressCallback
from ..domain.robot_profile import RobotProfile
from ..infrastructure.config_loader import (
    build_controller_groups,
    build_robot_profile,
    load_yaml,
)
from ..infrastructure.d6a_repository import D6aMotionRepository
from ..infrastructure.ros.joint_state_listener import JointStateListener
from ..infrastructure.ros.jtc_executor import JtcTrajectoryExecutor

LIVE_MOVE_DURATION_MS = 300
#: Owner tag for "Run action", so Stop targets it and not a live slider move.
PLAY_OWNER = "editor-play"


class EditorRosBridge:
    """Owns the editor's ROS node and its background spin thread."""

    def __init__(self, robot_config: Path, library_dir: str):
        config = load_yaml(Path(robot_config))
        self.profile: RobotProfile = build_robot_profile(config)
        controllers = build_controller_groups(config)

        self._node = Node("arm_motion_editor")
        callback_group = ReentrantCallbackGroup()

        self.repository = D6aMotionRepository(library_dir, self.profile)
        self.joint_states = JointStateListener(
            self._node, self.profile, callback_group=callback_group
        )
        self.executor_adapter = JtcTrajectoryExecutor(
            self._node, self.profile, controllers, callback_group=callback_group
        )

        self.save_motion = SaveMotionUseCase(self.repository)
        self.load_motion = LoadMotionUseCase(self.repository)
        self.list_motions = ListMotionsUseCase(self.repository)
        self.delete_motion = DeleteMotionUseCase(self.repository)

        self._ros_executor = MultiThreadedExecutor()
        self._ros_executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, name="arm_motion_editor_spin", daemon=True
        )
        self._running = True
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            self._ros_executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races are not interesting
            pass

    @property
    def logger(self):
        return self._node.get_logger()

    # ------------------------------------------------------------------
    # Operations used by the GUI
    # ------------------------------------------------------------------
    def current_pose(self) -> Optional[Pose]:
        return self.joint_states.current_pose()

    def goto_pose(self, pose: Pose, duration_ms: int = LIVE_MOVE_DURATION_MS) -> None:
        """Send a single-waypoint move — used for live slider following.

        Preempts any move still in flight: while dragging a slider only the
        newest pose matters, and queueing them would lag behind the user.
        """
        complete = self.profile.validate_pose(self.profile.fill_missing(pose))
        motion = Motion(
            name="live", steps=(MotionStep(pose=complete, duration_ms=duration_ms),)
        )
        try:
            self.executor_adapter.execute(motion, preempt=True)
        except MotionCancelledError:
            # Superseded by a newer slider position — expected, not an error.
            pass

    def play(
        self,
        motion: Motion,
        on_progress: Optional[ProgressCallback] = None,
        should_cancel: Optional[CancelCheck] = None,
    ) -> None:
        self.executor_adapter.execute(
            motion,
            on_progress=on_progress,
            should_cancel=should_cancel,
            owner=PLAY_OWNER,
        )

    def stop(self) -> None:
        """Stop a running Run-action, but never a live slider move."""
        self.executor_adapter.cancel(owner=PLAY_OWNER)

    def motion_names(self) -> List[str]:
        return [m.name for m in self.list_motions.execute()]

    def library_dir(self) -> Path:
        return self.repository.directory

    def shutdown(self) -> None:
        if not self._running:
            return
        self._running = False
        try:
            self.executor_adapter.cancel()
        except Exception:  # noqa: BLE001
            pass
        self._ros_executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Worker(threading.Thread):
    """Runs one blocking bridge call off the GUI thread."""

    def __init__(
        self,
        target: Callable[[], None],
        on_error: Callable[[str], None],
        on_done: Callable[[], None],
    ):
        super().__init__(daemon=True)
        self._target = target
        self._on_error = on_error
        self._on_done = on_done

    def run(self) -> None:
        try:
            self._target()
        except Exception as exc:  # noqa: BLE001 - surfaced in the status bar
            self._on_error(str(exc))
        finally:
            self._on_done()
