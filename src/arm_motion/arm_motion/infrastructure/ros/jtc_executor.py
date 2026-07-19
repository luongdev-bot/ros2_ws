"""Adapter: play a Motion through JointTrajectoryController action servers.

A motion is dispatched as **one trajectory per controller group** (arm and
gripper), all sharing the same time base, so the groups stay synchronised.
The controller interpolates between waypoints; this adapter only waits and
reports progress.

Executions are serialised: at most one motion is in flight at a time, and each
gets its own cancel token, so a cancel can never stop somebody else's motion.
Whenever an execution ends for any reason other than success, every goal it
accepted is cancelled — the arm must not keep moving after we report failure.
"""

import threading
import time
from typing import Dict, List, Optional

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ...domain.errors import MotionCancelledError, MotionExecutionError
from ...domain.motion import Motion
from ...domain.ports import CancelCheck, ProgressCallback, TrajectoryExecutor
from ...domain.robot_profile import RobotProfile

# How often we re-check futures and the cancel flag while waiting.
POLL_INTERVAL_S = 0.02
# Grace period added to the motion duration before we declare a timeout.
RESULT_GRACE_S = 5.0
# How long execute() waits for a previous execution to finish before giving up.
DEFAULT_ACQUIRE_TIMEOUT_S = 10.0

# rclpy GoalStatus codes.
_STATUS_SUCCEEDED = 4
_STATUS_CANCELED = 6


class _Execution:
    """Per-call state: its own cancel token and its own accepted goals.

    Goals are registered via a *future* callback rather than after the future
    resolves. A cancel that lands while a goal request is still in flight would
    otherwise find nothing to cancel, and the controller would happily start
    executing once the server got round to accepting it.
    """

    def __init__(self, owner=None) -> None:
        self.owner = owner
        self.cancel_event = threading.Event()
        self._handles: List[object] = []
        self._aborted = False
        self._lock = threading.Lock()

    def track_future(self, future) -> None:
        """Adopt a goal the moment the server accepts it."""
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception:  # noqa: BLE001 - the caller reports the failure
            return
        if handle is None or not getattr(handle, "accepted", False):
            return
        with self._lock:
            self._handles.append(handle)
            already_aborted = self._aborted
        if already_aborted:
            # We gave up before this acceptance arrived — stop it immediately.
            _try_cancel(handle)

    def handles(self) -> List[object]:
        with self._lock:
            return list(self._handles)

    def abort_goals(self) -> None:
        """Ask every accepted goal to stop. Safe to call more than once.

        Also latches an 'aborted' flag, so goals accepted *after* this point
        are cancelled as soon as they arrive.
        """
        with self._lock:
            self._aborted = True
            handles = list(self._handles)
        for handle in handles:
            _try_cancel(handle)


def _try_cancel(handle) -> None:
    try:
        handle.cancel_goal_async()
    except Exception:  # noqa: BLE001 - best effort, we are aborting anyway
        pass


class _Any:
    """Sentinel meaning 'cancel whoever is running'."""


ANY_OWNER = _Any()


class JtcTrajectoryExecutor(TrajectoryExecutor):
    """Sends FollowJointTrajectory goals to one action server per group."""

    def __init__(
        self,
        node: Node,
        profile: RobotProfile,
        controller_namespaces: Dict[str, str],
        *,
        callback_group=None,
        server_wait_timeout_s: float = 5.0,
    ):
        self._node = node
        self._profile = profile
        self._server_wait_timeout_s = server_wait_timeout_s

        # Serialises executions; `_current` is the one holding it.
        self._execution_lock = threading.Lock()
        self._current: Optional[_Execution] = None
        self._current_lock = threading.Lock()

        missing = [g for g in profile.groups() if g not in controller_namespaces]
        if missing:
            raise ValueError(
                "no controller configured for joint group(s): " + ", ".join(missing)
            )

        self._clients: Dict[str, ActionClient] = {
            group: ActionClient(
                node,
                FollowJointTrajectory,
                controller_namespaces[group],
                callback_group=callback_group,
            )
            for group in profile.groups()
        }
        self._namespaces = dict(controller_namespaces)

    # ------------------------------------------------------------------
    # TrajectoryExecutor
    # ------------------------------------------------------------------
    def cancel(self, owner=ANY_OWNER) -> bool:
        """Cancel the execution running right now. Returns whether it did.

        Args:
            owner: Cancel only if the running execution was started with this
                same owner tag. Without it, a caller whose motion is still
                *queued* would cancel whatever unrelated motion currently
                holds the arm.
        """
        with self._current_lock:
            execution = self._current
        if execution is None:
            return False
        if owner is not ANY_OWNER and execution.owner != owner:
            return False
        execution.cancel_event.set()
        execution.abort_goals()
        return True

    def is_busy(self) -> bool:
        with self._current_lock:
            return self._current is not None

    def execute(
        self,
        motion: Motion,
        *,
        on_progress: Optional[ProgressCallback] = None,
        should_cancel: Optional[CancelCheck] = None,
        preempt: bool = False,
        owner=None,
        acquire_timeout_s: float = DEFAULT_ACQUIRE_TIMEOUT_S,
    ) -> None:
        """Run ``motion`` to completion.

        Args:
            preempt: Cancel whatever is already running instead of queueing
                behind it. Used for live slider following, where only the
                newest pose matters.
            owner: Tag identifying the caller, so ``cancel(owner=...)`` can
                target this execution specifically.
            acquire_timeout_s: How long to wait for a previous execution.
        """
        motion.require_steps()

        if preempt:
            self.cancel()

        if not self._execution_lock.acquire(timeout=acquire_timeout_s):
            raise MotionExecutionError(
                "another motion is still running; try again in a moment"
            )

        execution = _Execution(owner=owner)
        with self._current_lock:
            self._current = execution
        try:
            self._run(motion, execution, on_progress, should_cancel)
        finally:
            with self._current_lock:
                if self._current is execution:
                    self._current = None
            self._execution_lock.release()

    # ------------------------------------------------------------------
    # One execution
    # ------------------------------------------------------------------
    def _run(
        self,
        motion: Motion,
        execution: _Execution,
        on_progress: Optional[ProgressCallback],
        should_cancel: Optional[CancelCheck],
    ) -> None:
        def cancelled() -> bool:
            if execution.cancel_event.is_set():
                return True
            return bool(should_cancel and should_cancel())

        trajectories = self._build_trajectories(motion)
        if not trajectories:
            raise MotionExecutionError(
                f"motion '{motion.name}' commands no known joints"
            )

        # A caller that was cancelled while queued behind another motion should
        # never touch the arm at all — bail out before any goal goes out.
        if cancelled():
            raise MotionCancelledError(
                f"motion '{motion.name}' cancelled before it started"
            )

        self._wait_for_servers(trajectories.keys())

        total_s = motion.total_duration_ms / 1000.0
        deadline = time.monotonic() + total_s + RESULT_GRACE_S

        succeeded = False
        try:
            goal_futures = {}
            for group, trajectory in trajectories.items():
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = trajectory
                future = self._clients[group].send_goal_async(goal)
                # Register before awaiting anything: a cancel arriving while
                # this request is still in flight must still reach the goal.
                execution.track_future(future)
                goal_futures[group] = future

            handles = {}
            for group, future in goal_futures.items():
                handle = self._await(future, deadline, cancelled, f"{group} goal")
                if handle is None or not handle.accepted:
                    raise MotionExecutionError(
                        f"{self._namespaces[group]} rejected the trajectory"
                    )
                handles[group] = handle

            result_futures = {
                group: handle.get_result_async() for group, handle in handles.items()
            }
            self._await_all(result_futures, motion, deadline, cancelled, on_progress)
            succeeded = True
        finally:
            if not succeeded:
                # Rejection, timeout, cancel or a controller error: never leave
                # an accepted goal executing behind us.
                execution.abort_goals()

    # ------------------------------------------------------------------
    # Trajectory construction
    # ------------------------------------------------------------------
    def _build_trajectories(self, motion: Motion) -> Dict[str, JointTrajectory]:
        cumulative_ms = motion.cumulative_times_ms()
        trajectories: Dict[str, JointTrajectory] = {}

        for group in self._profile.groups():
            joint_names = [
                spec.name
                for spec in self._profile.joints_in_group(group)
                if all(spec.name in step.pose for step in motion.steps)
            ]
            if not joint_names:
                continue

            trajectory = JointTrajectory()
            trajectory.joint_names = joint_names
            for step, elapsed_ms in zip(motion.steps, cumulative_ms):
                point = JointTrajectoryPoint()
                point.positions = [float(step.pose[name]) for name in joint_names]
                # Zero velocity at every waypoint keeps the arm settled on each
                # taught pose rather than blending through it.
                point.velocities = [0.0] * len(joint_names)
                point.time_from_start = _duration_from_ms(elapsed_ms)
                trajectory.points.append(point)
            trajectories[group] = trajectory
        return trajectories

    # ------------------------------------------------------------------
    # Waiting helpers
    # ------------------------------------------------------------------
    def _wait_for_servers(self, groups) -> None:
        for group in groups:
            client = self._clients[group]
            if not client.wait_for_server(timeout_sec=self._server_wait_timeout_s):
                raise MotionExecutionError(
                    f"action server {self._namespaces[group]} is not available; "
                    "is the controller spawned?"
                )

    def _await(self, future, deadline: float, cancelled: CancelCheck, what: str):
        """Block until ``future`` resolves, honouring cancel and deadline."""
        while not future.done():
            if cancelled():
                raise MotionCancelledError(f"cancelled while waiting for {what}")
            if time.monotonic() > deadline:
                raise MotionExecutionError(f"timed out waiting for {what}")
            time.sleep(POLL_INTERVAL_S)
        return future.result()

    def _await_all(
        self,
        result_futures: Dict[str, object],
        motion: Motion,
        deadline: float,
        cancelled: CancelCheck,
        on_progress: Optional[ProgressCallback],
    ) -> None:
        cumulative_ms = motion.cumulative_times_ms()
        step_count = len(motion)
        started = time.monotonic()
        reported = -1

        while True:
            if cancelled():
                raise MotionCancelledError(f"motion '{motion.name}' cancelled")
            if time.monotonic() > deadline:
                raise MotionExecutionError(
                    f"motion '{motion.name}' did not finish before its deadline"
                )

            if on_progress is not None:
                elapsed_ms = (time.monotonic() - started) * 1000.0
                index = _step_at(cumulative_ms, elapsed_ms)
                if index != reported:
                    reported = index
                    on_progress(index, step_count)

            if all(f.done() for f in result_futures.values()):
                break
            time.sleep(POLL_INTERVAL_S)

        for group, future in result_futures.items():
            self._check_result(group, future.result())

        if on_progress is not None and reported != step_count:
            on_progress(step_count, step_count)

    def _check_result(self, group: str, wrapped) -> None:
        status = getattr(wrapped, "status", None)
        if status == _STATUS_CANCELED:
            raise MotionCancelledError(f"{self._namespaces[group]} cancelled the goal")

        result = getattr(wrapped, "result", None)
        error_code = getattr(result, "error_code", None)
        if (
            error_code is not None
            and error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            detail = getattr(result, "error_string", "") or f"error_code={error_code}"
            raise MotionExecutionError(f"{self._namespaces[group]}: {detail}")

        if status is not None and status != _STATUS_SUCCEEDED:
            raise MotionExecutionError(
                f"{self._namespaces[group]} finished with status {status}"
            )


def _duration_from_ms(milliseconds: int) -> DurationMsg:
    duration = DurationMsg()
    duration.sec = int(milliseconds // 1000)
    duration.nanosec = int((milliseconds % 1000) * 1_000_000)
    return duration


def _step_at(cumulative_ms: List[int], elapsed_ms: float) -> int:
    """How many waypoints should have been passed by ``elapsed_ms``."""
    for index, boundary in enumerate(cumulative_ms):
        if elapsed_ms < boundary:
            return index
    return len(cumulative_ms)
