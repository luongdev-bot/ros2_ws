"""Blocking worker-thread adapter for a joint trajectory action."""

import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryActionClient:
    """Send single-point trajectories without spinning the owning node."""

    _SERVER_WAIT_S = 1.0
    _CANCEL_WAIT_S = 2.0

    def __init__(
        self,
        node,
        action_name,
        joint_names: list[str],
        default_callback_group,
        default_timeout_s: float = 60.0,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._joint_names = list(joint_names)
        self._default_timeout_s = float(default_timeout_s)
        self._client = ActionClient(
            node,
            FollowJointTrajectory,
            action_name,
            callback_group=default_callback_group,
        )

    @staticmethod
    def _wait_for_future(future, deadline: float) -> bool:
        """Wait for an executor-owned future without spinning its node."""
        if future.done():
            return True

        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        remaining_s = deadline - time.monotonic()
        return remaining_s > 0.0 and done.wait(remaining_s)

    def _cancel_goal(self, goal_handle) -> None:
        """Best-effort cancellation with a short wall-clock wait."""
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            self._node.get_logger().warning(
                f"failed to request cancellation for {self._action_name!r}: "
                f"{exc}"
            )
            return

        deadline = time.monotonic() + self._CANCEL_WAIT_S
        try:
            cancellation_confirmed = self._wait_for_future(
                cancel_future,
                deadline,
            )
        except Exception as exc:
            self._node.get_logger().warning(
                f"failed to wait for cancellation of "
                f"{self._action_name!r}: {exc}"
            )
            return
        if not cancellation_confirmed:
            self._node.get_logger().warning(
                f"timed out confirming cancellation for "
                f"{self._action_name!r}"
            )
            return

        try:
            cancel_response = cancel_future.result()
        except Exception as exc:
            self._node.get_logger().warning(
                f"failed to confirm cancellation for {self._action_name!r}: "
                f"{exc}"
            )
            return

        goals_canceling = getattr(cancel_response, "goals_canceling", None)
        if goals_canceling is not None and not goals_canceling:
            self._node.get_logger().warning(
                f"cancellation was not confirmed for {self._action_name!r}"
            )
        elif goals_canceling is None and cancel_response is None:
            self._node.get_logger().warning(
                f"cancellation was not confirmed for {self._action_name!r}"
            )

    def _cancel_goal_from_future(self, goal_future) -> None:
        """Cancel a goal that becomes accepted after a send timeout."""
        try:
            goal_handle = goal_future.result()
        except Exception:
            return
        if goal_handle is not None and goal_handle.accepted:
            self._cancel_goal(goal_handle)

    def move(
        self,
        positions: list[float],
        duration_s: float,
        timeout_s: float | None = None,
    ) -> bool:
        """Move the configured joints and report action success."""
        if not self._client.wait_for_server(timeout_sec=self._SERVER_WAIT_S):
            self._node.get_logger().warning(
                f"action server {self._action_name!r} is unavailable"
            )
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=float(duration_s)).to_msg()
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        if timeout_s is None:
            timeout_s = self._default_timeout_s
        # This remains a wall-clock budget; extremely low RTF may require
        # raising it further.
        deadline = time.monotonic() + float(timeout_s)
        goal_future = self._client.send_goal_async(goal)
        if not self._wait_for_future(goal_future, deadline):
            goal_future.add_done_callback(self._cancel_goal_from_future)
            self._node.get_logger().error(
                f"timed out sending goal to {self._action_name!r}"
            )
            return False

        try:
            goal_handle = goal_future.result()
        except Exception as exc:
            self._node.get_logger().error(
                f"failed to send goal to {self._action_name!r}: {exc}"
            )
            return False

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(
                f"goal was rejected by {self._action_name!r}"
            )
            return False

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            self._node.get_logger().error(
                f"failed to request result from {self._action_name!r}: {exc}"
            )
            self._cancel_goal(goal_handle)
            return False
        if not self._wait_for_future(result_future, deadline):
            self._node.get_logger().error(
                f"timed out waiting for {self._action_name!r}"
            )
            self._cancel_goal(goal_handle)
            return False

        try:
            goal_result = result_future.result()
            status = goal_result.status
            result = goal_result.result
        except Exception as exc:
            self._node.get_logger().error(
                f"failed to receive result from {self._action_name!r}: {exc}"
            )
            self._cancel_goal(goal_handle)
            return False

        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self._node.get_logger().error(
                (
                    f"trajectory on {self._action_name!r} failed with "
                    f"status {status}, error code {result.error_code}: "
                    f"{result.error_string}"
                )
            )
            self._cancel_goal(goal_handle)
            return False
        return True


__all__ = ["JointTrajectoryActionClient"]
