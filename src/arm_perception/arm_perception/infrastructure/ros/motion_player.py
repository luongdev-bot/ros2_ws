"""MotionPlayer backed by arm_motion's PlayMotion action server."""

import threading
from typing import Callable, Optional

from arm_motion_interfaces.action import PlayMotion
from rclpy.action import ActionClient
from rclpy.node import Node

from ...domain.errors import MotionPlaybackError
from ...domain.ports import MotionPlayer

#: Called with (motion_name, success).
FinishedCallback = Callable[[str, bool], None]


class RosMotionPlayer(MotionPlayer):
    """Sends PlayMotion goals and reports completion asynchronously.

    ``play()`` returns immediately; the caller learns the outcome through
    the callback registered with ``on_finished``. Nothing here blocks the
    executor, so the camera keeps streaming while the arm moves.
    """

    def __init__(
        self,
        node: Node,
        action_name: str = "/arm_motion_server/play_motion",
        *,
        server_wait_timeout: float = 5.0,
        callback_group=None,
    ) -> None:
        self._node = node
        self._action_name = action_name
        self._server_wait_timeout = server_wait_timeout
        self._client = ActionClient(
            node, PlayMotion, action_name, callback_group=callback_group
        )
        self._finished_callback: Optional[FinishedCallback] = None
        # _active_motion is written from the caller's thread (play) and from
        # the action result callback, which run in different callback groups.
        self._lock = threading.Lock()
        self._active_motion: Optional[str] = None
        self._goal_handle = None
        # Latches a cancel that arrives before the goal is accepted, so the
        # goal is aborted the moment the server accepts it rather than running
        # to completion uncancelled.
        self._cancel_requested = False

    def on_finished(self, callback: FinishedCallback) -> None:
        self._finished_callback = callback

    def is_busy(self) -> bool:
        with self._lock:
            return self._active_motion is not None

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        """Block until the arm_motion server is up. For startup checks only."""
        return self._client.wait_for_server(
            timeout_sec=timeout_sec
            if timeout_sec is not None
            else self._server_wait_timeout
        )

    def play(self, motion_name: str, *, duration_ms: int = 0) -> None:
        # Claim the slot atomically: the check and the set must not be
        # separable, or two callers could both find it free.
        with self._lock:
            if self._active_motion is not None:
                raise MotionPlaybackError(
                    f"cannot play {motion_name!r}: "
                    f"{self._active_motion!r} is still running"
                )
            self._active_motion = motion_name
            self._goal_handle = None
            self._cancel_requested = False

        # Every exit path from here on must release the slot, or a single
        # failure would wedge the player as permanently busy.
        #
        # Deliberately does NOT wait for the server: play() is called from the
        # camera callback, and blocking there would stall frame processing (and
        # the enable service) for the whole timeout while sensor data piles up.
        # A missing server is reported immediately; the use case starts its
        # cooldown and the next confirmed block retries.
        if not self._client.server_is_ready():
            with self._lock:
                self._active_motion = None
            raise MotionPlaybackError(
                f"action server {self._action_name!r} is not available yet"
            )

        goal = PlayMotion.Goal()
        goal.motion_name = motion_name
        goal.duration_ms = int(duration_ms)

        try:
            future = self._client.send_goal_async(goal)
        except Exception as exc:  # rclpy raises bare exceptions here
            with self._lock:
                self._active_motion = None
            raise MotionPlaybackError(f"failed to send goal: {exc}") from exc
        future.add_done_callback(self._handle_goal_response)

    def cancel(self) -> None:
        """Ask the arm to abort the running motion, if any.

        Safe to call before the goal is accepted: the request is latched and
        the goal is aborted as soon as it arrives (see _handle_goal_response).
        """
        with self._lock:
            self._cancel_requested = True
            handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()

    def _handle_goal_response(self, future) -> None:
        motion = self._active_motion or "<unknown>"
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"PlayMotion goal errored: {exc}")
            self._finish(motion, False)
            return

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warning(
                f"PlayMotion rejected motion {motion!r} - is it in the library?"
            )
            self._finish(motion, False)
            return

        with self._lock:
            self._goal_handle = goal_handle
            cancel_requested = self._cancel_requested
        if cancel_requested:
            # A cancel landed while the goal request was still in flight; abort
            # the goal now instead of letting it run to completion.
            goal_handle.cancel_goal_async()
        goal_handle.get_result_async().add_done_callback(self._handle_result)

    def _handle_result(self, future) -> None:
        motion = self._active_motion or "<unknown>"
        try:
            wrapped = future.result()
        except Exception as exc:
            self._node.get_logger().error(f"PlayMotion result errored: {exc}")
            self._finish(motion, False)
            return

        result = wrapped.result
        success = bool(result.success)
        if not success:
            self._node.get_logger().warning(
                f"motion {motion!r} failed: {result.message}"
            )
        self._finish(motion, success)

    def _finish(self, motion_name: str, success: bool) -> None:
        with self._lock:
            self._active_motion = None
            self._goal_handle = None
        if self._finished_callback is not None:
            self._finished_callback(motion_name, success)
