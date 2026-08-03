"""ROS adapter for blocking holonomic base motion.

Like the existing ``JointTrajectoryActionClient``, ``drive_to`` blocks and
must run in a worker thread while a ``MultiThreadedExecutor`` spins the owning
node. Never call it from a callback served by a single-threaded executor.
"""

import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from ...application.base_control import (
    BaseGains,
    Pose2D,
    body_velocity_command,
    yaw_from_quaternion,
)


class BaseDriver:
    """Drive a mecanum base to an odometry-frame planar goal."""

    _ODOM_WAIT_S = 2.0

    def __init__(
        self,
        node,
        default_callback_group,
        cmd_vel_topic: str = "/cmd_vel",
        odom_topic: str = "/odom",
        gains: BaseGains = BaseGains(),
        control_period_s: float = 0.05,
    ) -> None:
        if (
            not math.isfinite(control_period_s)
            or control_period_s <= 0.0
        ):
            raise ValueError("control_period_s must be finite and positive")

        self._node = node
        self._gains = gains
        self._control_period_s = float(control_period_s)
        self._pose_lock = threading.Lock()
        self._latest_pose = None

        self._cmd_vel_publisher = node.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self._odom_subscription = node.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
            callback_group=default_callback_group,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        with self._pose_lock:
            self._latest_pose = pose

    def _pose_snapshot(self):
        with self._pose_lock:
            return self._latest_pose

    def current_pose(self) -> Pose2D | None:
        """Return the latest odometry pose, or ``None`` before first odom."""
        return self._pose_snapshot()

    def _publish_velocity(
        self,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        command.angular.z = wz
        self._cmd_vel_publisher.publish(command)

    def drive_to(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
        timeout_s: float = 20.0,
    ) -> bool:
        """Block in a worker thread until the goal is reached or times out."""
        try:
            if not math.isfinite(timeout_s) or timeout_s < 0.0:
                raise ValueError("timeout_s must be finite and non-negative")

            goal = Pose2D(
                x=float(goal_x),
                y=float(goal_y),
                yaw=float(goal_yaw),
            )
            started = time.monotonic()
            deadline = started + float(timeout_s)
            odom_deadline = min(deadline, started + self._ODOM_WAIT_S)

            current = self._pose_snapshot()
            while current is None and time.monotonic() < odom_deadline:
                remaining = odom_deadline - time.monotonic()
                time.sleep(min(self._control_period_s, max(0.0, remaining)))
                current = self._pose_snapshot()

            if current is None:
                self._node.get_logger().warning(
                    "no odometry received before base drive deadline"
                )
                return False

            while time.monotonic() < deadline:
                current = self._pose_snapshot()
                if current is None:
                    self._node.get_logger().warning(
                        "odometry was lost during base drive"
                    )
                    return False
                vx, vy, wz, reached = body_velocity_command(
                    current,
                    goal,
                    self._gains,
                )
                self._publish_velocity(vx, vy, wz)
                if reached:
                    return True

                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(self._control_period_s, remaining))
            final = self._pose_snapshot()
            if final is not None:
                err = math.hypot(goal.x - final.x, goal.y - final.y)
                self._node.get_logger().warning(
                    f"drive_to timed out: pos_err={err:.3f}m at "
                    f"({final.x:.3f},{final.y:.3f},{final.yaw:.3f}) "
                    f"goal=({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f})"
                )
            return False
        finally:
            self.stop()

    def stop(self) -> None:
        """Publish a zero velocity command."""
        self._publish_velocity(0.0, 0.0, 0.0)


__all__ = ["BaseDriver"]
