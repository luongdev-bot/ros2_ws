"""Adapter: /joint_states -> domain Pose."""

import threading
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from ...domain.pose import Pose
from ...domain.ports import JointStateSource
from ...domain.robot_profile import RobotProfile

# joint_state_broadcaster publishes best-effort sensor data.
JOINT_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class JointStateListener(JointStateSource):
    """Caches the latest pose of the profile's joints."""

    def __init__(
        self,
        node: Node,
        profile: RobotProfile,
        topic: str = "/joint_states",
        callback_group=None,
    ):
        self._node = node
        self._profile = profile
        self._lock = threading.Lock()
        self._pose: Optional[Pose] = None
        self._subscription = node.create_subscription(
            JointState,
            topic,
            self._on_joint_state,
            JOINT_STATE_QOS,
            callback_group=callback_group,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        wanted = set(self._profile.joint_names)
        positions = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if name in wanted
        }
        if not positions:
            return
        with self._lock:
            # Merge, so a controller publishing only its own joints still
            # yields a complete pose over successive messages.
            merged = self._pose.as_dict() if self._pose is not None else {}
            merged.update(positions)
            self._pose = Pose(merged)

    def current_pose(self) -> Optional[Pose]:
        with self._lock:
            return self._pose

    def has_all_joints(self) -> bool:
        pose = self.current_pose()
        if pose is None:
            return False
        return all(name in pose for name in self._profile.joint_names)
