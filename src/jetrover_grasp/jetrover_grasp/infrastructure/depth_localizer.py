"""ROS adapter that localizes a depth pixel in the robot base frame."""

import numpy as np
import tf2_geometry_msgs  # noqa: F401  (register geometry message conversions)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from ..domain.camera_projection import (
    deproject_pixel,
    intrinsics_from_camera_info_k,
)
from ..domain.depth_sampling import sample_depth


class DepthLocalizer:
    """Cache aligned depth data and transform sampled pixels into a base frame.

    ``locate_pixel`` may block for up to ``tf_timeout_s`` while waiting for
    ``/tf``. It must run under a ``MultiThreadedExecutor`` (or with a
    separate TF spin); do not call it from the same single-threaded callback
    that must also service ``/tf``.
    """

    def __init__(
        self,
        node: Node,
        *,
        depth_topic: str = "/depth_cam/depth_image",
        camera_info_topic: str = "/depth_cam/camera_info",
        optical_frame: str = "depth_cam_frame",
        base_frame: str = "base_footprint",
        depth_window: int = 5,
        tf_timeout_s: float = 0.5,
        max_depth_age_s: float = 1.0,
        use_latest_tf: bool = False,
        callback_group=None,
    ) -> None:
        if (
            isinstance(depth_window, bool)
            or not isinstance(depth_window, (int, np.integer))
            or depth_window <= 0
            or depth_window % 2 == 0
        ):
            raise ValueError("depth_window must be a positive odd integer")
        if not np.isfinite(tf_timeout_s) or tf_timeout_s < 0:
            raise ValueError("tf_timeout_s must be finite and non-negative")
        try:
            max_depth_age_s = float(max_depth_age_s)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "max_depth_age_s must be finite and positive"
            ) from exc
        if not np.isfinite(max_depth_age_s) or max_depth_age_s <= 0:
            raise ValueError("max_depth_age_s must be finite and positive")
        if not isinstance(use_latest_tf, (bool, np.bool_)):
            raise ValueError("use_latest_tf must be a bool")

        self._node = node
        self._optical_frame = optical_frame
        self._base_frame = base_frame
        self._depth_window = int(depth_window)
        self._tf_timeout = Duration(seconds=float(tf_timeout_s))
        self._max_depth_age_s = max_depth_age_s
        self._use_latest_tf = bool(use_latest_tf)

        self._bridge = CvBridge()
        self._intrinsics = None
        self._depth_data = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._camera_info_subscription = node.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self._depth_subscription = node.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        try:
            intrinsics = intrinsics_from_camera_info_k(msg.k)
        except ValueError as exc:
            self._node.get_logger().warning(
                f"ignoring invalid camera intrinsics: {exc}",
                throttle_duration_sec=5.0,
            )
            return
        self._intrinsics = intrinsics

    def _depth_callback(self, msg: Image) -> None:
        encoding = msg.encoding.upper()
        if encoding not in ("16UC1", "32FC1"):
            self._node.get_logger().warning(
                f"ignoring unsupported depth encoding {msg.encoding!r}",
                throttle_duration_sec=5.0,
            )
            return

        try:
            converted = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
        except CvBridgeError as exc:
            self._node.get_logger().warning(
                f"could not convert depth image: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        depth_image = np.array(converted, dtype=np.float32, copy=True)
        if depth_image.ndim != 2:
            self._node.get_logger().warning(
                f"ignoring non-2D depth image with shape {depth_image.shape}",
                throttle_duration_sec=5.0,
            )
            return
        if encoding == "16UC1":
            depth_image *= np.float32(1e-3)

        self._depth_data = depth_image, msg.header.stamp

    def locate_pixel(self, u, v) -> np.ndarray | None:
        """Return pixel ``(u, v)`` in ``base_frame``, or ``None``."""
        intrinsics = self._intrinsics
        depth_data = self._depth_data
        if intrinsics is None or depth_data is None:
            return None

        depth_image, depth_stamp = depth_data
        if self._depth_is_stale(depth_stamp):
            return None

        depth = sample_depth(depth_image, u, v, window=self._depth_window)
        if depth is None:
            return None

        fx, fy, cx, cy = intrinsics
        try:
            camera_point = deproject_pixel(u, v, depth, fx, fy, cx, cy)
        except ValueError:
            return None

        point = PointStamped()
        if self._use_latest_tf:
            point.header.stamp = Time().to_msg()
        else:
            point.header.stamp = depth_stamp
        point.header.frame_id = self._optical_frame
        point.point.x = float(camera_point[0])
        point.point.y = float(camera_point[1])
        point.point.z = float(camera_point[2])

        try:
            transformed = self._tf_buffer.transform(
                point,
                self._base_frame,
                timeout=self._tf_timeout,
            )
        except TransformException as exc:
            self._node.get_logger().warning(
                (
                    f"could not transform localized point from "
                    f"{self._optical_frame!r} to {self._base_frame!r}: {exc}"
                ),
                throttle_duration_sec=5.0,
            )
            return None

        return np.asarray(
            (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ),
            dtype=float,
        )

    def _depth_is_stale(self, depth_stamp) -> bool:
        """Return whether a cached depth stamp is older than the age limit."""
        try:
            now = self._node.get_clock().now()
            stamp_time = Time.from_msg(depth_stamp)
            age_s = (now - stamp_time).nanoseconds / 1e9
        except Exception:
            # A clock that is not started, or an invalid stamp, should not
            # prevent a usable depth sample from being localized.
            return False

        if age_s <= 0:
            return False
        if age_s > self._max_depth_age_s:
            self._node.get_logger().warning(
                f"depth image is stale: age={age_s:.3f}s",
                throttle_duration_sec=5.0,
            )
            return True
        return False
