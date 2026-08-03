"""YOLO-segmentation camera line-following ROS 2 node.

    subscribe  <image_topic>    (sensor_msgs/Image)
    publish    <cmd_vel_topic>  (geometry_msgs/Twist)
    publish    ~/debug_image    (sensor_msgs/Image)
    service    ~/enable         (std_srvs/SetBool) - line following on/off
"""

import os
import sys
import threading

import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from ...application.line_follow import LineFollowUseCase
from ...domain.pid import PID
from ..yolo_line_detector import YoloLineDetector, draw_debug


#: Resolved from the installed share directory so the node is independent
#: of the source-workspace layout.
DEFAULT_MODEL = os.path.join(
    get_package_share_directory("line_follow"),
    "models",
    "line_yolo11n_seg.pt",
)


class YoloLineFollowNode(Node):
    """Publish chassis steering computed from a YOLO line detection."""

    def __init__(self) -> None:
        super().__init__("yolo_line_follow")

        self.declare_parameter("model_path", DEFAULT_MODEL)
        self.declare_parameter("image_topic", "/depth_cam/image")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cruise_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("min_speed_scale", 0.4)
        self.declare_parameter("pid_kp", 0.006)
        self.declare_parameter("pid_ki", 0.0)
        self.declare_parameter("pid_kd", 0.0)
        self.declare_parameter("lost_line_timeout_s", 0.4)
        self.declare_parameter("camera_timeout_s", 1.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("device", "0")
        self.declare_parameter("imgsz", 640)

        model_path = os.path.expanduser(
            self.get_parameter("model_path").value
        )
        image_topic = self.get_parameter("image_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self._publish_debug = bool(
            self.get_parameter("publish_debug_image").value
        )
        self._enabled = bool(self.get_parameter("start_enabled").value)
        self._camera_timeout_s = float(
            self.get_parameter("camera_timeout_s").value
        )
        if self._camera_timeout_s <= 0.0:
            raise ValueError(
                "camera_timeout_s must be positive, "
                f"got {self._camera_timeout_s}"
            )
        self._state_lock = threading.Lock()
        self._last_image_time = None
        self._camera_stalled = False

        requested_device = str(self.get_parameter("device").value)
        device = requested_device
        if requested_device != "cpu" and not torch.cuda.is_available():
            self.get_logger().warning(
                "device requests GPU inference but no CUDA device is "
                "available (torch.cuda.is_available() is false) - falling "
                "back to CPU inference"
            )
            device = "cpu"

        self._bridge = CvBridge()
        self._detector = YoloLineDetector(
            model_path,
            confidence_threshold=float(
                self.get_parameter("confidence_threshold").value
            ),
            device=device,
            imgsz=int(self.get_parameter("imgsz").value),
        )
        pid = PID(
            float(self.get_parameter("pid_kp").value),
            float(self.get_parameter("pid_ki").value),
            float(self.get_parameter("pid_kd").value),
            output_min=-max_angular_speed,
            output_max=max_angular_speed,
        )
        self._use_case = LineFollowUseCase(
            self._detector,
            pid,
            cruise_speed=float(self.get_parameter("cruise_speed").value),
            max_angular_speed=max_angular_speed,
            min_speed_scale=float(
                self.get_parameter("min_speed_scale").value
            ),
            lost_line_timeout_s=float(
                self.get_parameter("lost_line_timeout_s").value
            ),
            # Steering dt and lost-line deadlines use ROS time so simulation
            # speed and paused /clock behavior stay consistent.
            clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
        )

        # Image and enable callbacks share node/use-case state, so they remain
        # serialized. YOLO inference blocks, so the watchdog gets its own
        # group and can publish a stop while an image callback is running.
        # The lock keeps stall state and command publication ordered across
        # those two callback groups.
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._watchdog_callback_group = MutuallyExclusiveCallbackGroup()

        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 1)
        self._debug_pub = self.create_publisher(Image, "~/debug_image", 1)
        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetBool,
            "~/enable",
            self._enable_callback,
            callback_group=self._callback_group,
        )
        self.create_timer(
            min(1.0, self._camera_timeout_s / 2.0),
            self._check_camera_watchdog,
            callback_group=self._watchdog_callback_group,
        )

        state = "enabled" if self._enabled else "disabled"
        self.get_logger().info(
            f"YOLO line following {state}: {image_topic} -> {cmd_vel_topic}"
        )

    def _image_callback(self, msg: Image) -> None:
        with self._state_lock:
            self._last_image_time = self.get_clock().now()
            self._camera_stalled = False
            enabled = self._enabled
        if not enabled:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as exc:
            self._publish_command(0.0, 0.0)
            self.get_logger().warning(
                f"dropping frame: {exc}", throttle_duration_sec=5.0
            )
            return

        try:
            command = self._use_case.process_frame(
                frame,
                frame_width=float(frame.shape[1]),
            )
        except Exception as exc:
            self._publish_command(0.0, 0.0)
            self.get_logger().error(
                f"line inference failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return
        with self._state_lock:
            if self._camera_stalled:
                return
            self._publish_command(command.linear_x, command.angular_z)

        if self._publish_debug:
            self._publish_debug_image(frame, msg.header)

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _publish_debug_image(self, frame, header) -> None:
        # Reuses the detection and polygon from the inference just completed.
        annotated = draw_debug(
            frame,
            self._use_case.last_detection,
            self._detector.last_polygon,
        )
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(
                annotated, encoding="bgr8"
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f"debug image failed: {exc}")
            return
        debug_msg.header = header
        self._debug_pub.publish(debug_msg)

    def _check_camera_watchdog(self) -> None:
        with self._state_lock:
            if (
                not self._enabled
                or self._last_image_time is None
                or self._camera_stalled
            ):
                return

            elapsed_s = (
                self.get_clock().now() - self._last_image_time
            ).nanoseconds * 1e-9
            if elapsed_s <= self._camera_timeout_s:
                return
            self._camera_stalled = True
            self._publish_command(0.0, 0.0)
        self.get_logger().warning(
            f"camera stream stalled for {elapsed_s:.2f}s; stopping",
            throttle_duration_sec=5.0,
        )

    def _enable_callback(self, request, response):
        requested_enabled = bool(request.data)
        with self._state_lock:
            if self._enabled and not requested_enabled:
                self._enabled = False
                self._publish_command(0.0, 0.0)
            elif not self._enabled and requested_enabled:
                self._use_case.reset()
                self._enabled = True

            state = "enabled" if self._enabled else "disabled"
        response.success = True
        response.message = f"line following {state}"
        self.get_logger().info(response.message)
        return response


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = YoloLineFollowNode()
    except Exception as exc:
        print(f"yolo_line_follow failed to start: {exc}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
