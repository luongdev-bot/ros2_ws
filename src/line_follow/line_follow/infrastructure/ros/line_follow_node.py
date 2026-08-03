"""Camera line-following ROS 2 node.

    subscribe  <image_topic>    (sensor_msgs/Image)
    publish    <cmd_vel_topic>  (geometry_msgs/Twist)
    publish    ~/debug_image    (sensor_msgs/Image)
    service    ~/enable         (std_srvs/SetBool) - line following on/off
"""

import os
import sys

import cv2
import rclpy
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
from ...domain.errors import LineFollowError
from ...domain.pid import PID
from ..config_loader import load_line_follow_config
from ..lab_line_detector import LabLineDetector, draw_debug

#: Resolved from the installed share directory so the node is independent
#: of the source-workspace layout.
DEFAULT_CONFIG = os.path.join(
    get_package_share_directory("line_follow"), "config", "line_follow.yaml"
)


class LineFollowNode(Node):
    """Publish chassis steering computed from a camera line detection."""

    def __init__(self) -> None:
        super().__init__("line_follow")

        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("image_topic", "/depth_cam/image")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cruise_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.8)
        self.declare_parameter("min_speed_scale", 0.4)
        self.declare_parameter("lost_line_timeout_s", 0.4)
        self.declare_parameter("camera_timeout_s", 1.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("use_cuda", True)

        config_path = self.get_parameter("config").value
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
        self._last_image_time = None
        self._camera_stalled = False

        try:
            color, rois, gains = load_line_follow_config(config_path)
        except (OSError, LineFollowError) as exc:
            self.get_logger().fatal(f"cannot load line-follow config: {exc}")
            raise

        self._bridge = CvBridge()
        use_cuda = bool(self.get_parameter("use_cuda").value)
        if use_cuda:
            try:
                cuda_device_count = cv2.cuda.getCudaEnabledDeviceCount()
            except Exception:
                cuda_device_count = 0
            if cuda_device_count > 0:
                try:
                    self._detector = LabLineDetector(
                        color, rois, use_cuda=True
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        "CUDA line detector initialization failed "
                        f"({exc}) - falling back to CPU detection"
                    )
                    self._detector = LabLineDetector(color, rois)
                else:
                    self.get_logger().info(
                        "CUDA line detection enabled "
                        f"({cuda_device_count} device(s))"
                    )
            else:
                self.get_logger().warning(
                    "use_cuda:=true but no CUDA device is available "
                    "(cv2 built without CUDA support, or the CUDA-enabled "
                    "cv2 build is not on PYTHONPATH) - falling back to CPU "
                    "detection"
                )
                self._detector = LabLineDetector(color, rois)
        else:
            self._detector = LabLineDetector(color, rois)
        pid = PID(
            gains.kp,
            gains.ki,
            gains.kd,
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

        # The image, enable, and watchdog callbacks touch shared node/use-case
        # state. None blocks, so one mutually exclusive group prevents races
        # while a MultiThreadedExecutor serves the node.
        self._callback_group = MutuallyExclusiveCallbackGroup()

        self._cmd_vel_pub = self.create_publisher(
            Twist, cmd_vel_topic, 1
        )
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
            callback_group=self._callback_group,
        )

        state = "enabled" if self._enabled else "disabled"
        self.get_logger().info(
            f"line following {state}: {image_topic} -> {cmd_vel_topic}"
        )

    def _image_callback(self, msg: Image) -> None:
        self._last_image_time = self.get_clock().now()
        self._camera_stalled = False
        if not self._enabled:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as exc:
            self.get_logger().warning(
                f"dropping frame: {exc}", throttle_duration_sec=5.0
            )
            return

        command = self._use_case.process_frame(
            frame,
            frame_width=float(frame.shape[1]),
        )
        self._publish_command(command.linear_x, command.angular_z)

        if self._publish_debug:
            self._publish_debug_image(frame, msg.header)

    def _publish_command(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _publish_debug_image(self, frame, header) -> None:
        # Reuses the detection the use case just computed - re-running the
        # detector here would double the per-frame cost.
        annotated = draw_debug(
            frame,
            self._detector.rois,
            self._use_case.last_detection,
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
        if (
            not self._enabled
            or self._last_image_time is None
            or self._camera_stalled
        ):
            return

        elapsed_s = (
            self.get_clock().now() - self._last_image_time
        ).nanoseconds * 1e-9
        if elapsed_s > self._camera_timeout_s:
            self._camera_stalled = True
            self._publish_command(0.0, 0.0)
            self.get_logger().warning(
                f"camera stream stalled for {elapsed_s:.2f}s; stopping",
                throttle_duration_sec=5.0,
            )

    def _enable_callback(self, request, response):
        requested_enabled = bool(request.data)
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
        node = LineFollowNode()
    except Exception as exc:  # config errors already logged as fatal
        print(f"line_follow failed to start: {exc}", file=sys.stderr)
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
