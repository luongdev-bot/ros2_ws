"""Colour-block pick node.

    subscribe  <image_topic>            (sensor_msgs/Image)
    publish    ~/detections             (vision_msgs/Detection2DArray)
    publish    ~/debug_image            (sensor_msgs/Image)
    service    ~/enable                 (std_srvs/SetBool) - arm triggering on/off

Sees a coloured block, confirms it over several frames, then asks
arm_motion to play the action group bound to that colour.
"""

import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from ...application.color_pick import ColorPickUseCase
from ...domain.errors import PerceptionError
from ...domain.pick_policy import PickPolicy
from ..config_loader import load_color_pick_config
from ..lab_block_detector import LabBlockDetector, draw_detections
from .motion_player import RosMotionPlayer

#: Resolved from the installed share directory, so the node works from any
#: workspace layout - not just ~/ros2_ws/src.
DEFAULT_CONFIG = os.path.join(
    get_package_share_directory("arm_perception"), "config", "color_pick.yaml"
)


class ColorPickNode(Node):

    def __init__(self) -> None:
        super().__init__("color_pick")

        self.declare_parameter("config", DEFAULT_CONFIG)
        self.declare_parameter("image_topic", "/depth_cam/image")
        self.declare_parameter("play_motion_action", "/arm_motion_server/play_motion")
        self.declare_parameter("confirm_frames", 5)
        self.declare_parameter("cooldown_s", 3.0)
        # 0.0 disables the centring check - useful while tuning, when you
        # want a trigger from anywhere in frame.
        self.declare_parameter("center_tolerance_px", 0.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("proc_width", 320)
        self.declare_parameter("proc_height", 240)

        config_path = self.get_parameter("config").value
        image_topic = self.get_parameter("image_topic").value
        tolerance = float(self.get_parameter("center_tolerance_px").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)

        try:
            palette, motion_map = load_color_pick_config(config_path)
        except (OSError, PerceptionError) as exc:
            self.get_logger().fatal(f"cannot load colour config: {exc}")
            raise

        self._bridge = CvBridge()
        self._detector = LabBlockDetector(
            palette,
            proc_size=(
                int(self.get_parameter("proc_width").value),
                int(self.get_parameter("proc_height").value),
            ),
        )

        policy = PickPolicy(
            confirm_frames=int(self.get_parameter("confirm_frames").value),
            cooldown_s=float(self.get_parameter("cooldown_s").value),
            center_tolerance_px=tolerance if tolerance > 0 else None,
        )

        # Frame processing MUST be mutually exclusive: under a
        # MultiThreadedExecutor a reentrant group would let _image_callback
        # run concurrently with itself (OpenCV releases the GIL), and both
        # PickPolicy's streak/state and RosMotionPlayer's busy flag are
        # plain unsynchronised attributes. Two interleaved frames could each
        # see "not busy" and dispatch a goal. The ~/enable service shares the
        # group because it mutates the same use-case state.
        self._frame_group = MutuallyExclusiveCallbackGroup()
        # The action client gets its own group so goal/result callbacks are
        # not stuck behind a frame that is still being processed.
        self._action_group = ReentrantCallbackGroup()
        self._player = RosMotionPlayer(
            self,
            self.get_parameter("play_motion_action").value,
            callback_group=self._action_group,
        )

        self._detections_pub = self.create_publisher(
            Detection2DArray, "~/detections", 10
        )
        self._debug_pub = self.create_publisher(Image, "~/debug_image", 1)

        # The sink receives each frame's detections from inside the use
        # case, so they are published without running detection twice.
        self._use_case = ColorPickUseCase(
            self._detector,
            policy,
            motion_map,
            self._player,
            clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
            detection_sink=self._handle_detections,
        )
        if not bool(self.get_parameter("start_enabled").value):
            self._use_case.disable()

        self._latest_header = None
        self._last_reason = ""
        self._latest_detections = []

        self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data,
            callback_group=self._frame_group,
        )
        self.create_service(
            SetBool, "~/enable", self._enable_callback,
            callback_group=self._frame_group,
        )

        self.get_logger().info(
            f"watching {image_topic} for {', '.join(palette.names())}"
        )
        for color, motion in motion_map.as_dict().items():
            self.get_logger().info(f"  {color:<8} -> action group {motion!r}")
        if not self._player.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning(
                f"{self.get_parameter('play_motion_action').value} not up yet - "
                "will retry when a block is confirmed"
            )

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"dropping frame: {exc}", throttle_duration_sec=5.0)
            return

        self._latest_header = msg.header
        height, width = frame.shape[:2]

        decision = self._use_case.process_frame(
            frame, frame_center=(width / 2.0, height / 2.0)
        )

        if decision.should_play:
            self.get_logger().info(
                f"{decision.color} block confirmed at "
                f"({decision.detection.center_x:.0f}, {decision.detection.center_y:.0f})"
            )
        elif decision.reason and decision.reason != self._last_reason:
            # Log only on change: at 30 Hz "no block in view" would
            # otherwise flood the console.
            self.get_logger().debug(decision.reason)
        self._last_reason = decision.reason

        if self._publish_debug:
            self._publish_debug_image(frame, msg.header)

    def _handle_detections(self, detections) -> None:
        """Detection sink: called once per frame from inside the use case."""
        self._latest_detections = list(detections)
        self._publish_detections(self._latest_detections)

    def _publish_detections(self, detections) -> None:
        msg = Detection2DArray()
        if self._latest_header is not None:
            msg.header = self._latest_header

        for det in detections:
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = det.center_x
            detection.bbox.center.position.y = det.center_y
            # vision_msgs specifies theta in radians; OpenCV gives degrees.
            detection.bbox.center.theta = math.radians(det.angle_deg)
            detection.bbox.size_x = det.width
            detection.bbox.size_y = det.height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det.color
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self._detections_pub.publish(msg)

    def _publish_debug_image(self, frame, header) -> None:
        # Reuses the detections the use case just computed - re-running
        # the detector here would double the per-frame cost.
        annotated = draw_detections(frame, self._latest_detections)
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"debug image failed: {exc}")
            return
        debug_msg.header = header
        self._debug_pub.publish(debug_msg)

    def _enable_callback(self, request, response):
        if request.data:
            self._use_case.enable()
            response.message = "colour picking enabled"
        else:
            self._use_case.disable()
            response.message = "colour picking disabled"
        response.success = True
        self.get_logger().info(response.message)
        return response


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = ColorPickNode()
    except Exception as exc:  # config errors already logged as fatal
        print(f"color_pick failed to start: {exc}", file=sys.stderr)
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
