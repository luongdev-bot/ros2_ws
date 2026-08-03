"""ROS 2 adapter for the robot tools and text-agent bridge.

Service and action calls are started from utterance worker threads.  Their
futures are completed by the :class:`MultiThreadedExecutor` spun in
``main()``; worker threads must never spin this node themselves.  ROS 2
Humble's ``rclpy.task.Future.result`` has no timeout argument, so this module
waits on a thread event with a wall-clock timeout before reading the result.

Location YAML entries use degrees for ``yaw``.  A minimal file looks like::

    kho_hang: {x: 1.0, y: 2.0, yaw: 90.0}
"""

import json
import math
import re
import threading
import time
import unicodedata
from typing import Optional

import cv2
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from ...application.process_utterance import ProcessUtterance
from ...domain.ports import RobotToolExecutorPort
from ...domain.tool_schemas import TOOL_SCHEMAS, with_location_enum
from ..llm.ollama_client import OllamaClient


class ToolExecutorNode(Node, RobotToolExecutorPort):
    """Implement robot tool ports with ROS topics, services, and actions."""

    _SERVICE_WAIT_TIMEOUT_S = 5.0
    _SERVICE_RESPONSE_TIMEOUT_S = 10.0

    def __init__(self) -> None:
        super().__init__("tool_executor")

        self.declare_parameter("camera_topic", "depth_cam/rgb/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("grasp_executor_node_name", "grasp_executor")
        self.declare_parameter(
            "line_follow_enable_service",
            "/line_follow/enable",
        )
        self.declare_parameter("line_follow_configured_color", "black")
        self.declare_parameter("locations_yaml_path", "")
        self.declare_parameter("ollama_base_url", "http://localhost:11434")
        self.declare_parameter("ollama_model", "qwen2.5vl:3b")
        self.declare_parameter("nav_timeout_s", 120.0)
        self.declare_parameter("object_track_duration_s", 15.0)
        self.declare_parameter("object_track_center_kp", 0.003)
        self.declare_parameter("object_track_forward_speed", 0.1)
        self.declare_parameter("obstacle_distance_threshold_m", 0.3)
        self.declare_parameter("obstacle_forward_angle_deg", 30.0)

        camera_topic = str(self.get_parameter("camera_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        grasp_node_name = str(
            self.get_parameter("grasp_executor_node_name").value
        ).strip("/")
        line_follow_service = str(
            self.get_parameter("line_follow_enable_service").value
        )
        self._line_follow_configured_color = str(
            self.get_parameter("line_follow_configured_color").value
        )
        locations_path = str(
            self.get_parameter("locations_yaml_path").value
        )
        ollama_base_url = str(
            self.get_parameter("ollama_base_url").value
        )
        ollama_model = str(self.get_parameter("ollama_model").value)
        self._nav_timeout_s = float(
            self.get_parameter("nav_timeout_s").value
        )
        self._object_track_duration_s = float(
            self.get_parameter("object_track_duration_s").value
        )
        self._object_track_center_kp = float(
            self.get_parameter("object_track_center_kp").value
        )
        self._object_track_forward_speed = float(
            self.get_parameter("object_track_forward_speed").value
        )
        self._obstacle_distance_threshold_m = float(
            self.get_parameter("obstacle_distance_threshold_m").value
        )
        self._obstacle_forward_angle_deg = float(
            self.get_parameter("obstacle_forward_angle_deg").value
        )

        self._bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None
        self._scan_lock = threading.Lock()
        self._locations = self._load_locations(locations_path)

        self._llm = OllamaClient(
            base_url=ollama_base_url,
            model=ollama_model,
        )
        tools = with_location_enum(
            TOOL_SCHEMAS,
            list(self._locations.keys()),
        )
        self._process_utterance = ProcessUtterance(
            llm=self._llm,
            executor=self,
            tools=tools,
        )

        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )
        self._agent_reply_publisher = self.create_publisher(
            String,
            "~/agent_reply",
            10,
        )
        self._user_utterance_subscription = self.create_subscription(
            String,
            "~/user_utterance",
            self._on_user_utterance,
            10,
        )
        self._camera_subscription = self.create_subscription(
            Image,
            camera_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._grasp_parameters_client = self.create_client(
            SetParameters,
            f"/{grasp_node_name}/set_parameters",
        )
        self._grasp_trigger_client = self.create_client(
            Trigger,
            f"/{grasp_node_name}/grasp_next",
        )
        self._line_follow_client = self.create_client(
            SetBool,
            line_follow_service,
        )
        self._navigation_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

    def _on_user_utterance(self, message: String) -> None:
        """Start slow LLM/tool orchestration outside executor callbacks."""
        worker = threading.Thread(
            target=self._handle_user_utterance,
            args=(message.data,),
            daemon=True,
            name="tool_executor_utterance",
        )
        worker.start()

    def _handle_user_utterance(self, user_text: str) -> None:
        try:
            reply = self._process_utterance.handle(user_text)
        except Exception as error:
            self.get_logger().error(f"Không thể xử lý phát ngôn: {error}")
            reply = "Không thể xử lý yêu cầu lúc này."
        self._agent_reply_publisher.publish(String(data=reply))

    def _on_image(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as error:
            self.get_logger().warning(
                f"Không thể chuyển đổi ảnh camera: {error}"
            )
            return

        with self._frame_lock:
            self._latest_frame = frame

    def _get_latest_frame(self):
        """Return an isolated copy of the newest camera frame, if present."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def _on_scan(self, message: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = message

    def _get_latest_scan(self) -> Optional[LaserScan]:
        """Return the newest lidar scan, if one has been received."""
        with self._scan_lock:
            return self._latest_scan

    def _latest_frame_as_jpeg(self) -> bytes | None:
        frame = self._get_latest_frame()
        if frame is None:
            return None
        encoded_ok, encoded = cv2.imencode(".jpg", frame)
        if not encoded_ok:
            raise RuntimeError("Không thể mã hóa ảnh camera thành JPEG")
        return encoded.tobytes()

    @staticmethod
    def _wait_for_future_result(future, timeout_s: float):
        """Wait for an executor-owned future without spinning this node."""
        if future.done():
            return future.result()

        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not future.done() and not done.wait(max(0.0, timeout_s)):
            raise TimeoutError("ROS future timed out")
        return future.result()

    def _load_locations(self, yaml_path: str) -> dict[str, dict]:
        """Load named map poses whose yaw values are expressed in degrees."""
        if not yaml_path:
            return {}

        try:
            with open(yaml_path, encoding="utf-8") as yaml_file:
                raw_locations = yaml.safe_load(yaml_file) or {}
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().error(
                f"Không thể đọc danh sách địa điểm {yaml_path!r}: {error}"
            )
            return {}

        if not isinstance(raw_locations, dict):
            self.get_logger().error(
                "Danh sách địa điểm phải là một mapping YAML."
            )
            return {}

        locations = {}
        for name, pose in raw_locations.items():
            if not isinstance(name, str) or not isinstance(pose, dict):
                self.get_logger().warning(
                    f"Bỏ qua địa điểm không hợp lệ: {name!r}"
                )
                continue
            if not {"x", "y", "yaw"}.issubset(pose):
                self.get_logger().warning(
                    f"Bỏ qua địa điểm {name!r}: thiếu x, y hoặc yaw"
                )
                continue
            try:
                locations[name] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw": float(pose["yaw"]),
                }
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"Bỏ qua địa điểm {name!r}: tọa độ không hợp lệ"
                )
        return locations

    @staticmethod
    def _normalize_location_case_and_spacing(name: str) -> str:
        return name.strip().lower().replace(" ", "_")

    @classmethod
    def _normalize_location_name(cls, name: str) -> str:
        normalized = cls._normalize_location_case_and_spacing(name)
        decomposed = unicodedata.normalize("NFD", normalized)
        return "".join(
            character
            for character in decomposed
            if not unicodedata.combining(character)
        )

    def _find_location(self, destination: str) -> dict | None:
        if destination in self._locations:
            return self._locations[destination]

        normalized_case_and_spacing = (
            self._normalize_location_case_and_spacing(destination)
        )
        for name, pose in self._locations.items():
            if (
                self._normalize_location_case_and_spacing(name)
                == normalized_case_and_spacing
            ):
                return pose

        normalized_destination = self._normalize_location_name(destination)
        for name, pose in self._locations.items():
            if self._normalize_location_name(name) == normalized_destination:
                return pose
        return None

    def robot_move_control(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
    ) -> str:
        """Publish a velocity for ``duration`` seconds, then stop the robot."""
        command = Twist()
        command.linear.x = float(linear_x)
        command.linear.y = float(linear_y)
        command.angular.z = float(angular_z)
        duration_s = float(duration)

        self._cmd_vel_publisher.publish(command)
        try:
            time.sleep(duration_s)
        finally:
            self._cmd_vel_publisher.publish(Twist())

        return (
            "Đã điều khiển robot với vận tốc "
            f"x={linear_x}, y={linear_y}, góc z={angular_z} "
            f"trong {duration} giây."
        )

    def arm_transport_function(self, color: str, action: str) -> str:
        """Run the whole grasp-and-place cycle for the requested color.

        ``grasp_executor`` cannot split pick from place, so ``action`` is
        recorded as intent only and never changes the actual ROS operation.
        """
        self.get_logger().info(
            f"Ý định tay máy: action={action!r}, color={color!r}"
        )

        try:
            if not self._grasp_parameters_client.wait_for_service(
                timeout_sec=self._SERVICE_WAIT_TIMEOUT_S
            ):
                return "Chưa kết nối được với hệ thống gắp vật."

            set_request = SetParameters.Request()
            set_request.parameters = [
                Parameter(
                    name="grasp_order",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING_ARRAY,
                        string_array_value=[color],
                    ),
                )
            ]
            set_response = self._wait_for_future_result(
                self._grasp_parameters_client.call_async(set_request),
                self._SERVICE_RESPONSE_TIMEOUT_S,
            )
            if (
                set_response is None
                or not set_response.results
                or not set_response.results[0].successful
            ):
                return "Không thể cấu hình màu vật cần gắp."

            if not self._grasp_trigger_client.wait_for_service(
                timeout_sec=self._SERVICE_WAIT_TIMEOUT_S
            ):
                return "Chưa kết nối được với hệ thống gắp vật."

            grasp_response = self._wait_for_future_result(
                self._grasp_trigger_client.call_async(Trigger.Request()),
                self._SERVICE_RESPONSE_TIMEOUT_S,
            )
            if grasp_response is not None and grasp_response.success:
                return f"Đã bắt đầu gắp và đặt vật màu {color}."
            if (
                grasp_response is not None
                and grasp_response.message.strip().lower()
                == "grasp cycle already running"
            ):
                return "Tay máy đang bận với một vật khác, vui lòng đợi."
            return f"Không thể gắp và đặt vật màu {color}."
        except TimeoutError as error:
            self.get_logger().error(f"Hết thời gian chờ hệ thống gắp: {error}")
            return "Chưa kết nối được với hệ thống gắp vật."
        except Exception as error:
            self.get_logger().error(f"Lỗi hệ thống gắp vật: {error}")
            return f"Không thể gắp và đặt vật màu {color}."

    def line_following(self, color: str) -> str:
        """Enable line following only for its startup-configured color."""
        configured = self._line_follow_configured_color
        if color.strip().lower() != configured.strip().lower():
            return (
                "Robot hiện chỉ được cấu hình dò vạch màu "
                f"{configured}, chưa hỗ trợ đổi sang màu {color} lúc chạy."
            )

        try:
            if not self._line_follow_client.wait_for_service(
                timeout_sec=self._SERVICE_WAIT_TIMEOUT_S
            ):
                return "Chưa kết nối được với hệ thống dò vạch."
            request = SetBool.Request()
            request.data = True
            response = self._wait_for_future_result(
                self._line_follow_client.call_async(request),
                self._SERVICE_RESPONSE_TIMEOUT_S,
            )
            if response is not None and response.success:
                return f"Đã bật chế độ dò vạch màu {configured}."
            return f"Không thể bật chế độ dò vạch màu {configured}."
        except Exception as error:
            self.get_logger().error(f"Lỗi hệ thống dò vạch: {error}")
            return f"Không thể bật chế độ dò vạch màu {configured}."

    def move_to_location(self, destination: str) -> str:
        """Navigate to a named map pose; YAML yaw values are in degrees."""
        location = self._find_location(destination)
        if location is None:
            return (
                f"Không tìm thấy địa điểm '{destination}' "
                "trong danh sách đã biết."
            )

        failure_reply = f"Không thể di chuyển tới {destination}."
        deadline = time.monotonic() + self._nav_timeout_s
        try:
            server_wait_s = min(
                self._SERVICE_WAIT_TIMEOUT_S,
                max(0.0, deadline - time.monotonic()),
            )
            if not self._navigation_client.wait_for_server(
                timeout_sec=server_wait_s
            ):
                return failure_reply

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = location["x"]
            goal.pose.pose.position.y = location["y"]
            yaw_radians = math.radians(location["yaw"])
            goal.pose.pose.orientation.z = math.sin(yaw_radians / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw_radians / 2.0)

            goal_handle = self._wait_for_future_result(
                self._navigation_client.send_goal_async(goal),
                deadline - time.monotonic(),
            )
            if goal_handle is None or not goal_handle.accepted:
                return failure_reply

            wrapped_result = self._wait_for_future_result(
                goal_handle.get_result_async(),
                deadline - time.monotonic(),
            )
            if (
                wrapped_result is not None
                and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            ):
                return f"Đã di chuyển tới {destination}."
            return failure_reply
        except Exception as error:
            self.get_logger().error(
                f"Không thể điều hướng tới {destination!r}: {error}"
            )
            return failure_reply

    def describe_current_view(self, question: str) -> str:
        """Answer a short Vietnamese question about the latest RGB frame."""
        image_bytes = self._latest_frame_as_jpeg()
        if image_bytes is None:
            return "Chưa nhận được hình ảnh từ camera."
        prompt = (
            "Bạn là một robot thông minh đang quan sát qua camera. "
            "Hãy trả lời ngắn gọn (dưới 40 từ) câu hỏi sau bằng tiếng Việt, "
            f"dựa trên hình ảnh: {question}"
        )
        return self._llm.vision(prompt, image_bytes)

    def get_object_box_distance(self, user_query: str) -> str:
        """Estimate near/far qualitatively from RGB, without true depth.

        This simplified simulation implementation asks the vision model for a
        qualitative estimate; it does not measure metric depth-camera data.
        """
        image_bytes = self._latest_frame_as_jpeg()
        if image_bytes is None:
            return "Chưa nhận được hình ảnh từ camera."
        prompt = (
            "Bạn là một robot thông minh. Hãy xác định (các) vật thể được "
            "nhắc tới trong câu sau và ước lượng khoảng cách một cách định "
            "tính (gần/xa) bằng tiếng Việt, không quá 40 từ: "
            f"{user_query}"
        )
        return self._llm.vision(prompt, image_bytes)

    def object_track(self, box: str) -> str:
        """Track an initial ``[x1, y1, x2, y2]`` box with CSRT."""
        coordinates = None
        stripped_box = box.strip()
        if stripped_box.startswith("["):
            try:
                parsed_box = json.loads(stripped_box)
                if isinstance(parsed_box, list) and len(parsed_box) == 4:
                    coordinates = [float(value) for value in parsed_box]
            except (json.JSONDecodeError, TypeError, ValueError):
                coordinates = None

        if coordinates is None:
            matches = re.findall(r"-?\d+\.?\d*", box)
            if len(matches) >= 4:
                coordinates = [float(value) for value in matches[:4]]

        if (
            coordinates is None
            or len(coordinates) != 4
            or not all(math.isfinite(value) for value in coordinates)
        ):
            return f"Không hiểu định dạng vùng ảnh: {box}"

        x1, y1, x2, y2 = (int(round(value)) for value in coordinates)
        width = x2 - x1
        height = y2 - y1
        if width <= 0 or height <= 0:
            return f"Không hiểu định dạng vùng ảnh: {box}"

        frame = self._get_latest_frame()
        if frame is None:
            return "Chưa nhận được hình ảnh từ camera."

        try:
            tracker = cv2.TrackerCSRT_create()
        except Exception as error:
            self.get_logger().error(
                f"Không thể tạo CSRT tracker: {error}"
            )
            return f"Không thể khởi tạo bộ bám vật thể CSRT: {error}"
        if tracker is None:
            return "Không thể khởi tạo bộ bám vật thể CSRT."

        try:
            tracker.init(frame, (x1, y1, width, height))
        except Exception as error:
            self.get_logger().error(
                f"Không thể khởi tạo vùng bám vật thể: {error}"
            )
            return f"Không thể khởi tạo vùng bám vật thể: {error}"

        started_at = time.monotonic()
        lost_track = False
        try:
            while (
                time.monotonic() - started_at
                < self._object_track_duration_s
            ):
                frame = self._get_latest_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue

                try:
                    ok, bounding_box = tracker.update(frame)
                except Exception as error:
                    self.get_logger().warning(
                        f"CSRT tracker không thể cập nhật: {error}"
                    )
                    lost_track = True
                    break
                if not ok:
                    lost_track = True
                    break

                bbox_x, bbox_y, bbox_width, bbox_height = bounding_box
                object_center_x = bbox_x + bbox_width / 2.0
                image_center_x = frame.shape[1] / 2.0
                horizontal_error = object_center_x - image_center_x
                bbox_area = bbox_width * bbox_height
                frame_area = float(frame.shape[0] * frame.shape[1])

                command = Twist()
                command.angular.z = (
                    -self._object_track_center_kp * horizontal_error
                )
                if bbox_area < 0.15 * frame_area:
                    command.linear.x = self._object_track_forward_speed
                self._cmd_vel_publisher.publish(command)
                time.sleep(0.1)
        finally:
            self._cmd_vel_publisher.publish(Twist())

        if lost_track:
            elapsed = time.monotonic() - started_at
            return f"Đã mất dấu vật thể sau {elapsed:.1f} giây bám theo."
        return (
            "Đã bám theo vật thể trong "
            f"{self._object_track_duration_s} giây."
        )

    def lidar_scan_detect(self, scan_detect: str) -> str:
        """Detect obstacles around 0 rad, the +x/front LaserScan axis.

        ``scan_detect`` is an untrusted LLM guess and is intentionally ignored.
        Following LaserScan/REP 103 conventions, 0 rad is treated as the
        robot-forward direction in ``lidar_frame``.
        """
        scan = self._get_latest_scan()
        if scan is None:
            return "Chưa nhận được dữ liệu lidar."

        half_angle = math.radians(self._obstacle_forward_angle_deg)
        valid_distances = []
        for index, distance in enumerate(scan.ranges):
            ray_angle = scan.angle_min + index * scan.angle_increment
            normalized_angle = math.atan2(
                math.sin(ray_angle),
                math.cos(ray_angle),
            )
            if abs(normalized_angle) > half_angle:
                continue
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            valid_distances.append(distance)

        if not valid_distances:
            return "Không phát hiện vật cản phía trước."

        min_distance = min(valid_distances)
        threshold = self._obstacle_distance_threshold_m
        if min_distance < threshold:
            return (
                "Phát hiện vật cản phía trước, cách khoảng "
                f"{min_distance:.2f} mét."
            )
        return (
            "Không phát hiện vật cản phía trước trong phạm vi "
            f"{threshold} mét (gần nhất: {min_distance:.2f} mét)."
        )


def main():
    """Spin with multiple executor threads so worker futures can complete."""
    rclpy.init()
    node = ToolExecutorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
