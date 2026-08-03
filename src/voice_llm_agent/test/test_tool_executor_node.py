"""Focused tests for the ROS tool-executor adapter."""

import math
from unittest.mock import patch

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from voice_llm_agent.infrastructure.ros.tool_executor_node import (
    ToolExecutorNode,
)


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_initializes_ros_interfaces(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        assert node._cmd_vel_publisher is not None
        assert node._agent_reply_publisher is not None
        assert node._user_utterance_subscription is not None
        assert node._camera_subscription is not None
        assert node._scan_subscription is not None
        assert node._grasp_parameters_client is not None
        assert node._grasp_trigger_client is not None
        assert node._line_follow_client is not None
        assert node._navigation_client is not None
    finally:
        node.destroy_node()


def test_lidar_scan_detect_reports_near_obstacle(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 180.0
        scan.ranges = [1.0] * 361
        scan.ranges[180] = 0.2
        node._latest_scan = scan

        reply = node.lidar_scan_detect("LLM đoán không đáng tin")

        assert "Phát hiện vật cản" in reply
    finally:
        node.destroy_node()


def test_lidar_scan_detect_reports_clear_forward_sector(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 180.0
        scan.ranges = [1.0] * 361
        node._latest_scan = scan

        reply = node.lidar_scan_detect("bất kỳ")

        assert "Không phát hiện vật cản" in reply
    finally:
        node.destroy_node()


def test_lidar_scan_detect_without_scan(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        assert node.lidar_scan_detect("bất kỳ") == (
            "Chưa nhận được dữ liệu lidar."
        )
    finally:
        node.destroy_node()


def test_object_track_rejects_unparseable_box(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        assert node.object_track("không có tọa độ") == (
            "Không hiểu định dạng vùng ảnh: không có tọa độ"
        )
    finally:
        node.destroy_node()


def test_object_track_without_camera_frame(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        with patch.object(node, "_get_latest_frame", return_value=None):
            reply = node.object_track("[0, 0, 10, 10]")

        assert reply == "Chưa nhận được hình ảnh từ camera."
    finally:
        node.destroy_node()


def test_move_to_location_with_empty_location_table(ros_context) -> None:
    node = ToolExecutorNode()
    try:
        assert node.move_to_location("Kho hàng") == (
            "Không tìm thấy địa điểm 'Kho hàng' trong danh sách đã biết."
        )
    finally:
        node.destroy_node()


def test_location_enum_and_unaccented_lookup(ros_context, monkeypatch) -> None:
    locations = {
        "Khu tiếp nhận": {"x": 1.0, "y": 2.0, "yaw": 90.0},
    }
    monkeypatch.setattr(
        ToolExecutorNode,
        "_load_locations",
        lambda _self, _yaml_path: locations,
    )

    node = ToolExecutorNode()
    try:
        move_tool = next(
            tool
            for tool in node._process_utterance._tools
            if tool["function"]["name"] == "move_to_location"
        )
        destination_schema = move_tool["function"]["parameters"][
            "properties"
        ]["destination"]

        assert destination_schema["enum"] == ["Khu tiếp nhận"]
        assert ToolExecutorNode._normalize_location_name(
            "Khu tiếp nhận"
        ) == "khu_tiep_nhan"
        assert node._find_location("khu_tiep_nhan") is locations[
            "Khu tiếp nhận"
        ]
    finally:
        node.destroy_node()


def test_line_following_rejects_unconfigured_color_immediately(
    ros_context,
) -> None:
    node = ToolExecutorNode()
    try:
        started = node.get_clock().now()
        reply = node.line_following("red")
        elapsed_s = (node.get_clock().now() - started).nanoseconds / 1e9

        assert reply == (
            "Robot hiện chỉ được cấu hình dò vạch màu black, "
            "chưa hỗ trợ đổi sang màu red lúc chạy."
        )
        assert elapsed_s < 1.0
    finally:
        node.destroy_node()
