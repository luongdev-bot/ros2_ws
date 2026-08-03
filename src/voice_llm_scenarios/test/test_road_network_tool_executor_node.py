"""Tests for graph-routed named navigation without a Nav2 server."""

from unittest.mock import call, patch

import pytest
import rclpy

from voice_llm_agent.infrastructure.ros.tool_executor_node import (
    ToolExecutorNode,
)
from voice_llm_scenarios.infrastructure.road_network_tool_executor_node import (
    RoadNetworkToolExecutorNode,
)


LOCATIONS_YAML = """\
Điểm xuất phát: {x: 0.0, y: 0.0, yaw: 0.0}
Khu trung tâm: {x: 6.73, y: 1.38, yaw: 0.0}
Góc tây nam: {x: -7.57, y: -4.12, yaw: 0.0}
Khu vực kệ hàng: {x: 15.83, y: -1.57, yaw: 0.0}
Khu tiếp nhận: {x: 3.53, y: -6.22, yaw: 0.0}
"""


@pytest.fixture
def node_factory(monkeypatch, tmp_path):
    """Create one executor node with temporary locations and ROS context."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    locations_path = tmp_path / "locations_warehouse.yaml"
    locations_path.write_text(LOCATIONS_YAML, encoding="utf-8")
    nodes = []

    def create_node(
        start_location_name: str | None = None,
    ) -> RoadNetworkToolExecutorNode:
        ros_args = [
            "--ros-args",
            "-p",
            f"locations_yaml_path:={locations_path}",
        ]
        if start_location_name is not None:
            ros_args.extend(
                [
                    "-p",
                    f"start_location_name:={start_location_name}",
                ]
            )

        rclpy.init(args=ros_args)
        node = RoadNetworkToolExecutorNode()
        nodes.append(node)
        return node

    try:
        yield create_node
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _successful_move(_node, destination: str) -> str:
    return f"Đã di chuyển tới {destination}."


def test_moves_from_default_start_to_shelf_via_center(node_factory) -> None:
    node = node_factory()

    with patch.object(
        ToolExecutorNode,
        "move_to_location",
        autospec=True,
        side_effect=_successful_move,
    ) as parent_move:
        reply = node.move_to_location("Khu vực kệ hàng")

    assert parent_move.call_args_list == [
        call(node, "Khu trung tâm"),
        call(node, "Khu vực kệ hàng"),
    ]
    assert node._current_location_name == "Khu vực kệ hàng"
    assert reply == (
        "Đã đi qua 2 chặng "
        "(Điểm xuất phát → Khu trung tâm → Khu vực kệ hàng) "
        "và tới Khu vực kệ hàng."
    )


def test_moves_three_hops_in_order_and_updates_current_location(
    node_factory,
) -> None:
    node = node_factory("Góc tây nam")

    with patch.object(
        ToolExecutorNode,
        "move_to_location",
        autospec=True,
        side_effect=_successful_move,
    ) as parent_move:
        reply = node.move_to_location("khu_vuc_ke_hang")

    assert parent_move.call_args_list == [
        call(node, "Điểm xuất phát"),
        call(node, "Khu trung tâm"),
        call(node, "Khu vực kệ hàng"),
    ]
    assert node._current_location_name == "Khu vực kệ hàng"
    assert reply == (
        "Đã đi qua 3 chặng "
        "(Góc tây nam → Điểm xuất phát → Khu trung tâm → "
        "Khu vực kệ hàng) và tới khu_vuc_ke_hang."
    )


def test_stops_after_failed_middle_hop_without_updating_location(
    node_factory,
) -> None:
    node = node_factory("Góc tây nam")

    with patch.object(
        ToolExecutorNode,
        "move_to_location",
        autospec=True,
        side_effect=[
            "Đã di chuyển tới Điểm xuất phát.",
            "Không thể di chuyển tới Khu trung tâm.",
            "Đã di chuyển tới Khu vực kệ hàng.",
        ],
    ) as parent_move:
        reply = node.move_to_location("Khu vực kệ hàng")

    assert parent_move.call_args_list == [
        call(node, "Điểm xuất phát"),
        call(node, "Khu trung tâm"),
    ]
    assert node._current_location_name == "Góc tây nam"
    assert reply == (
        "Di chuyển thất bại tại chặng Khu trung tâm "
        "trên đường tới Khu vực kệ hàng."
    )
