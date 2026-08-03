"""ROS 2 tool executor that navigates through a named road network."""

import rclpy

from voice_llm_agent.infrastructure.ros.tool_executor_node import (
    ToolExecutorNode,
)

from ..domain.road_graph import ROAD_GRAPH, shortest_path


class RoadNetworkToolExecutorNode(ToolExecutorNode):
    """Execute named navigation as a sequence of graph-adjacent hops."""

    _MOVE_SUCCESS_PREFIX = "Đã di chuyển tới"

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter(
            "start_location_name",
            "Điểm xuất phát",
        )
        self._current_location_name: str = str(
            self.get_parameter("start_location_name").value
        )

    def _find_road_graph_node_name(self, destination: str) -> str | None:
        """Match a graph key with the parent node's lookup strategy."""
        if destination in ROAD_GRAPH:
            return destination

        normalized_case_and_spacing = (
            self._normalize_location_case_and_spacing(destination)
        )
        for name in ROAD_GRAPH:
            if (
                self._normalize_location_case_and_spacing(name)
                == normalized_case_and_spacing
            ):
                return name

        normalized_destination = self._normalize_location_name(destination)
        for name in ROAD_GRAPH:
            if self._normalize_location_name(name) == normalized_destination:
                return name
        return None

    def move_to_location(self, destination: str) -> str:
        """Navigate to ``destination`` one road-network edge at a time."""
        matched_node = self._find_road_graph_node_name(destination)
        if matched_node is None:
            return (
                f"Không tìm thấy địa điểm '{destination}' "
                "trong danh sách đã biết."
            )

        try:
            path = shortest_path(
                self._current_location_name,
                matched_node,
            )
        except ValueError:
            return (
                f"Không tìm thấy đường đi tới {destination} "
                "trong mạng lưới đường hiện có."
            )

        for hop in path[1:]:
            hop_reply = super().move_to_location(hop)
            if not hop_reply.startswith(self._MOVE_SUCCESS_PREFIX):
                return (
                    f"Di chuyển thất bại tại chặng {hop} "
                    f"trên đường tới {destination}."
                )

        self._current_location_name = matched_node
        return (
            f"Đã đi qua {len(path) - 1} chặng "
            f"({' → '.join(path)}) và tới {destination}."
        )


def main():
    """Spin the road-network executor with multiple executor threads."""
    rclpy.init()
    node = RoadNetworkToolExecutorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
