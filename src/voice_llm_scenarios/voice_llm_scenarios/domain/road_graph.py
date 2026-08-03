"""Named road-network graph and shortest-path planning."""

from collections import deque


ROAD_GRAPH: dict[str, list[str]] = {
    "Điểm xuất phát": ["Khu trung tâm", "Góc tây nam"],
    "Khu trung tâm": [
        "Điểm xuất phát",
        "Khu vực kệ hàng",
        "Khu tiếp nhận",
    ],
    "Góc tây nam": ["Điểm xuất phát"],
    "Khu vực kệ hàng": ["Khu trung tâm"],
    "Khu tiếp nhận": ["Khu trung tâm"],
}


def shortest_path(start: str, goal: str) -> list[str]:
    """Return the BFS shortest path from ``start`` to ``goal``.

    The returned list includes both endpoints. Unknown endpoints and
    disconnected endpoint pairs raise :class:`ValueError`.
    """
    if start not in ROAD_GRAPH:
        raise ValueError(
            f"Điểm bắt đầu {start!r} không tồn tại trong đồ thị đường."
        )
    if goal not in ROAD_GRAPH:
        raise ValueError(
            f"Điểm đích {goal!r} không tồn tại trong đồ thị đường."
        )
    if start == goal:
        return [start]

    pending_paths = deque([[start]])
    visited = {start}

    while pending_paths:
        path = pending_paths.popleft()
        current = path[-1]

        for neighbor in ROAD_GRAPH[current]:
            if neighbor in visited:
                continue

            next_path = [*path, neighbor]
            if neighbor == goal:
                return next_path

            visited.add(neighbor)
            pending_paths.append(next_path)

    raise ValueError(
        f"Không có đường đi từ {start!r} tới {goal!r} trong đồ thị đường."
    )
