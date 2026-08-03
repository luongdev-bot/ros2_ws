"""Unit tests for the pure road-network domain logic."""

import pytest

from voice_llm_scenarios.domain.road_graph import shortest_path


def test_shortest_path_from_southwest_to_shelf_area() -> None:
    assert shortest_path("Góc tây nam", "Khu vực kệ hàng") == [
        "Góc tây nam",
        "Điểm xuất phát",
        "Khu trung tâm",
        "Khu vực kệ hàng",
    ]


def test_shortest_path_to_same_node() -> None:
    assert shortest_path("Khu trung tâm", "Khu trung tâm") == [
        "Khu trung tâm"
    ]


@pytest.mark.parametrize(
    ("start", "goal"),
    [
        ("Địa điểm không tồn tại", "Khu trung tâm"),
        ("Khu trung tâm", "Địa điểm không tồn tại"),
    ],
)
def test_shortest_path_rejects_unknown_node(start: str, goal: str) -> None:
    with pytest.raises(ValueError, match="không tồn tại"):
        shortest_path(start, goal)


def test_shortest_path_from_start_to_receiving_area() -> None:
    path = shortest_path("Điểm xuất phát", "Khu tiếp nhận")

    assert path == [
        "Điểm xuất phát",
        "Khu trung tâm",
        "Khu tiếp nhận",
    ]
    assert len(path) == 3
