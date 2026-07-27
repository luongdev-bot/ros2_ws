"""Unit tests for pure colour-target selection."""

from jetrover_grasp.application.target_selection import (
    DetectedBlock,
    rank_targets,
    select_target,
)


def _block(color, area, u=0.0, v=0.0):
    return DetectedBlock(color=color, u=u, v=v, area=area)


def test_picks_largest_allowed_detection():
    small_blue = _block("blue", 10.0)
    large_green = _block("green", 30.0)
    medium_blue = _block("blue", 20.0)

    selected = select_target(
        [small_blue, large_green, medium_blue],
        {"blue", "green"},
    )

    assert selected is large_green


def test_ignores_larger_disallowed_detection():
    allowed = _block("yellow", 12.0)
    disallowed = _block("red", 100.0)

    assert select_target([disallowed, allowed], {"yellow"}) is allowed


def test_returns_none_for_empty_or_none_allowed():
    assert select_target([], {"blue"}) is None
    assert select_target([_block("red", 10.0)], {"blue"}) is None


def test_equal_area_tie_keeps_first_detection():
    first = _block("blue", 20.0, u=10.0)
    second = _block("green", 20.0, u=30.0)

    assert select_target([first, second], {"blue", "green"}) is first


def test_rank_targets_orders_all_allowed_by_descending_area():
    small = _block("blue", 10.0)
    large = _block("green", 30.0)
    medium = _block("yellow", 20.0)

    assert rank_targets(
        [small, large, medium],
        {"blue", "green", "yellow"},
    ) == [large, medium, small]


def test_rank_targets_filters_disallowed_detections():
    allowed = _block("blue", 10.0)
    disallowed = _block("red", 100.0)

    assert rank_targets([disallowed, allowed], {"blue"}) == [allowed]


def test_rank_targets_returns_empty_for_no_matches():
    assert rank_targets([_block("red", 10.0)], {"blue"}) == []
