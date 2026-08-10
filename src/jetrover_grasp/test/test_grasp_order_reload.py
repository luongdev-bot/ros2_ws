"""Tests for runtime grasp-order parameter updates."""

from unittest.mock import Mock, patch

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from jetrover_grasp.infrastructure.ros.grasp_executor_node import (
    GraspExecutorNode,
)


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (["green", "red"], ("green", "red")),
        ("blue", ("blue",)),
        (None, GraspExecutorNode._BIN_COLORS),
    ],
)
def test_coerce_grasp_order(value, expected):
    assert GraspExecutorNode._coerce_grasp_order(value) == expected


@pytest.mark.parametrize(
    ("value", "expected", "warns"),
    [
        (" Blue ", "blue", False),
        ("purple", "", True),
        ("", "", False),
        (None, "", False),
    ],
)
def test_coerce_place_color_override(value, expected, warns):
    logger = Mock()

    result = GraspExecutorNode._coerce_place_color_override(
        value,
        set(GraspExecutorNode._BIN_COLORS),
        logger=logger,
    )

    assert result == expected
    if warns:
        logger.warning.assert_called_once_with(
            "invalid place_color_override parameter; placing in the "
            "object's own bin"
        )
    else:
        logger.warning.assert_not_called()


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_set_parameters_reloads_grasp_order(ros_context):
    node = GraspExecutorNode()
    try:
        results = node.set_parameters(
            [
                Parameter(
                    "grasp_order",
                    type_=Parameter.Type.STRING_ARRAY,
                    value=["blue"],
                )
            ]
        )

        assert results[0].successful
        assert node._grasp_order == ("blue",)
    finally:
        node.destroy_node()


def test_set_parameters_reloads_place_color_override(ros_context):
    node = GraspExecutorNode()
    try:
        results = node.set_parameters(
            [
                Parameter(
                    "place_color_override",
                    type_=Parameter.Type.STRING,
                    value="blue",
                )
            ]
        )

        assert results[0].successful
        assert node._place_color_override == "blue"
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    ("mobile_enabled", "cycle_method_name"),
    [
        (True, "_run_mobile_cycle"),
        (False, "_run_fixed_base_cycle"),
    ],
)
def test_run_cycle_snapshots_place_color_override(
    ros_context,
    mobile_enabled,
    cycle_method_name,
):
    node = GraspExecutorNode()
    target = Mock()
    try:
        with node._state_lock:
            node._latest_detections = [target]
            node._place_color_override = "blue"
        node._mobile_enabled = mobile_enabled

        def rank_and_overwrite_override(_detections, _allowed_colours):
            node._place_color_override = "yellow"
            return [target]

        with (
            patch(
                "jetrover_grasp.infrastructure.ros."
                "grasp_executor_node.rank_targets",
                side_effect=rank_and_overwrite_override,
            ),
            patch.object(
                node,
                cycle_method_name,
                return_value=True,
            ) as cycle_method,
            patch.object(node._base_driver, "stop"),
        ):
            node._run_cycle()

        cycle_method.assert_called_once_with([target], "blue")
    finally:
        node.destroy_node()


@pytest.mark.parametrize("cycle_success", [True, False])
def test_run_cycle_records_actual_outcome(ros_context, cycle_success):
    node = GraspExecutorNode()
    target = Mock()
    try:
        with node._state_lock:
            node._latest_detections = [target]
            node._cycle_running = True
        node._mobile_enabled = True

        with (
            patch(
                "jetrover_grasp.infrastructure.ros."
                "grasp_executor_node.rank_targets",
                return_value=[target],
            ),
            patch.object(
                node,
                "_run_mobile_cycle",
                return_value=cycle_success,
            ),
            patch.object(node._base_driver, "stop"),
        ):
            node._run_cycle()

        assert node._cycle_done_event.is_set()
        assert node._last_cycle_success is cycle_success
        assert not node._cycle_running
    finally:
        node.destroy_node()


def test_start_cycle_signals_failure_without_target(ros_context):
    node = GraspExecutorNode()
    try:
        with patch.object(node._base_driver, "stop"):
            assert node._start_cycle()
            assert node._cycle_done_event.wait(0.5)
            node._worker_thread.join(timeout=0.5)

        assert node._last_cycle_success is False
        assert not node._cycle_running
        assert not node._worker_thread.is_alive()
    finally:
        node.destroy_node()


def test_grasp_next_callback_reports_cycle_timeout(ros_context):
    node = GraspExecutorNode(
        parameter_overrides=[
            Parameter("cycle_completion_timeout_s", value=0.05)
        ]
    )
    try:
        with patch.object(node, "_start_cycle", return_value=True):
            response = node._grasp_next_callback(None, Trigger.Response())

        assert response.success is False
        assert response.message == "grasp cycle timed out"
    finally:
        node.destroy_node()
