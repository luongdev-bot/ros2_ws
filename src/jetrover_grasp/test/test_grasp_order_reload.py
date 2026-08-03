"""Tests for runtime grasp-order parameter updates."""

import pytest
import rclpy
from rclpy.parameter import Parameter

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
