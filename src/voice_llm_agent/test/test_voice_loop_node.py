"""Focused initialization test for the ROS voice-loop adapter."""

import pytest

try:
    import faster_whisper  # noqa: F401
    import piper  # noqa: F401
    import sounddevice  # noqa: F401
except (ImportError, OSError) as error:
    pytest.skip(
        f"Thiếu môi trường voice_llm_env: {error}",
        allow_module_level=True,
    )

import rclpy

from voice_llm_agent.infrastructure.ros.voice_loop_node import VoiceLoopNode


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_initializes_agent_topics(ros_context) -> None:
    node = VoiceLoopNode()
    try:
        assert node._utterance_publisher.topic_name == (
            "/tool_executor/user_utterance"
        )
        assert node._agent_reply_subscription.topic_name == (
            "/tool_executor/agent_reply"
        )
    finally:
        node.destroy_node()
