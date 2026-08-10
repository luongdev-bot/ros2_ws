"""Focused initialization and reply-display tests for the text chat adapter."""

import tkinter as tk

import pytest

try:
    import piper  # noqa: F401
    import sounddevice  # noqa: F401
except (ImportError, OSError) as error:
    pytest.skip(
        f"Thiếu môi trường voice_llm_env: {error}",
        allow_module_level=True,
    )

try:
    _display_probe = tk.Tk()
    _display_probe.withdraw()
    _display_probe.destroy()
except tk.TclError as error:
    pytest.skip(
        f"Không có display cho tkinter: {error}",
        allow_module_level=True,
    )

import rclpy

from voice_llm_agent.infrastructure.ros.text_chat_node import TextChatNode


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def _destroy_node(node: TextChatNode) -> None:
    node._cancel_drain_callback()
    node.destroy_node()
    node._root.destroy()


def test_node_initializes_agent_topics(ros_context) -> None:
    node = TextChatNode()
    try:
        assert node._utterance_publisher.topic_name == (
            "/tool_executor/user_utterance"
        )
        assert node._agent_reply_subscription.topic_name == (
            "/tool_executor/agent_reply"
        )
    finally:
        _destroy_node(node)


def test_drain_replies_updates_chat(ros_context) -> None:
    node = TextChatNode()
    try:
        node._speak_reply = lambda _reply: None
        node.results.put("Đã hoàn thành lệnh.")
        node._drain_replies()

        chat = node._chat_text.get("1.0", "end")
        assert "Robot: Đã hoàn thành lệnh." in chat
    finally:
        _destroy_node(node)
