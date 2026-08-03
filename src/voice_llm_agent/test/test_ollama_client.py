"""Integration tests chạy trực tiếp với Ollama local."""

import urllib.error
import urllib.request

import pytest

from voice_llm_agent.domain.entities import ConversationTurn
from voice_llm_agent.domain.tool_schemas import TOOL_SCHEMAS
from voice_llm_agent.infrastructure.llm.ollama_client import OllamaClient


def _ollama_is_running() -> bool:
    try:
        with urllib.request.urlopen(
            "http://localhost:11434/api/version",
            timeout=2,
        ):
            return True
    except (urllib.error.URLError, TimeoutError, OSError):
        return False


pytestmark = pytest.mark.skipif(
    not _ollama_is_running(),
    reason="Ollama không chạy trên máy này",
)


def test_chat_without_tools_returns_text() -> None:
    client = OllamaClient()
    reply, tool_calls = client.chat(
        [
            ConversationTurn(
                role="user",
                content="Một cộng một bằng bao nhiêu? Trả lời thật ngắn.",
            )
        ],
        tools=[],
    )

    assert reply
    assert tool_calls == []


def test_chat_emits_robot_move_tool_call() -> None:
    client = OllamaClient()
    diagnostics = []

    for _ in range(3):
        reply, tool_calls = client.chat(
            [
                ConversationTurn(
                    role="user",
                    content=(
                        "Hãy di chuyển robot tới trước trong 2 giây với "
                        "vận tốc 0.2."
                    ),
                )
            ],
            tools=TOOL_SCHEMAS,
        )
        diagnostics.append((reply, tool_calls))
        if len(tool_calls) == 1 and (
            tool_calls[0].name == "robot_move_control"
        ):
            break
    else:
        pytest.fail(
            "Model không tạo đúng robot_move_control sau 3 lần thử: "
            f"{diagnostics!r}"
        )

    assert len(tool_calls) == 1
    assert tool_calls[0].name == "robot_move_control"
