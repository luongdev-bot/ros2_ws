"""Các thực thể thuần Python của miền hội thoại và gọi công cụ."""

from dataclasses import dataclass


@dataclass
class ToolCall:
    """Một yêu cầu gọi công cụ do mô hình ngôn ngữ tạo ra."""

    id: str
    name: str
    arguments: dict


@dataclass
class ToolResult:
    """Kết quả thực thi một yêu cầu gọi công cụ."""

    tool_call_id: str
    name: str
    content: str
    success: bool = True


@dataclass
class ConversationTurn:
    """Một lượt trong lịch sử hội thoại gửi tới mô hình ngôn ngữ."""

    role: str
    content: str
    tool_calls: list[ToolCall] | None = None
    tool_call_id: str | None = None
