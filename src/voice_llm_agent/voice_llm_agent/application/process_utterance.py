"""Use case xử lý một câu nói và điều phối vòng lặp gọi công cụ."""

from ..domain.entities import ConversationTurn, ToolCall, ToolResult
from ..domain.ports import LLMPort, RobotToolExecutorPort
from ..domain.system_prompt import SYSTEM_PROMPT
from ..domain.tool_schemas import TOOL_SCHEMAS


class ProcessUtterance:
    """Xử lý một phát ngôn người dùng bằng LLM và các công cụ robot."""

    _MAX_TOOL_CALLING_ROUNDS = 3
    _DEFAULT_REPLY = "Đã thực hiện xong."

    def __init__(
        self,
        llm: LLMPort,
        executor: RobotToolExecutorPort,
        system_prompt: str = SYSTEM_PROMPT,
        tools: list[dict] = TOOL_SCHEMAS,
    ) -> None:
        self._llm = llm
        self._executor = executor
        self._system_prompt = system_prompt
        self._tools = tools

    def handle(self, user_text: str) -> str:
        """Xử lý văn bản người dùng và trả về câu trả lời cuối cùng."""
        history = [
            ConversationTurn(role="system", content=self._system_prompt),
            ConversationTurn(role="user", content=user_text),
        ]
        reply_text = ""

        for _ in range(self._MAX_TOOL_CALLING_ROUNDS):
            reply_text, tool_calls = self._llm.chat(history, self._tools)
            if not tool_calls:
                return reply_text or self._DEFAULT_REPLY

            tool_results = [
                self._execute_tool_call(tool_call)
                for tool_call in tool_calls
            ]
            history.append(
                ConversationTurn(
                    role="assistant",
                    content=reply_text,
                    tool_calls=tool_calls,
                )
            )
            history.extend(
                ConversationTurn(
                    role="tool",
                    content=result.content,
                    tool_call_id=result.tool_call_id,
                )
                for result in tool_results
            )

        return reply_text or self._DEFAULT_REPLY

    def _execute_tool_call(self, tool_call: ToolCall) -> ToolResult:
        """Thực thi một tool call và chuyển mọi lỗi thành ToolResult."""
        method = getattr(self._executor, tool_call.name, None)
        if method is None:
            return ToolResult(
                tool_call_id=tool_call.id,
                name=tool_call.name,
                content=f"Không tìm thấy công cụ {tool_call.name}",
                success=False,
            )

        try:
            content = method(**tool_call.arguments)
        except Exception as error:
            return ToolResult(
                tool_call_id=tool_call.id,
                name=tool_call.name,
                content=(
                    f"Lỗi khi thực thi {tool_call.name}: {error}"
                ),
                success=False,
            )

        return ToolResult(
            tool_call_id=tool_call.id,
            name=tool_call.name,
            content=content,
        )
