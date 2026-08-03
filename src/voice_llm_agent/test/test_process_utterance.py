from voice_llm_agent.application.process_utterance import ProcessUtterance
from voice_llm_agent.domain.entities import (
    ConversationTurn,
    ToolCall,
    ToolResult,
)
from voice_llm_agent.domain.ports import LLMPort, RobotToolExecutorPort
from voice_llm_agent.domain.tool_schemas import TOOL_SCHEMAS


class FakeLLM(LLMPort):
    def __init__(
        self,
        responses: list[tuple[str, list[ToolCall]]],
    ) -> None:
        self._responses = list(responses)
        self.histories: list[list[ConversationTurn]] = []
        self.tools_seen: list[list[dict]] = []

    def chat(
        self,
        history: list[ConversationTurn],
        tools: list[dict],
    ) -> tuple[str, list[ToolCall]]:
        self.histories.append(list(history))
        self.tools_seen.append(tools)
        if not self._responses:
            raise AssertionError("LLM giả không còn phản hồi đã lập trình")
        return self._responses.pop(0)

    def vision(self, prompt: str, image_bytes: bytes) -> str:
        return "Mô tả ảnh giả lập."


class AlwaysToolCallingLLM(LLMPort):
    def __init__(self) -> None:
        self.chat_count = 0

    def chat(
        self,
        history: list[ConversationTurn],
        tools: list[dict],
    ) -> tuple[str, list[ToolCall]]:
        self.chat_count += 1
        tool_call = ToolCall(
            id=f"move-{self.chat_count}",
            name="move_to_location",
            arguments={"destination": "phòng thí nghiệm"},
        )
        return "", [tool_call]

    def vision(self, prompt: str, image_bytes: bytes) -> str:
        return "Mô tả ảnh giả lập."


class FakeRobotToolExecutor(RobotToolExecutorPort):
    def __init__(self, failing_tool: str | None = None) -> None:
        self.failing_tool = failing_tool
        self.calls: list[tuple[str, dict]] = []

    def _execute(self, name: str, arguments: dict) -> str:
        self.calls.append((name, arguments))
        if name == self.failing_tool:
            raise RuntimeError("lỗi executor giả lập")
        return f"Đã thực thi {name}"

    def robot_move_control(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
    ) -> str:
        return self._execute(
            "robot_move_control",
            {
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
                "duration": duration,
            },
        )

    def arm_transport_function(self, color: str, action: str) -> str:
        return self._execute(
            "arm_transport_function",
            {"color": color, "action": action},
        )

    def line_following(self, color: str) -> str:
        return self._execute("line_following", {"color": color})

    def move_to_location(self, destination: str) -> str:
        return self._execute(
            "move_to_location",
            {"destination": destination},
        )

    def describe_current_view(self, question: str) -> str:
        return self._execute(
            "describe_current_view",
            {"question": question},
        )

    def get_object_box_distance(self, user_query: str) -> str:
        return self._execute(
            "get_object_box_distance",
            {"user_query": user_query},
        )

    def object_track(self, box: str) -> str:
        return self._execute("object_track", {"box": box})

    def lidar_scan_detect(self, scan_detect: str) -> str:
        return self._execute(
            "lidar_scan_detect",
            {"scan_detect": scan_detect},
        )


class InspectableProcessUtterance(ProcessUtterance):
    def __init__(
        self,
        llm: LLMPort,
        executor: RobotToolExecutorPort,
    ) -> None:
        super().__init__(llm, executor)
        self.tool_results: list[ToolResult] = []

    def _execute_tool_call(self, tool_call: ToolCall) -> ToolResult:
        result = super()._execute_tool_call(tool_call)
        self.tool_results.append(result)
        return result


def test_returns_reply_directly_without_tool_call() -> None:
    llm = FakeLLM([("Xin chào bạn!", [])])
    executor = FakeRobotToolExecutor()

    reply = ProcessUtterance(llm, executor).handle("Xin chào")

    assert reply == "Xin chào bạn!"
    assert executor.calls == []
    assert [turn.role for turn in llm.histories[0]] == ["system", "user"]
    assert llm.histories[0][1].content == "Xin chào"
    assert llm.tools_seen == [TOOL_SCHEMAS]


def test_executes_valid_tool_then_returns_second_reply() -> None:
    arguments = {
        "linear_x": 0.5,
        "linear_y": 0.0,
        "angular_z": 0.2,
        "duration": 1.5,
    }
    tool_call = ToolCall(
        id="move-1",
        name="robot_move_control",
        arguments=arguments,
    )
    llm = FakeLLM(
        [
            ("", [tool_call]),
            ("Robot đã di chuyển xong.", []),
        ]
    )
    executor = FakeRobotToolExecutor()

    reply = ProcessUtterance(llm, executor).handle("Tiến lên")

    assert reply == "Robot đã di chuyển xong."
    assert executor.calls == [("robot_move_control", arguments)]
    second_history = llm.histories[1]
    assert [turn.role for turn in second_history] == [
        "system",
        "user",
        "assistant",
        "tool",
    ]
    assert second_history[2].tool_calls == [tool_call]
    assert second_history[3].tool_call_id == "move-1"
    assert second_history[3].content == "Đã thực thi robot_move_control"


def test_unknown_tool_becomes_failed_result_without_crashing() -> None:
    tool_call = ToolCall(
        id="unknown-1",
        name="unknown_tool",
        arguments={},
    )
    llm = FakeLLM(
        [
            ("", [tool_call]),
            ("Tôi không thể dùng công cụ đó.", []),
        ]
    )
    executor = FakeRobotToolExecutor()
    use_case = InspectableProcessUtterance(llm, executor)

    reply = use_case.handle("Dùng công cụ lạ")

    assert reply == "Tôi không thể dùng công cụ đó."
    assert executor.calls == []
    assert use_case.tool_results == [
        ToolResult(
            tool_call_id="unknown-1",
            name="unknown_tool",
            content="Không tìm thấy công cụ unknown_tool",
            success=False,
        )
    ]


def test_executor_exception_becomes_failed_result_without_crashing() -> None:
    tool_call = ToolCall(
        id="line-1",
        name="line_following",
        arguments={"color": "blue"},
    )
    llm = FakeLLM(
        [
            ("", [tool_call]),
            ("Không thể đi theo vạch lúc này.", []),
        ]
    )
    executor = FakeRobotToolExecutor(failing_tool="line_following")
    use_case = InspectableProcessUtterance(llm, executor)

    reply = use_case.handle("Đi theo vạch xanh")

    assert reply == "Không thể đi theo vạch lúc này."
    assert len(use_case.tool_results) == 1
    result = use_case.tool_results[0]
    assert result.success is False
    assert result.tool_call_id == "line-1"
    assert result.name == "line_following"
    assert "Lỗi khi thực thi line_following" in result.content
    assert "lỗi executor giả lập" in result.content


def test_limits_repeated_tool_calling_to_three_rounds() -> None:
    llm = AlwaysToolCallingLLM()
    executor = FakeRobotToolExecutor()

    reply = ProcessUtterance(llm, executor).handle("Tiếp tục gọi công cụ")

    assert reply == "Đã thực hiện xong."
    assert llm.chat_count == 3
    assert len(executor.calls) == 3
