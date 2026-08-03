"""Các cổng do domain sở hữu và tầng infrastructure sẽ triển khai."""

from abc import ABC, abstractmethod

from .entities import ConversationTurn, ToolCall


class LLMPort(ABC):
    """Cổng giao tiếp với mô hình ngôn ngữ và mô hình thị giác."""

    @abstractmethod
    def chat(
        self,
        history: list[ConversationTurn],
        tools: list[dict],
    ) -> tuple[str, list[ToolCall]]:
        """Trả về (reply_text, tool_calls).

        ``reply_text`` có thể rỗng nếu model chỉ gọi tool.
        """

    @abstractmethod
    def vision(self, prompt: str, image_bytes: bytes) -> str:
        """Trả về mô tả bằng văn bản cho một ảnh JPEG/PNG đã encode."""


class ASRPort(ABC):
    """Cổng nhận dạng tiếng nói."""

    @abstractmethod
    def listen(self) -> str:
        """Ghi âm từ mic và trả về văn bản tiếng Việt đã nhận dạng."""


class TTSPort(ABC):
    """Cổng tổng hợp tiếng nói."""

    @abstractmethod
    def speak(self, text: str) -> None:
        """Đọc to văn bản qua loa."""


class RobotToolExecutorPort(ABC):
    """Cổng thực thi các công cụ điều khiển và cảm nhận của robot."""

    @abstractmethod
    def robot_move_control(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration: float,
    ) -> str:
        ...

    @abstractmethod
    def arm_transport_function(self, color: str, action: str) -> str:
        ...

    @abstractmethod
    def line_following(self, color: str) -> str:
        ...

    @abstractmethod
    def move_to_location(self, destination: str) -> str:
        ...

    @abstractmethod
    def describe_current_view(self, question: str) -> str:
        ...

    @abstractmethod
    def get_object_box_distance(self, user_query: str) -> str:
        ...

    @abstractmethod
    def object_track(self, box: str) -> str:
        ...

    @abstractmethod
    def lidar_scan_detect(self, scan_detect: str) -> str:
        ...
