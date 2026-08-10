"""Adapter Ollama dùng HTTP API và giả lập function-calling bằng prompt."""

import base64
import json
import uuid
import urllib.error
import urllib.request

from ...domain.entities import ConversationTurn, ToolCall
from ...domain.ports import LLMPort


class OllamaClient(LLMPort):
    """Gọi model Ollama local mà không cần thư viện HTTP bên thứ ba."""

    _OPTIONS = {
        "num_gpu": 24,
        "num_ctx": 2048,
        "temperature": 0,
    }

    def __init__(
        self,
        base_url: str = "http://localhost:11434",
        model: str = "qwen2.5vl:3b",
        timeout: float = 60.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def chat(
        self,
        history: list[ConversationTurn],
        tools: list[dict],
    ) -> tuple[str, list[ToolCall]]:
        """Trả lời hội thoại hoặc một tool call được mã hóa dưới dạng JSON."""
        messages = [self._turn_to_message(turn) for turn in history]
        valid_tool_names = self._valid_tool_names(tools)

        if tools:
            self._add_tool_instructions(messages, tools)

        content = self._post_chat(messages)
        parsed = self._parse_json_object(content)
        if (
            isinstance(parsed, dict)
            and isinstance(parsed.get("tool"), str)
            and parsed["tool"] in valid_tool_names
        ):
            arguments = parsed.get("arguments", {}) or {}
            if not isinstance(arguments, dict):
                return content.strip(), []

            return "", [
                ToolCall(
                    id=str(uuid.uuid4()),
                    name=parsed["tool"],
                    arguments=arguments,
                )
            ]

        return content.strip(), []

    def vision(self, prompt: str, image_bytes: bytes) -> str:
        """Mô tả ảnh bằng format ``images`` riêng của Ollama."""
        images = [base64.b64encode(image_bytes).decode("utf-8")]
        messages = [
            {
                "role": "user",
                "content": prompt,
                "images": images,
            }
        ]
        return self._post_chat(messages).strip()

    def _post_chat(self, messages: list[dict]) -> str:
        payload = {
            "model": self.model,
            "stream": False,
            "options": dict(self._OPTIONS),
            "messages": messages,
        }
        request = urllib.request.Request(
            f"{self.base_url}/api/chat",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(
                request,
                timeout=self.timeout,
            ) as response:
                response_data = json.loads(response.read().decode("utf-8"))
            content = response_data["message"]["content"]
            if not isinstance(content, str):
                raise TypeError("message.content không phải chuỗi")
            return content
        except urllib.error.HTTPError as error:
            try:
                detail = error.read().decode("utf-8", errors="replace")
            except Exception:
                detail = str(error)
            raise RuntimeError(
                f"Không gọi được LLM: HTTP {error.code}: {detail}"
            ) from error
        except (
            urllib.error.URLError,
            TimeoutError,
            OSError,
            json.JSONDecodeError,
            KeyError,
            TypeError,
        ) as error:
            raise RuntimeError(f"Không gọi được LLM: {error}") from error

    @staticmethod
    def _turn_to_message(turn: ConversationTurn) -> dict:
        if turn.role == "tool":
            return {
                "role": "user",
                "content": (
                    f"[Kết quả công cụ {turn.tool_call_id}]: {turn.content}"
                ),
            }

        content = turn.content
        if turn.role == "assistant" and turn.tool_calls:
            calls = [
                {
                    "id": call.id,
                    "tool": call.name,
                    "arguments": call.arguments,
                }
                for call in turn.tool_calls
            ]
            call_text = json.dumps(calls, ensure_ascii=False)
            content = "\n".join(
                part
                for part in (
                    content,
                    f"[Lệnh gọi công cụ trước đó]: {call_text}",
                )
                if part
            )

        supported_roles = {"system", "user", "assistant"}
        role = turn.role if turn.role in supported_roles else "user"
        return {"role": role, "content": content}

    @staticmethod
    def _valid_tool_names(tools: list[dict]) -> set[str]:
        names = set()
        for schema in tools:
            try:
                name = schema["function"]["name"]
            except (KeyError, TypeError):
                continue
            if isinstance(name, str):
                names.add(name)
        return names

    def _add_tool_instructions(
        self,
        messages: list[dict],
        tools: list[dict],
    ) -> None:
        tool_lines = []
        for schema in tools:
            function = schema.get("function", {})
            name = function.get("name", "")
            description = function.get("description", "")
            parameters = json.dumps(
                function.get("parameters", {}),
                ensure_ascii=False,
                separators=(",", ":"),
            )
            tool_lines.append(
                f"- {name}: {description}\n  Tham số JSON: {parameters}"
            )

        instructions = "\n".join(
            [
                "CÔNG CỤ CÓ THỂ DÙNG:",
                *tool_lines,
                "",
                "QUY TẮC GỌI CÔNG CỤ BẮT BUỘC:",
                (
                    "Khi người dùng yêu cầu robot thực hiện một hành động "
                    "phù hợp với công cụ ở trên, phải gọi công cụ; không được "
                    "tự nhận rằng hành động đã hoàn thành."
                ),
                (
                    "Khi người dùng hỏi robot/camera hiện đang thấy gì, vật "
                    "thể có xuất hiện hay khoảng cách tới vật thể, không được "
                    "tự trả lời hoặc nói rằng chưa có thông tin hình ảnh; phải "
                    "gọi describe_current_view hoặc get_object_box_distance "
                    "tùy câu hỏi."
                ),
                (
                    "Nếu lịch sử đã có dòng [Kết quả công cụ ...] cho hành "
                    "động hiện tại, không gọi lại công cụ mà hãy trả lời ngắn "
                    "gọn dựa trên kết quả đó."
                ),
                (
                    "Nếu cần gọi một công cụ để thực hiện yêu cầu, CHỈ trả "
                    "lời DUY NHẤT một dòng JSON hợp lệ theo đúng mẫu sau, "
                    "không kèm bất kỳ chữ nào khác: "
                    '{"tool": "<tên_công_cụ>", "arguments": {...}}. '
                    "Nếu không cần công cụ, chỉ trả lời bình thường bằng "
                    "tiếng Việt, không có JSON."
                ),
                (
                    "Ví dụ cho lệnh đi thẳng 2 giây với vận tốc 0.2: "
                    '{"tool":"robot_move_control","arguments":'
                    '{"linear_x":0.2,"linear_y":0.0,"angular_z":0.0,'
                    '"duration":2.0}}'
                ),
                (
                    "Ví dụ cho lệnh gắp khối đỏ bỏ vào hộp xanh: "
                    '{"tool":"arm_transport_function","arguments":'
                    '{"color":"red","action":"pick",'
                    '"destination_color":"blue"}}'
                ),
            ]
        )

        system_indexes = [
            index
            for index, message in enumerate(messages)
            if message["role"] == "system"
        ]
        if system_indexes:
            index = system_indexes[-1]
            messages[index]["content"] = (
                f"{messages[index]['content'].rstrip()}\n\n{instructions}"
            )
        else:
            messages.insert(
                0,
                {"role": "system", "content": instructions},
            )

    @staticmethod
    def _parse_json_object(content: str) -> dict | None:
        decoder = json.JSONDecoder()
        for index, character in enumerate(content):
            if character != "{":
                continue
            try:
                parsed, _ = decoder.raw_decode(content[index:])
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                return parsed
        return None
