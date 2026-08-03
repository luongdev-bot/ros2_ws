"""Lược đồ công cụ theo định dạng OpenAI function-calling."""

import copy

TOOL_SCHEMAS: list[dict] = [
    {
        "type": "function",
        "function": {
            "name": "robot_move_control",
            "description": (
                "Di chuyển robot với vận tốc dài và vận tốc góc trong một "
                "khoảng thời gian tính bằng giây."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "linear_x": {
                        "type": "number",
                        "description": "Vận tốc dài theo trục x.",
                    },
                    "linear_y": {
                        "type": "number",
                        "description": "Vận tốc dài theo trục y.",
                    },
                    "angular_z": {
                        "type": "number",
                        "description": "Vận tốc góc quanh trục z.",
                    },
                    "duration": {
                        "type": "number",
                        "description": "Thời gian di chuyển, tính bằng giây.",
                    },
                },
                "required": [
                    "linear_x",
                    "linear_y",
                    "angular_z",
                    "duration",
                ],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "arm_transport_function",
            "description": "Gắp hoặc đặt vật thể theo màu bằng tay máy.",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {
                        "type": "string",
                        "enum": ["red", "green", "blue", "yellow"],
                        "description": "Màu của vật thể cần thao tác.",
                    },
                    "action": {
                        "type": "string",
                        "enum": ["pick", "place"],
                        "description": "Thao tác gắp hoặc đặt vật thể.",
                    },
                },
                "required": ["color", "action"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "line_following",
            "description": "Cho robot đi theo vạch kẻ có màu được chỉ định.",
            "parameters": {
                "type": "object",
                "properties": {
                    "color": {
                        "type": "string",
                        "enum": [
                            "red",
                            "green",
                            "blue",
                            "black",
                            "yellow",
                        ],
                        "description": "Màu của vạch kẻ cần đi theo.",
                    },
                },
                "required": ["color"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "move_to_location",
            "description": (
                "Di chuyển robot đến một địa điểm đã được đặt tên trước."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": "Tên địa điểm đích.",
                    },
                },
                "required": ["destination"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "describe_current_view",
            "description": (
                "Mô tả những gì camera của robot đang thấy và trả lời một "
                "câu hỏi cụ thể về khung cảnh."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "question": {
                        "type": "string",
                        "description": "Câu hỏi cần trả lời về hình ảnh.",
                    },
                },
                "required": ["question"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_object_box_distance",
            "description": (
                "Dùng camera độ sâu để xác định khoảng cách tới một hoặc "
                "nhiều vật thể được hỏi tới."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "user_query": {
                        "type": "string",
                        "description": (
                            "Yêu cầu về vật thể cần đo khoảng cách."
                        ),
                    },
                },
                "required": ["user_query"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "object_track",
            "description": (
                "Theo dõi vật thể theo hộp bao đã biết. Chức năng này hiện "
                "chưa được hỗ trợ trong bản mô phỏng."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "box": {
                        "type": "string",
                        "description": "Hộp bao của vật thể cần theo dõi.",
                    },
                },
                "required": ["box"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "lidar_scan_detect",
            "description": (
                "Phát hiện vật cản bằng lidar khi robot đang đi theo vạch. "
                "Chức năng này hiện chưa được hỗ trợ đầy đủ trong bản "
                "mô phỏng."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "scan_detect": {
                        "type": "string",
                        "description": "Yêu cầu phát hiện vật cản bằng lidar.",
                    },
                },
                "required": ["scan_detect"],
            },
        },
    },
]


def with_location_enum(
    tools: list[dict],
    destinations: list[str],
) -> list[dict]:
    """Trả về bản sao tools với destination bị ràng buộc nếu có địa điểm."""
    constrained_tools = copy.deepcopy(tools)
    if not destinations:
        return constrained_tools

    for entry in constrained_tools:
        function = entry.get("function", {})
        if function.get("name") != "move_to_location":
            continue

        destination = function["parameters"]["properties"]["destination"]
        destination["enum"] = list(destinations)
        destination["description"] += (
            " Phải chọn CHÍNH XÁC một trong các tên sau (giữ nguyên dấu, "
            "chữ hoa/thường, dấu cách): "
            + ", ".join(destinations)
        )
        break

    return constrained_tools
