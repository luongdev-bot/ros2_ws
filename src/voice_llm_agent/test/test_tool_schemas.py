import copy

from voice_llm_agent.domain.tool_schemas import (
    TOOL_SCHEMAS,
    with_location_enum,
)


EXPECTED_TOOL_NAMES = [
    "robot_move_control",
    "arm_transport_function",
    "line_following",
    "move_to_location",
    "describe_current_view",
    "get_object_box_distance",
    "object_track",
    "lidar_scan_detect",
]


def test_tool_schemas_contain_exactly_eight_valid_functions() -> None:
    assert len(TOOL_SCHEMAS) == 8
    assert [schema["function"]["name"] for schema in TOOL_SCHEMAS] == (
        EXPECTED_TOOL_NAMES
    )

    for schema in TOOL_SCHEMAS:
        assert schema["type"] == "function"
        function = schema["function"]
        assert function["name"]
        assert function["description"]

        parameters = function["parameters"]
        assert isinstance(parameters, dict)
        assert parameters["type"] == "object"
        assert parameters["properties"]


def test_arm_transport_destination_color_is_optional() -> None:
    arm_transport = next(
        schema
        for schema in TOOL_SCHEMAS
        if schema["function"]["name"] == "arm_transport_function"
    )
    parameters = arm_transport["function"]["parameters"]

    assert "destination_color" in parameters["properties"]
    assert "destination_color" not in parameters["required"]


def test_with_location_enum() -> None:
    original_tools = copy.deepcopy(TOOL_SCHEMAS)

    unconstrained_tools = with_location_enum(TOOL_SCHEMAS, [])
    assert unconstrained_tools == original_tools
    assert unconstrained_tools is not TOOL_SCHEMAS

    destinations = ["Kho hàng", "Nhà"]
    constrained_tools = with_location_enum(TOOL_SCHEMAS, destinations)
    move_to_location_index = 3
    destination_schema = constrained_tools[move_to_location_index][
        "function"
    ]["parameters"]["properties"]["destination"]

    assert destination_schema["enum"] == destinations
    assert "Kho hàng, Nhà" in destination_schema["description"]
    assert all(
        constrained == original
        for index, (constrained, original) in enumerate(
            zip(constrained_tools, original_tools)
        )
        if index != move_to_location_index
    )
    assert TOOL_SCHEMAS == original_tools
    assert (
        "enum"
        not in TOOL_SCHEMAS[move_to_location_index]["function"][
            "parameters"
        ]["properties"]["destination"]
    )
