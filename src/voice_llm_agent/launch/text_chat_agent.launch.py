"""Launch the tool executor and venv-based text chat GUI."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    python_executable = DeclareLaunchArgument(
        "python_executable",
        default_value=os.path.join(
            os.path.expanduser("~"),
            "voice_llm_env",
            "bin",
            "python3",
        ),
        description=(
            "Machine-specific Python interpreter with ROS 2 and voice "
            "dependencies available; override if its venv moves."
        ),
    )

    camera_topic = DeclareLaunchArgument(
        "camera_topic",
        default_value="/depth_cam/image",
        description="RGB camera stream used by the tool executor.",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Chassis velocity command topic.",
    )
    grasp_executor_node_name = DeclareLaunchArgument(
        "grasp_executor_node_name",
        default_value="grasp_executor",
        description="ROS node name of the grasp executor.",
    )
    line_follow_enable_service = DeclareLaunchArgument(
        "line_follow_enable_service",
        default_value="/line_follow/enable",
        description="Service used to enable line following.",
    )
    line_follow_configured_color = DeclareLaunchArgument(
        "line_follow_configured_color",
        default_value="black",
        description="Line color configured at startup.",
    )
    locations_yaml_path = DeclareLaunchArgument(
        "locations_yaml_path",
        default_value="",
        description="YAML file containing named navigation locations.",
    )
    ollama_base_url = DeclareLaunchArgument(
        "ollama_base_url",
        default_value="http://localhost:11434",
        description="Base URL of the Ollama service.",
    )
    ollama_model = DeclareLaunchArgument(
        "ollama_model",
        default_value="qwen2.5vl:3b",
        description="Ollama model used by the agent.",
    )
    nav_timeout_s = DeclareLaunchArgument(
        "nav_timeout_s",
        default_value="120.0",
        description="Navigation timeout in seconds.",
    )

    user_utterance_topic = DeclareLaunchArgument(
        "user_utterance_topic",
        default_value="/tool_executor/user_utterance",
        description="Absolute topic for typed user utterances.",
    )
    agent_reply_topic = DeclareLaunchArgument(
        "agent_reply_topic",
        default_value="/tool_executor/agent_reply",
        description=(
            "Absolute topic carrying text replies to display and speak."
        ),
    )
    piper_voice_path = DeclareLaunchArgument(
        "piper_voice_path",
        default_value=(
            "~/.local/share/piper-voices/"
            "vi_VN-vais1000-medium.onnx"
        ),
        description="Piper voice model path.",
    )

    tool_executor_node_action = Node(
        package="voice_llm_agent",
        executable="tool_executor",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "grasp_executor_node_name": LaunchConfiguration(
                    "grasp_executor_node_name"
                ),
                "line_follow_enable_service": LaunchConfiguration(
                    "line_follow_enable_service"
                ),
                "line_follow_configured_color": LaunchConfiguration(
                    "line_follow_configured_color"
                ),
                "locations_yaml_path": LaunchConfiguration(
                    "locations_yaml_path"
                ),
                "ollama_base_url": LaunchConfiguration("ollama_base_url"),
                "ollama_model": LaunchConfiguration("ollama_model"),
                "nav_timeout_s": LaunchConfiguration("nav_timeout_s"),
            }
        ],
    )

    # The invoking shell must source ROS 2 and this workspace overlay first;
    # ExecuteProcess inherits that PYTHONPATH for the venv interpreter.
    text_chat_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "voice_llm_agent.infrastructure.ros.text_chat_node",
            "--ros-args",
            "-p",
            [
                "user_utterance_topic:=",
                LaunchConfiguration("user_utterance_topic"),
            ],
            "-p",
            [
                "agent_reply_topic:=",
                LaunchConfiguration("agent_reply_topic"),
            ],
            "-p",
            [
                "piper_voice_path:=",
                LaunchConfiguration("piper_voice_path"),
            ],
        ],
        output="screen",
        additional_env={
            "LD_LIBRARY_PATH": (
                os.path.join(os.path.expanduser("~"), ".local", "lib")
                + ":"
                + os.environ.get("LD_LIBRARY_PATH", "")
            )
        },
    )

    return LaunchDescription(
        [
            python_executable,
            camera_topic,
            cmd_vel_topic,
            grasp_executor_node_name,
            line_follow_enable_service,
            line_follow_configured_color,
            locations_yaml_path,
            ollama_base_url,
            ollama_model,
            nav_timeout_s,
            user_utterance_topic,
            agent_reply_topic,
            piper_voice_path,
            tool_executor_node_action,
            text_chat_process,
        ]
    )
