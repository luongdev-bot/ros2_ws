"""Launch the tool executor and venv-based microphone voice loop."""

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
        description="Absolute topic for recognized user utterances.",
    )
    agent_reply_topic = DeclareLaunchArgument(
        "agent_reply_topic",
        default_value="/tool_executor/agent_reply",
        description="Absolute topic carrying text replies to speak.",
    )
    whisper_model_size = DeclareLaunchArgument(
        "whisper_model_size",
        default_value="small",
        description="faster-whisper model size.",
    )
    whisper_language = DeclareLaunchArgument(
        "whisper_language",
        default_value="vi",
        description="Language code passed to faster-whisper.",
    )
    whisper_initial_prompt = DeclareLaunchArgument(
        "whisper_initial_prompt",
        default_value="Đây là một đoạn hội thoại tiếng Việt với robot.",
        description=(
            "Câu mồi ngữ cảnh tiếng Việt giúp Whisper nhận dạng đúng ngôn "
            "ngữ hơn, có thể chỉnh cho phù hợp với chủ đề hội thoại thực tế."
        ),
    )
    whisper_device = DeclareLaunchArgument(
        "whisper_device",
        default_value="cpu",
        description="Device used for faster-whisper inference.",
    )
    whisper_compute_type = DeclareLaunchArgument(
        "whisper_compute_type",
        default_value="int8",
        description="Compute type used by faster-whisper.",
    )
    record_seconds = DeclareLaunchArgument(
        "record_seconds",
        default_value="5.0",
        description="Microphone recording duration per utterance.",
    )
    silence_rms_threshold = DeclareLaunchArgument(
        "silence_rms_threshold",
        default_value="0.05",
        description=(
            "Ngưỡng RMS âm thanh; dưới ngưỡng này coi là im lặng, bỏ qua "
            "không gọi Whisper (tránh Whisper 'bịa' câu khi ghi phải im "
            "lặng/nhiễu nền). Giá trị khởi đầu, cần tự canh chỉnh theo "
            "phòng/mic thật — xem log INFO 'RMS đo được' khi chạy để biết "
            "mức tiếng ồn nền thực tế và chỉnh ngưỡng cho phù hợp (nên đặt "
            "cao hơn RMS tiếng ồn nền khoảng 1.5-2 lần)."
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
    voice_loop_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "voice_llm_agent.infrastructure.ros.voice_loop_node",
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
                "whisper_model_size:=",
                LaunchConfiguration("whisper_model_size"),
            ],
            "-p",
            [
                "whisper_language:=",
                LaunchConfiguration("whisper_language"),
            ],
            "-p",
            [
                "whisper_initial_prompt:=",
                LaunchConfiguration("whisper_initial_prompt"),
            ],
            "-p",
            [
                "whisper_device:=",
                LaunchConfiguration("whisper_device"),
            ],
            "-p",
            [
                "whisper_compute_type:=",
                LaunchConfiguration("whisper_compute_type"),
            ],
            "-p",
            [
                "record_seconds:=",
                LaunchConfiguration("record_seconds"),
            ],
            "-p",
            [
                "silence_rms_threshold:=",
                LaunchConfiguration("silence_rms_threshold"),
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
            whisper_model_size,
            whisper_language,
            whisper_initial_prompt,
            whisper_device,
            whisper_compute_type,
            record_seconds,
            silence_rms_threshold,
            piper_voice_path,
            tool_executor_node_action,
            voice_loop_process,
        ]
    )
