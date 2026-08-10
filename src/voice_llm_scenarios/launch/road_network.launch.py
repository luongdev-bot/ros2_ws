"""Launch the warehouse voice scenario with graph-routed navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario_share = get_package_share_directory("voice_llm_scenarios")
    gazebo_share = get_package_share_directory("jetrover_gazebo")
    navigation_share = get_package_share_directory("navigation")

    world_path = os.path.join(gazebo_share, "worlds", "warehouse.sdf")
    map_path = os.path.join(scenario_share, "maps", "warehouse.yaml")
    locations_path = os.path.join(
        scenario_share,
        "config",
        "locations_warehouse.yaml",
    )

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
    piper_voice_path = DeclareLaunchArgument(
        "piper_voice_path",
        default_value=(
            "~/.local/share/piper-voices/"
            "vi_VN-vais1000-medium.onnx"
        ),
        description="Piper voice model path.",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "spawn_x": "0",
            "spawn_y": "0",
            "spawn_yaw": "0",
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "localization": "true",
            "map": map_path,
        }.items(),
    )

    initial_pose_message = (
        "{header: {frame_id: map}, pose: {pose: {"
        "position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, "
        "covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
    )
    publish_initial_pose = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    initial_pose_message,
                ],
                output="screen",
            )
        ],
    )

    road_network_tool_executor = Node(
        package="voice_llm_scenarios",
        executable="road_network_tool_executor",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "locations_yaml_path": locations_path,
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
            ollama_base_url,
            ollama_model,
            nav_timeout_s,
            user_utterance_topic,
            agent_reply_topic,
            whisper_model_size,
            whisper_language,
            whisper_device,
            whisper_compute_type,
            record_seconds,
            piper_voice_path,
            gazebo_launch,
            navigation_launch,
            publish_initial_pose,
            road_network_tool_executor,
            voice_loop_process,
        ]
    )
