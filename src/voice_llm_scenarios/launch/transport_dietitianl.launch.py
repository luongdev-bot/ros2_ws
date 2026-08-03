"""Launch the warehouse grasp-and-delivery voice scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scenario_share = get_package_share_directory("voice_llm_scenarios")
    gazebo_share = get_package_share_directory("jetrover_gazebo")
    navigation_share = get_package_share_directory("navigation")
    voice_agent_share = get_package_share_directory("voice_llm_agent")

    world_path = os.path.join(gazebo_share, "worlds", "warehouse.sdf")
    map_path = os.path.join(scenario_share, "maps", "warehouse.yaml")
    locations_path = os.path.join(
        scenario_share,
        "config",
        "locations_warehouse.yaml",
    )

    auto_grasp = DeclareLaunchArgument(
        "auto_grasp",
        default_value="false",
        description="Automatically start a cycle when a block is detected.",
    )
    mobile_enabled = DeclareLaunchArgument(
        "mobile_enabled",
        default_value="true",
        description="Drive the mecanum base between blocks and bins.",
    )
    show_camera = DeclareLaunchArgument(
        "show_camera",
        default_value="true",
        description="Show colour detections in rqt_image_view.",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo_arm.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "world_name": "warehouse",
            "spawn_x": "0",
            "spawn_y": "0",
            "spawn_yaw": "0",
        }.items(),
    )

    color_pick = Node(
        package="arm_perception",
        executable="color_pick",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                # Keep this node as the detection publisher; grasp_executor
                # owns all motion in the closed-loop demo.
                "start_enabled": False,
            }
        ],
    )

    grasp_executor = Node(
        package="jetrover_grasp",
        executable="grasp_executor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "auto_grasp": ParameterValue(
                    LaunchConfiguration("auto_grasp"),
                    value_type=bool,
                ),
                "mobile_enabled": ParameterValue(
                    LaunchConfiguration("mobile_enabled"),
                    value_type=bool,
                ),
                "bin_red": [-0.135, -0.32, 0.02],
                "bin_green": [-0.055, -0.32, 0.02],
                "bin_blue": [0.025, -0.32, 0.02],
                "bin_yellow": [0.105, -0.32, 0.02],
            }
        ],
    )

    camera_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/color_pick/debug_image"],
        condition=IfCondition(LaunchConfiguration("show_camera")),
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

    # AMCL needs an initial map-frame estimate. Delay the one-shot publisher
    # until the localization lifecycle manager has had time to activate AMCL;
    # ros2 topic pub will also wait for a matching subscription if startup is
    # slower than the nominal delay.
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

    voice_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                voice_agent_share,
                "launch",
                "voice_agent.launch.py",
            )
        ),
        launch_arguments={
            "locations_yaml_path": locations_path,
            "grasp_executor_node_name": "grasp_executor",
        }.items(),
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            auto_grasp,
            mobile_enabled,
            show_camera,
            gazebo_launch,
            color_pick,
            grasp_executor,
            camera_view,
            navigation_launch,
            publish_initial_pose,
            voice_agent_launch,
        ]
    )
