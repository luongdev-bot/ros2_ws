"""Launch the warehouse navigation-and-transport voice scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
        }.items(),
    )

    return LaunchDescription(
        [
            gazebo_launch,
            navigation_launch,
            publish_initial_pose,
            voice_agent_launch,
        ]
    )
