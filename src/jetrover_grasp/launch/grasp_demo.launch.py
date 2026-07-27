"""Launch the closed-loop IK colour-sorting simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory("jetrover_gazebo")

    auto_grasp = DeclareLaunchArgument(
        "auto_grasp",
        default_value="false",
        description="Automatically start a cycle when a block is detected.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo_arm.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(
                gazebo_share,
                "worlds",
                "color_sort_world.sdf",
            ),
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
                "bin_red": [-0.135, -0.32, 0.02],
                "bin_green": [-0.055, -0.32, 0.02],
                "bin_blue": [0.025, -0.32, 0.02],
                "bin_yellow": [0.105, -0.32, 0.02],
            }
        ],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            auto_grasp,
            gazebo,
            color_pick,
            grasp_executor,
        ]
    )
