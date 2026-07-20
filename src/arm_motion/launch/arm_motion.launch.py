"""Bring up the arm motion server, and optionally the Qt editor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('arm_motion')

    robot_config = DeclareLaunchArgument(
        'robot_config',
        default_value=PathJoinSubstitution([share, 'config', 'jetrover_arm.yaml']),
        description='Joint limits and servo scaling.',
    )
    task_config = DeclareLaunchArgument(
        'task_config',
        default_value=PathJoinSubstitution([share, 'config', 'task_motions.yaml']),
        description='Task -> action group bindings.',
    )
    library_dir = DeclareLaunchArgument(
        'library_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'ActionGroups'),
        description='Directory holding the .d6a action group files.',
    )
    prelude_motion = DeclareLaunchArgument(
        'prelude_motion',
        default_value='home',
        description="Action group the arm passes through before every motion; '' disables.",
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo clock.',
    )
    open_editor = DeclareLaunchArgument(
        'editor',
        default_value='false',
        description='Also launch the Qt action group editor.',
    )

    server = Node(
        package='arm_motion',
        executable='arm_motion_server',
        name='arm_motion_server',
        output='screen',
        parameters=[{
            'robot_config': LaunchConfiguration('robot_config'),
            'task_config': LaunchConfiguration('task_config'),
            'library_dir': LaunchConfiguration('library_dir'),
            'prelude_motion': LaunchConfiguration('prelude_motion'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    editor = Node(
        package='arm_motion',
        executable='arm_motion_editor',
        name='arm_motion_editor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('editor')),
        arguments=[
            '--robot-config', LaunchConfiguration('robot_config'),
            '--library-dir', LaunchConfiguration('library_dir'),
        ],
    )

    return LaunchDescription([
        robot_config,
        task_config,
        library_dir,
        prelude_motion,
        use_sim_time,
        open_editor,
        server,
        editor,
    ])
