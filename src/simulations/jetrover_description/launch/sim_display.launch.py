import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess


def generate_launch_description():
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    jetrover_description_package_path = get_package_share_directory('jetrover_description')
    urdf_path = os.path.join(jetrover_description_package_path, 'urdf/jetrover.xacro')
    rviz_config_file = os.path.join(jetrover_description_package_path, 'rviz/view.rviz')

    robot_description = Command(['xacro ', urdf_path])

    # Stand-in for joint_state_publisher_gui (not installed): publishes all
    # movable joints at 0.0 so robot_state_publisher can compute full TF.
    default_joint_state_publisher_node = Node(
        package='jetrover_description',
        executable='default_joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'frame_prefix': frame_prefix, 'use_sim_time': use_sim_time}],
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    delay_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        frame_prefix_arg,
        use_sim_time_arg,
        default_joint_state_publisher_node,
        robot_state_publisher_node,
        delay_rviz_node,
    ])
