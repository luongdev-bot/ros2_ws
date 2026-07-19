import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch_ros.actions import Node


def launch_setup(context):
    # 2D SLAM (slam_toolbox) for the JetRover Gazebo simulation.
    # Assumes gazebo.launch.py is already running (it provides /scan, /odom,
    # TF odom->base_footprint and the robot_state_publisher static TFs).
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    scan_topic = LaunchConfiguration('scan_topic', default='/scan').perform(context)
    use_rviz = LaunchConfiguration('use_rviz', default='true').perform(context)

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value=scan_topic)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value=use_rviz)

    slam_package_path = get_package_share_directory('slam')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/slam_base.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_topic': scan_topic,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(slam_package_path, 'rviz/slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        condition=IfCondition(use_rviz),
    )

    return [
        use_sim_time_arg,
        scan_topic_arg,
        use_rviz_arg,
        slam_launch,
        TimerAction(period=3.0, actions=[rviz_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
