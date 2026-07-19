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
    # 3D SLAM (RTAB-Map, RGBD + laser) for the JetRover Gazebo simulation.
    # Assumes gazebo.launch.py is already running (it provides /scan, /odom,
    # the depth camera topics and TF).
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    qos = LaunchConfiguration('qos', default='1').perform(context)
    use_rviz = LaunchConfiguration('use_rviz', default='true').perform(context)
    use_gpu = LaunchConfiguration('use_gpu', default='false').perform(context)

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    qos_arg = DeclareLaunchArgument('qos', default_value=qos)
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value=use_rviz)
    use_gpu_arg = DeclareLaunchArgument('use_gpu', default_value=use_gpu)

    slam_package_path = get_package_share_directory('slam')

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'qos': qos,
            'use_gpu': use_gpu,
        }.items(),
    )

    rviz_config = os.path.join(slam_package_path, 'rviz/rtabmap.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        condition=IfCondition(use_rviz),
    )

    return [
        use_sim_time_arg,
        qos_arg,
        use_rviz_arg,
        rtabmap_launch,
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
