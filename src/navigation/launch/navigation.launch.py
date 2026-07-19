import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    # Nav2 for the JetRover Gazebo simulation. Designed to run ALONGSIDE SLAM
    # (slam_toolbox in 2D or RTAB-Map in 3D), which already provides the
    # map->odom transform, so AMCL/map_server are NOT started. To instead
    # navigate on a saved static map, pass  localization:=true map:=<file.yaml>.
    nav_pkg = get_package_share_directory('navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    autostart = LaunchConfiguration('autostart', default='true').perform(context)
    localization = LaunchConfiguration('localization', default='false').perform(context)
    map_yaml = LaunchConfiguration('map', default='').perform(context)
    use_rviz = LaunchConfiguration('use_rviz', default='true').perform(context)
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav_pkg, 'config', 'nav2_params.yaml')).perform(context)
    controller_param = os.path.join(nav_pkg, 'config', 'nav2_controller_rpp.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('autostart', default_value=autostart),
        DeclareLaunchArgument('localization', default_value=localization,
                              description='Start map_server+amcl on a static map '
                                          '(false when running with live SLAM)'),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz),
        DeclareLaunchArgument('params_file', default_value=params_file),
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    stdout_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_file, {'autostart': autostart == 'true'}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
        output='screen',
    )

    nav_nodes = [
        # Both files: nav2_params.yaml carries local_costmap (which the
        # controller server needs), nav2_controller_rpp.yaml comes last so its
        # RPP/controller settings win.
        ComposableNode(
            package='nav2_controller', plugin='nav2_controller::ControllerServer',
            name='controller_server', parameters=[params_file, controller_param],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        ComposableNode(
            package='nav2_smoother', plugin='nav2_smoother::SmootherServer',
            name='smoother_server', parameters=[params_file], remappings=remappings),
        ComposableNode(
            package='nav2_planner', plugin='nav2_planner::PlannerServer',
            name='planner_server', parameters=[params_file], remappings=remappings),
        ComposableNode(
            package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
            name='behavior_server', parameters=[params_file], remappings=remappings),
        ComposableNode(
            package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator', parameters=[params_file], remappings=remappings),
        ComposableNode(
            package='nav2_waypoint_follower', plugin='nav2_waypoint_follower::WaypointFollower',
            name='waypoint_follower', parameters=[params_file], remappings=remappings),
        ComposableNode(
            package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother', parameters=[params_file],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', 'cmd_vel')]),
        ComposableNode(
            package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[{'use_sim_time': use_sim_time == 'true', 'autostart': autostart == 'true',
                         'node_names': ['controller_server', 'smoother_server', 'planner_server',
                                        'behavior_server', 'bt_navigator', 'waypoint_follower',
                                        'velocity_smoother']}]),
    ]

    # Optional localization on a static map (only when NOT using live SLAM).
    loc_nodes = [
        ComposableNode(
            package='nav2_map_server', plugin='nav2_map_server::MapServer',
            name='map_server',
            parameters=[params_file, {'use_sim_time': use_sim_time == 'true',
                                      'yaml_filename': map_yaml}],
            remappings=remappings),
        ComposableNode(
            package='nav2_amcl', plugin='nav2_amcl::AmclNode',
            name='amcl', parameters=[params_file, {'use_sim_time': use_sim_time == 'true'}],
            remappings=remappings),
        ComposableNode(
            package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': use_sim_time == 'true', 'autostart': autostart == 'true',
                         'node_names': ['map_server', 'amcl']}]),
    ]

    descriptions = nav_nodes
    if localization == 'true':
        descriptions = loc_nodes + nav_nodes

    load_nodes = LoadComposableNodes(target_container='nav2_container',
                                     composable_node_descriptions=descriptions)

    # RViz: prefer nav2_bringup's default view (has the Nav2 Goal tool); fall
    # back to the slam view if nav2_bringup isn't installed.
    try:
        rviz_config = os.path.join(get_package_share_directory('nav2_bringup'),
                                   'rviz', 'nav2_default_view.rviz')
    except Exception:
        rviz_config = os.path.join(get_package_share_directory('slam'), 'rviz', 'slam.rviz')

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time == 'true'}],
        condition=IfCondition(use_rviz))

    return args + [stdout_envvar, GroupAction([container, load_nodes]), rviz_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
