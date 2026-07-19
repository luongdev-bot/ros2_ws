"""Gazebo + robot with a controllable arm, WITHOUT MoveIt.

This is gazebo_moveit.launch.py minus move_group and the MoveIt RViz, which
together add minutes to startup and open a second RViz window.

3D SLAM (RTAB-Map) needs the depth camera, which is mounted on the arm, so it
needs everything here:
  * jetrover.urdf.xacro with use_gazebo:=true -> gz_ros2_control
  * joint_state_broadcaster + arm_controller  -> the arm can be posed
    (see scripts/pose_arm_camera.sh; at joint 0 the camera points straight up)
  * the depth_cam_frame -> jetrover/link4/depth_camera static TF, without which
    RTAB-Map drops every RGBD frame ("TF of received image ... is not set")

It does NOT need motion planning, so move_group stays out. Use
gazebo_moveit.launch.py when you actually want MoveIt.
"""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    jetrover_description_share = get_package_share_directory('jetrover_description')
    jetrover_gazebo_share = get_package_share_directory('jetrover_gazebo')
    jetrover_moveit_config_share = get_package_share_directory('jetrover_moveit_config')

    resource_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(jetrover_description_share) + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # Gazebo loads the ros2_control system plugin (libign_ros2_control-system.so)
    # by filename only, and it lives in the ROS prefix, not on Gazebo's default
    # search path. Without this the plugin is skipped SILENTLY - no error is
    # printed, controller_manager never starts, the arm has nothing holding it
    # and the whole robot collapses in the world.
    plugin_path_env = SetEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix('gz_ros2_control'), 'lib') + ':' +
        os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
    )

    # Hybrid-graphics (PRIME on-demand): keep Gazebo's renderer and the GPU
    # lidar/rgbd sensors on the NVIDIA GPU instead of the Intel iGPU.
    nv_offload_env = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nv_glx_env = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    nv_vk_env = SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(jetrover_gazebo_share, 'worlds', 'jetrover_world.sdf'),
    )
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='jetrover')

    urdf_path = os.path.join(jetrover_moveit_config_share, 'config', 'jetrover.urdf.xacro')
    initial_positions_path = os.path.join(
        jetrover_moveit_config_share, 'config', 'initial_positions.yaml')
    robot_description = Command([
        'xacro ', urdf_path,
        ' use_gazebo:=true',
        ' initial_positions_file:=', initial_positions_path,
    ], on_stderr='warn')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items(),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }],
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('robot_name'),
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
    )

    # Gazebo's rgbd_camera reports header.frame_id "jetrover/link4/depth_camera"
    # after URDF->SDF conversion; depth_cam_frame is co-located with it, so an
    # identity transform lets RTAB-Map (and MoveIt's octomap) resolve the TF.
    depth_cam_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'depth_cam_frame',
                   [LaunchConfiguration('robot_name'), '/link4/depth_camera']],
        parameters=[{'use_sim_time': True}],
    )

    # Same problem as the camera, for the laser: after URDF->SDF conversion the
    # lidar sensor ends up scoped to its parent link, so /scan arrives with
    # header.frame_id "jetrover/base_footprint/lidar" while TF only knows
    # "lidar_frame". Without this bridge both slam_toolbox and RTAB-Map reject
    # every scan ("Could not convert laser scan msg!").
    # Built from robot_name (not hardcoded) so renaming the model does not
    # silently break the bridge.
    robot_name = LaunchConfiguration('robot_name')
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_frame', [robot_name, '/base_footprint/lidar']],
        parameters=[{'use_sim_time': True}],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(jetrover_gazebo_share, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )
    # controller_manager lives inside the Gazebo process (gz_ros2_control), so
    # only start the spawners once the entity has actually been created.
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        resource_path_env,
        plugin_path_env,
        nv_offload_env,
        nv_glx_env,
        nv_vk_env,
        SetParameter(name='use_sim_time', value=True),
        world_arg,
        robot_name_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        depth_cam_frame_bridge,
        lidar_frame_bridge,
        delayed_controller_spawners,
    ])
