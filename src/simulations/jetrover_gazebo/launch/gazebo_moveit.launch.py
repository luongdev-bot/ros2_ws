import os
from ament_index_python.packages import get_package_share_directory

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

    # This laptop is a hybrid-graphics (PRIME on-demand) setup: without these,
    # Gazebo's ogre2 renderer (GUI + the GPU lidar/rgbd_camera sensors) runs
    # on the Intel iGPU instead of the idle NVIDIA GPU.
    nv_offload_env = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nv_glx_env = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    nv_vk_env = SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(jetrover_gazebo_share, 'worlds', 'jetrover_world.sdf'),
    )
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='jetrover')

    # Full robot description used for Gazebo: jetrover_sim.xacro (wheels +
    # sensors) plus the arm/gripper <ros2_control> block wired to
    # gz_ros2_control instead of the demo's mock_components.
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

    # Gazebo's rgbd_camera sensor reports header.frame_id as
    # "jetrover/link4/depth_camera" (its own naming after URDF->SDF
    # conversion) instead of our "depth_cam_frame" link, which breaks
    # MoveIt's octomap TF lookup. depth_cam_joint_sim already places
    # depth_cam_frame at the standard ROS optical-frame pose, so the two
    # frames are co-located - bridge them with an identity transform.
    depth_cam_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'depth_cam_frame', 'jetrover/link4/depth_camera'],
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

    # gz_ros2_control hosts controller_manager inside the Gazebo process (see
    # the <plugin> in jetrover.urdf.xacro); these spawners just activate the
    # controllers it exposes, same as on real ros2_control hardware.
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
    # Spawners fail fast if controller_manager isn't up yet; chain them after
    # the entity has finished spawning instead of racing it.
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

    # Reuse the pre-generated launch files as-is: they build their own
    # MoveItConfigsBuilder internally and already handle the
    # ParameterValue(..., value_type=str) wrapping robot_description needs.
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetrover_moveit_config_share, 'launch', 'move_group.launch.py')
        ),
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetrover_moveit_config_share, 'launch', 'moveit_rviz.launch.py')
        ),
    )

    return LaunchDescription([
        resource_path_env,
        nv_offload_env,
        nv_glx_env,
        nv_vk_env,
        # move_group's TF listener otherwise runs off the wall clock while
        # Gazebo publishes sim time, causing octomap "extrapolation into
        # the future" TF lookup failures.
        SetParameter(name='use_sim_time', value=True),
        world_arg,
        robot_name_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        depth_cam_frame_bridge,
        delayed_controller_spawners,
        move_group_launch,
        moveit_rviz_launch,
    ])
