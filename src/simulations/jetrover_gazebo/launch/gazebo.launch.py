import os
from ament_index_python.packages import get_package_share_directory

# The robot xacro (jetrover_sim.xacro) reads MACHINE_TYPE and LIDAR_TYPE from the
# environment. If they are unset, xacro fails and robot_description is empty, so
# the robot never spawns (you only see the world). Provide sane defaults here so
# the sim works even when these are not exported by the shell.
os.environ.setdefault('MACHINE_TYPE', 'JetRover_Mecanum')
os.environ.setdefault('LIDAR_TYPE', 'A1')

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    jetrover_description_share = get_package_share_directory('jetrover_description')
    jetrover_gazebo_share = get_package_share_directory('jetrover_gazebo')

    # URDF mesh URIs are `package://jetrover_description/...`; sdformat_urdf
    # rewrites that prefix to `model://jetrover_description/...`, which Gazebo
    # Sim can only resolve if a directory literally named `jetrover_description`
    # is on GZ_SIM_RESOURCE_PATH. share/jetrover_description satisfies that.
    resource_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(jetrover_description_share) + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # Hybrid-graphics (PRIME on-demand) laptop: force Gazebo's renderer onto
    # the discrete NVIDIA GPU instead of the Intel iGPU it defaults to.
    nv_offload_env = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    nv_glx_env = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    nv_vk_env = SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(jetrover_gazebo_share, 'worlds', 'jetrover_world.sdf'),
    )
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='jetrover')

    urdf_path = os.path.join(jetrover_description_share, 'urdf', 'jetrover_sim.xacro')
    robot_description = Command(['xacro ', urdf_path])

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
        # xacro output is an XML string; without ParameterValue(value_type=str)
        # launch tries to parse it as YAML and the whole launch aborts.
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

    # After URDF->SDF conversion Gazebo scopes each sensor to its parent link,
    # so /scan arrives with header.frame_id "jetrover/base_footprint/lidar" and
    # the camera topics with "jetrover/link4/depth_camera" - neither exists in
    # TF, which only knows the URDF links. slam_toolbox and RTAB-Map then reject
    # every message ("Could not convert laser scan msg!"). The sensors sit
    # exactly on lidar_frame / depth_cam_frame, so identity transforms bridge
    # the two naming schemes.
    # The gz frame names are scoped with the spawned model name, so build them
    # from the robot_name argument instead of hardcoding "jetrover" - otherwise
    # changing robot_name breaks these bridges silently.
    robot_name = LaunchConfiguration('robot_name')
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_frame', [robot_name, '/base_footprint/lidar']],
        parameters=[{'use_sim_time': True}],
    )
    depth_cam_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'depth_cam_frame', [robot_name, '/link4/depth_camera']],
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

    return LaunchDescription([
        resource_path_env,
        nv_offload_env,
        nv_glx_env,
        nv_vk_env,
        world_arg,
        robot_name_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        lidar_frame_bridge,
        depth_cam_frame_bridge,
    ])
