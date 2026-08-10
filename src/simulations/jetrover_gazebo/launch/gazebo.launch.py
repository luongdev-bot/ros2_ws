import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# jetrover_sim.xacro reads MACHINE_TYPE and LIDAR_TYPE from the environment.
# With them unset, xacro aborts ("environment variable 'LIDAR_TYPE' is not set")
# and the launch dies before Gazebo starts. Default them here - set at import,
# long before the lazy xacro Command actually runs - so the launch works from a
# plain shell. setdefault, so an explicitly exported value still wins.
os.environ.setdefault('MACHINE_TYPE', 'JetRover_Mecanum')
os.environ.setdefault('LIDAR_TYPE', 'A1')


def generate_launch_description():
    jetrover_description_share = get_package_share_directory('jetrover_description')
    jetrover_gazebo_share = get_package_share_directory('jetrover_gazebo')

    # URDF mesh URIs are `package://jetrover_description/...`; sdformat_urdf
    # rewrites that prefix to `model://jetrover_description/...`, which Gazebo
    # Sim can only resolve if a directory literally named `jetrover_description`
    # is on GZ_SIM_RESOURCE_PATH. The vendored turtlebot3_house model is
    # similarly installed under this package's share root as
    # share/turtlebot3_house, so expose both per-package install roots.
    resource_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(jetrover_description_share) + ':' +
        os.path.dirname(jetrover_gazebo_share) + ':' +
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

    # Where to drop the robot. (0, 0) is clear in the JetRover worlds, but a
    # downloaded world places its building wherever its author did - spawning at
    # the origin can put the robot inside a shelf, or outside the building where
    # the lidar sees nothing and SLAM maps an empty plane. The world catalogue
    # (config/world_catalog.yaml) carries a per-world spawn point for that reason.
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0')

    urdf_path = os.path.join(jetrover_description_share, 'urdf', 'jetrover_sim.xacro')
    robot_description = Command(['xacro ', urdf_path])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [
            LaunchConfiguration('world'), ' -r',
            # Stock Fortress GUI plus the VisualizeLidar plugin, so the
            # laser fan is drawn in the Gazebo window itself.
            ' --gui-config ', os.path.join(
                jetrover_gazebo_share, 'config', 'gui.config'),
        ]}.items(),
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
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.05',
            '-Y', LaunchConfiguration('spawn_yaw'),
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
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        lidar_frame_bridge,
        depth_cam_frame_bridge,
    ])
