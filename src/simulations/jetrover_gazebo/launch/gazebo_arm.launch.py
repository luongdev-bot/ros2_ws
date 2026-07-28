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

    # Where to drop the robot. (0, 0) is clear in the JetRover worlds, but a
    # downloaded world places its building wherever its author did - spawning at
    # the origin can put the robot inside a shelf, or outside the building where
    # the lidar sees nothing and SLAM maps an empty plane. The world catalogue
    # (config/world_catalog.yaml) carries a per-world spawn point for that reason.
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0')

    # The pose the arm boots in is task-specific, so it is an argument rather
    # than a constant: arm_perception wants pick_init (the default below), the
    # SLAM sims want the horizontal pose that aims the depth camera forward
    # (config/slam_initial_positions.yaml). Sharing one file made the two
    # fight - see the comments in both yaml files.
    initial_positions_arg = DeclareLaunchArgument(
        'initial_positions_file',
        default_value=os.path.join(
            jetrover_moveit_config_share, 'config', 'initial_positions.yaml'),
        description='YAML of initial joint positions for the gz_ros2_control system.',
    )

    urdf_path = os.path.join(jetrover_moveit_config_share, 'config', 'jetrover.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_path,
        ' use_gazebo:=true',
        ' initial_positions_file:=', LaunchConfiguration('initial_positions_file'),
    ], on_stderr='warn')

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

    # Gazebo's rgbd_camera reports header.frame_id "jetrover/link4/depth_camera"
    # after URDF->SDF conversion, which ROS consumers cannot resolve. Anchor it
    # to depth_cam_frame, NOT depth_cam_link: the sensor is mounted on
    # depth_cam_link because Gazebo aims cameras along +X, but every ROS
    # consumer of the image (image_geometry, depth_image_proc, RTAB-Map,
    # MoveIt's octomap) assumes the header frame uses the OPTICAL convention
    # (+Z forward). depth_cam_frame is exactly that frame, and its +Z equals
    # depth_cam_link's +X. Anchoring to depth_cam_link instead projects every
    # 3D point behind the camera - verified with a known-position object.
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
        arguments=[
            '/world/color_blocks_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        parameters=[{
            'config_file': os.path.join(jetrover_gazebo_share, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    ros_gz_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/grasp/entity_command@ros_gz_interfaces/msg/Entity]ignition.msgs.Entity',
        ],
        parameters=[{'use_sim_time': True}],
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
    grasp_attacher_node = Node(
        package='jetrover_gazebo',
        executable='grasp_attacher',
        output='screen',
        parameters=[{
            'robot_model_name': LaunchConfiguration('robot_name'),
            'use_sim_time': True,
        }],
    )
    # controller_manager lives inside the Gazebo process (gz_ros2_control), so
    # only start the spawners and simulation attacher once the entity has
    # actually been created.
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
                grasp_attacher_node,
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
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        initial_positions_arg,
        gz_sim,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        ros_gz_cmd_bridge,
        depth_cam_frame_bridge,
        lidar_frame_bridge,
        delayed_controller_spawners,
    ])
