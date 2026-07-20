"""Colour-block detection + action-group triggering.

Assumes the camera is already streaming and arm_motion is up. For the
full stack, launch in three terminals:

    ros2 launch jetrover_gazebo gazebo.launch.py \
        world:=$(ros2 pkg prefix jetrover_gazebo)/share/jetrover_gazebo/worlds/color_blocks_world.sdf
    ros2 launch arm_motion arm_motion.launch.py
    ros2 launch arm_perception color_pick.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('arm_perception')

    config = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([share, 'config', 'color_pick.yaml']),
        description='Colour thresholds and colour -> action group bindings.',
    )
    image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/depth_cam/image',
        description='RGB stream to watch.',
    )
    play_motion_action = DeclareLaunchArgument(
        'play_motion_action',
        default_value='/arm_motion_server/play_motion',
        description="arm_motion's PlayMotion action server.",
    )
    confirm_frames = DeclareLaunchArgument(
        'confirm_frames',
        default_value='5',
        description='Consecutive frames of one colour before a motion fires.',
    )
    cooldown_s = DeclareLaunchArgument(
        'cooldown_s',
        default_value='3.0',
        description='Quiet period after a motion ends, in seconds.',
    )
    center_tolerance_px = DeclareLaunchArgument(
        'center_tolerance_px',
        default_value='0.0',
        description='Max distance from frame centre to trigger; 0 disables the check.',
    )
    start_enabled = DeclareLaunchArgument(
        'start_enabled',
        default_value='true',
        description='Trigger motions immediately, or wait for ~/enable.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo clock.',
    )

    color_pick = Node(
        package='arm_perception',
        executable='color_pick',
        name='color_pick',
        output='screen',
        parameters=[{
            'config': LaunchConfiguration('config'),
            'image_topic': LaunchConfiguration('image_topic'),
            'play_motion_action': LaunchConfiguration('play_motion_action'),
            'confirm_frames': LaunchConfiguration('confirm_frames'),
            'cooldown_s': LaunchConfiguration('cooldown_s'),
            'center_tolerance_px': LaunchConfiguration('center_tolerance_px'),
            'start_enabled': LaunchConfiguration('start_enabled'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        config,
        image_topic,
        play_motion_action,
        confirm_frames,
        cooldown_s,
        center_tolerance_px,
        start_enabled,
        use_sim_time,
        color_pick,
    ])
