"""Camera line following that publishes planar base velocity.

Assumes a camera is streaming on ``image_topic`` and something is listening
on ``cmd_vel_topic``: either the mecanum base driver or the Gazebo sim plugin.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('line_follow')

    config = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([share, 'config', 'line_follow.yaml']),
        description='LAB line threshold, weighted ROIs, and PID gains.',
    )
    image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/depth_cam/image',
        description='RGB camera stream used to detect the line.',
    )
    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Chassis velocity command topic.',
    )
    cruise_speed = DeclareLaunchArgument(
        'cruise_speed',
        default_value='0.15',
        description='Forward speed on a centered line, in m/s.',
    )
    max_angular_speed = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.8',
        description='Absolute steering limit, in rad/s.',
    )
    min_speed_scale = DeclareLaunchArgument(
        'min_speed_scale',
        default_value='0.4',
        description='Minimum forward-speed fraction during sharp turns.',
    )
    lost_line_timeout_s = DeclareLaunchArgument(
        'lost_line_timeout_s',
        default_value='0.4',
        description='How long to coast through a missed detection.',
    )
    camera_timeout_s = DeclareLaunchArgument(
        'camera_timeout_s',
        default_value='1.0',
        description='Stop if the camera publishes no frame for this long.',
    )
    publish_debug_image = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Publish ROI and line annotations on ~/debug_image.',
    )
    start_enabled = DeclareLaunchArgument(
        'start_enabled',
        default_value='true',
        description='Follow immediately, or wait for the ~/enable service.',
    )
    use_cuda = DeclareLaunchArgument(
        'use_cuda',
        default_value='true',
        description=(
            'Enable GPU-accelerated line detection when a CUDA-enabled cv2 '
            'build is reachable.'
        ),
    )
    cuda_cv2_path = DeclareLaunchArgument(
        'cuda_cv2_path',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'ultralytics_env',
            'lib',
            'python3.10',
            'site-packages',
            'cv2',
            'python-3.10',
        ),
        description=(
            'Machine-specific directory containing a separately-built '
            'CUDA-enabled OpenCV Python extension; override if its venv moves.'
        ),
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo clock.',
    )

    cuda_pythonpath = PythonExpression([
        "'",
        LaunchConfiguration('cuda_cv2_path'),
        "' + (",
        repr(os.pathsep),
        " + '",
        EnvironmentVariable('PYTHONPATH', default_value=''),
        "' if '",
        EnvironmentVariable('PYTHONPATH', default_value=''),
        "' else '')",
    ])
    prepend_cuda_cv2 = SetEnvironmentVariable(
        'PYTHONPATH',
        cuda_pythonpath,
        condition=IfCondition(LaunchConfiguration('use_cuda')),
    )

    line_follow = Node(
        package='line_follow',
        executable='line_follow',
        name='line_follow',
        output='screen',
        parameters=[{
            'config': LaunchConfiguration('config'),
            'image_topic': LaunchConfiguration('image_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'cruise_speed': LaunchConfiguration('cruise_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'min_speed_scale': LaunchConfiguration('min_speed_scale'),
            'lost_line_timeout_s': LaunchConfiguration('lost_line_timeout_s'),
            'camera_timeout_s': LaunchConfiguration('camera_timeout_s'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'start_enabled': LaunchConfiguration('start_enabled'),
            'use_cuda': LaunchConfiguration('use_cuda'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        config,
        image_topic,
        cmd_vel_topic,
        cruise_speed,
        max_angular_speed,
        min_speed_scale,
        lost_line_timeout_s,
        camera_timeout_s,
        publish_debug_image,
        start_enabled,
        use_cuda,
        cuda_cv2_path,
        use_sim_time,
        prepend_cuda_cv2,
        line_follow,
    ])
