"""Launch the YOLO-segmentation camera line-following node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('line_follow')

    python_executable = DeclareLaunchArgument(
        'python_executable',
        default_value=os.path.join(
            os.path.expanduser('~'), 'ultralytics_env', 'bin', 'python3'
        ),
        description=(
            'Machine-specific Python interpreter with ROS 2, torch, and '
            'ultralytics available; override if its venv moves.'
        ),
    )
    model_path = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution(
            [share, 'models', 'line_yolo11n_seg.pt']
        ),
        description='YOLO11n-seg line model file.',
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
    pid_kp = DeclareLaunchArgument(
        'pid_kp',
        default_value='0.006',
        description='PID proportional steering gain.',
    )
    pid_ki = DeclareLaunchArgument(
        'pid_ki',
        default_value='0.0',
        description='PID integral steering gain.',
    )
    pid_kd = DeclareLaunchArgument(
        'pid_kd',
        default_value='0.0',
        description='PID derivative steering gain.',
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
        description='Publish line annotations on ~/debug_image.',
    )
    start_enabled = DeclareLaunchArgument(
        'start_enabled',
        default_value='true',
        description='Follow immediately, or wait for the ~/enable service.',
    )
    confidence_threshold = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum YOLO segmentation confidence.',
    )
    device = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Ultralytics inference device, such as 0 or cpu.',
    )
    imgsz = DeclareLaunchArgument(
        'imgsz',
        default_value='640',
        description='Square YOLO inference image size in pixels.',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo clock.',
    )

    # The invoking shell must source ROS 2 and this workspace overlay first;
    # ExecuteProcess inherits that PYTHONPATH for the venv interpreter.
    yolo_line_follow = ExecuteProcess(
        cmd=[
            LaunchConfiguration('python_executable'),
            '-m',
            'line_follow.infrastructure.ros.yolo_line_follow_node',
            '--ros-args',
            '-p', ['model_path:=', LaunchConfiguration('model_path')],
            '-p', ['image_topic:=', LaunchConfiguration('image_topic')],
            '-p', ['cmd_vel_topic:=', LaunchConfiguration('cmd_vel_topic')],
            '-p', ['cruise_speed:=', LaunchConfiguration('cruise_speed')],
            '-p', [
                'max_angular_speed:=',
                LaunchConfiguration('max_angular_speed'),
            ],
            '-p', ['min_speed_scale:=', LaunchConfiguration('min_speed_scale')],
            '-p', ['pid_kp:=', LaunchConfiguration('pid_kp')],
            '-p', ['pid_ki:=', LaunchConfiguration('pid_ki')],
            '-p', ['pid_kd:=', LaunchConfiguration('pid_kd')],
            '-p', [
                'lost_line_timeout_s:=',
                LaunchConfiguration('lost_line_timeout_s'),
            ],
            '-p', ['camera_timeout_s:=', LaunchConfiguration('camera_timeout_s')],
            '-p', [
                'publish_debug_image:=',
                LaunchConfiguration('publish_debug_image'),
            ],
            '-p', ['start_enabled:=', LaunchConfiguration('start_enabled')],
            '-p', [
                'confidence_threshold:=',
                LaunchConfiguration('confidence_threshold'),
            ],
            '-p', ["device:='", LaunchConfiguration('device'), "'"],
            '-p', ['imgsz:=', LaunchConfiguration('imgsz')],
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
        ],
        output='screen',
    )

    return LaunchDescription([
        python_executable,
        model_path,
        image_topic,
        cmd_vel_topic,
        cruise_speed,
        max_angular_speed,
        min_speed_scale,
        pid_kp,
        pid_ki,
        pid_kd,
        lost_line_timeout_s,
        camera_timeout_s,
        publish_debug_image,
        start_enabled,
        confidence_threshold,
        device,
        imgsz,
        use_sim_time,
        yolo_line_follow,
    ])
