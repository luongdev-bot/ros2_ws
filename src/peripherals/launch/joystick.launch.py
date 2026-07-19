from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Gamepad teleop for Gazebo: joy_node (reads the physical pad) -> our
    # joystick_control node -> /cmd_vel.
    cmd_vel = LaunchConfiguration('cmd_vel', default='/cmd_vel')
    # joy_node (Humble) selects the pad by index, not by /dev path, so expose
    # device_id - the previous 'device' argument was declared but never used.
    device_id = LaunchConfiguration('device_id', default='0')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel', default_value=cmd_vel),
        DeclareLaunchArgument('device_id', default_value=device_id,
                              description='Index of the gamepad (js0 -> 0)'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': ParameterValue(device_id, value_type=int),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='peripherals',
            executable='joystick_control',
            name='joystick_control',
            output='screen',
            parameters=[{'cmd_vel': cmd_vel}],
        ),
    ])
