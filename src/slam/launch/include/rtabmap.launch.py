from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    qos = LaunchConfiguration('qos', default='1')
    # GPU feature extraction (ORB/FAST on CUDA). Requires rtabmap built against
    # an OpenCV compiled WITH CUDA (cudafeatures2d). The stock apt OpenCV is
    # CPU-only, so this defaults to false; set use_gpu:=true only after you have
    # a CUDA-enabled OpenCV + rtabmap (see slam/README.md).
    use_gpu = LaunchConfiguration('use_gpu', default='false').perform(context) == 'true'

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        # RTAB-Map's parameters must be strings:
        'queue_size': 50,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',       # planar robot -> lock to 2D motion
        'Grid/RangeMin': '0.2',        # ignore laser points on the robot itself
        'Optimizer/GravitySigma': '0',  # no IMU constraints (already 2D)
        'Kp/DetectorStrategy': '2',    # 2 = ORB
        'Vis/FeatureType': '2',        # 2 = ORB
        'ORB/Gpu': 'true' if use_gpu else 'false',
        'FAST/Gpu': 'true' if use_gpu else 'false',
        'Grid/Sensor': 'true',
    }

    # Sim topic names come from jetrover_gazebo/config/gz_bridge.yaml:
    #   RGB   -> /depth_cam/image
    #   depth -> /depth_cam/depth_image
    #   info  -> /depth_cam/camera_info   (shared, camera is a single rgbd sensor)
    #   odom  -> /odom      scan -> /scan
    remappings = [
        ('rgb/image', '/depth_cam/image'),
        ('rgb/camera_info', '/depth_cam/camera_info'),
        ('depth/image', '/depth_cam/depth_image'),
        ('odom', '/odom'),
        ('scan', '/scan'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.02,
                     'use_sim_time': use_sim_time, 'qos': qos}],
        remappings=remappings)

    rtabmap = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'])

    return [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('qos', default_value='1',
                              description='QoS used for input sensor topics'),
        DeclareLaunchArgument('use_gpu', default_value='false',
                              description='CUDA ORB/FAST — needs CUDA-enabled OpenCV+rtabmap'),
        rgbd_sync,
        rtabmap,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
