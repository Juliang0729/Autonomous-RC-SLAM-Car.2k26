from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'publish_tf': False,          # EKF owns TF
            'publish_odom': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'laser_frame': 'laser'
        }]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/home/julian/ros2_ws/src/ekf_config/config/ekf.yaml']
    )

    return LaunchDescription([
        laser_scan_matcher,
        ekf
    ])