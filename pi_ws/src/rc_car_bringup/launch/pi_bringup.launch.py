from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_icm20948'),
                'imu_bringup_launch.py'
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_c1_launch.py'
            )
        )
    )

    static_tf = Node(
        package='rc_car_tf',
        executable='robot_tf_node',
        name='robot_tf_node',
        output='screen'
    )

    car_driver = Node(
        package='servomotor_ros',
        executable='car_driver_node',
        name='car_driver_node',
        output='screen'
    )

    return LaunchDescription([
        imu_launch,
        lidar_launch,
        static_tf,
        car_driver
    ])