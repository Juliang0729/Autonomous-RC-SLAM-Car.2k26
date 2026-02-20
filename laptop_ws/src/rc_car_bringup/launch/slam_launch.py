from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    transform_period = LaunchConfiguration('transform_publish_period')

    slam_params = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link'
        ),

        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom'
        ),

        DeclareLaunchArgument(
            'transform_publish_period',
            default_value='0.02'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {
                    'base_frame': base_frame,
                    'odom_frame': odom_frame,
                    'transform_publish_period': transform_period,
                }
            ]
        )
    ])