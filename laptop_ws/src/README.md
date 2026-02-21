# Laptop Workspace (ROS 2)

This workspace contains high-level autonomy packages:

- ekf_config – Extended Kalman Filter configuration
- rc_car_bringup – System launch
- rc_nav2 – Navigation2 configuration and costmap setup

This workspace is intended to run on Ubuntu 22.04 with ROS 2 Humble.

# Dependencies:

- slam_toolbox
- navigation2
- robot_localization
- imu_tools (https://github.com/CCNYRoboticsLab/imu_tools)
- ROS2 csm (https://github.com/AlexKaravaev/csm)
- ros2_laser_scan_matcher (https://github.com/AlexKaravaev/ros2_laser_scan_matcher)
- rviz2

# System Architecture:

This project separates low-level hardware control (pi_ws) from high-level autonomy (laptop_ws) to:

- Reduce hardware load
- Improve SLAM and navigation performance
