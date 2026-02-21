# Pi Workspace (ROS 2)

This workspace contains low-level hardware interface and vehicle control packages:

- servomotor_ros – Motor and steering control node (PiCar-X interface)
- rc_car_tf – Base TF broadcaster for vehicle frames
- rc_car_bringup – System launch
- ros2_icm20948 – IMU driver (custom fork included as submodule)

This workspace is intended to run on Ubuntu 24.04 Server with ROS 2 Jazzy.

# Dependencies:

- imu_tools (https://github.com/CCNYRoboticsLab/imu_tools)
- rplidar_ros (https://github.com/Slamtec/rplidar_ros)

# Submodules:

The IMU driver is included as a Git submodule:
- ros2_icm20948 (custom fork)

After cloning this repository, initialize the submodule:
```console
git submodule update --init --recursive
```

# System Architecture:

This project separates low-level hardware control (pi_ws) from high-level autonomy (laptop_ws) to:

- Reduce hardware load
- Improve SLAM and navigation performance
