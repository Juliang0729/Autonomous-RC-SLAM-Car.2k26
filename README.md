# Autonomous-RC-SLAM-Car.2k26
Repository created for the RC SLAM Car project completed January - February 2026 by GWU student Julian Gross.
## Bill of Materials:
- SunFounder PiCar-X
- Raspberry Pi 5 4GB
- Raspberry Pi 5 Active Cooler
- Micro SD Card 64GB + Adapter
- SLAMTEC RPLIDAR C1M1
- 3D Printed LiDAR Mount (Ordered Online)
- Adafruit ICM-20948 9-DoF IMU
- Pimoroni 4 Pin JST-SH Cable
- Geekworm X1200 5V UPS HAT Shield
- 2x Samsung 30Q 18650 3000mAh Battery
- M2.5 Hex Brass Standoff Screws Kit
## Assembly Process:
The assembly process for the SLAM car began with the SunFounder PiCar-X. The PiCar was chosen as the base for this project for three primary reasons:
1. Price: The SunFounder PiCar-X retails at ~ $90 (Pi 5 NOT included), which is a relatively low price for a vehicle with multiple included sensors (Camera, Ultrasonic, Greyscale), Ackermann steering, and a completely aluminum alloy build. While cheaper alternatives do exist, many of the lower-cost vehicles I found during the research phase of the project are made with low quality materials that may fracture due to the many collisions and roll-overs occurring in the physical testing phase of the project.

2. Size: From the outset, I knew that size would be a determining factor in the success of this project. As development began at my home in Atlanta, GA and ended at my dorm room in Washington, D.C., minimizing the car's size and weight while maximizing portability was a crucial step in the assembly process. On top of this, I understood that needlessly increasing the size of the car would one, raise the total cost of the project due to the need for progressively larger motors, servos, batteries, etc., and two, would not improve the overall SLAM process. The PiCar, coming in at ~ 26 x 17 x 11 cm and 800g, fell perfectly within these constraints.

3. Purpose & Versatility: Unlike other RC cars on the market, the PiCar was specifically created to be a platform for Raspberry Pi-based personal projects much like my own. Thanks to this, the vehicle works as intended right out of the box, eliminating the need to rewire factory RC electronics, modify RF communication hardware, or perform custom soldering jobs. This purpose-built design by the car allowed me to skip much of the electrical work unfamiliar to me as a mechanical engineering student.

<p align="center">
  <img src="https://github.com/user-attachments/assets/808b555f-9fc4-4afd-8139-edf70d404c09" width="35%" />
</p>
<p align="center"><em>Figure 1: Fully assembled base PiCar-X.</em></p>

With the PiCar-X now fully assembled, I brainstormed ways to physically attach the RPLIDAR C1 to the chassis of the car. After much deliberation, I came to the final decision of 3D printing a custom stand for the LiDAR and attaching it to the chassis with foam Velcro tape. To avoid capturing the car itself in the point cloud, the LiDAR stand needed to raise the sensor ~ 90mm above the chassis. A truss design was used to achieve this while maintaining structural stability. Apart from creating an easily detachable connection, the foam tape also served the purpose of dampening vibrations caused by the DC motors.

<p align="center">
  <img src="https://github.com/user-attachments/assets/05f4acf8-b94f-4c64-8248-77fe3c8f0b3f" width="45%" />
</p>
<p align="center"><em>Figure 2: LiDAR mount static stress test.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/55bd55cc-8e83-4420-b0c1-10864ae8001e" width="45%" />
</p>
<p align="center"><em>Figure 3: LiDAR attachment to stand w/ 4x M2.5 screws.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/4a077228-db19-4785-87c5-02175781b2d3" width="45%" />
</p>
<p align="center"><em>Figure 4: LiDAR mount attachment to chassis.</em></p>

Attaching the IMU to the chassis was very straightforward. Two 2.5mm holes were drilled into the base of the car and the IMU was mounted via standoffs.

<p align="center">
  <img src="https://github.com/user-attachments/assets/bc4a4ffb-3cf9-44f4-b7e5-5cb48b2f2d59" width="35%" />
</p>
<p align="center"><em>Figure 5: 2.5mm holes drilled into chassis.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/71bab5be-4592-4cf6-98f9-e2c995a5c503" width="35%" />
</p>
<p align="center"><em>Figure 6: IMU attachment to chassis.</em></p>


To deliver sufficient power to the LiDAR and IMU, a second battery must be connected to the Pi in addition to the 2000mAh Li-ion battery pack included with the PiCar. Initially, I thought of combining a standalone LiPo battery pack with a DC-DC buck converter to deliver a constant 5V @ 5A to the Pi. While this method works theoretically, it introduces unforeseen factors such as battery over-discharge, brownouts and overheating that could potentially damage vital components of the car.

As a safer alternative, I went with using the Geekworm X1200 UPS HAT paired with two 18650 3000mAh cells. This approach both simplified the power delivery architecture and met the tight spatial constraints of the PiCar chassis. Physically, the UPS HAT mounts directly under the Pi, delivering a regulated 5.1V @ 5A through the board's pogo pins and requiring no additional wiring or external cabling. With both the Geekworm UPS HAT and SunFounder Robot HAT mounted directly to the Pi, the power delivery system formed a compact "stack" with the Pi 5 sitting between the two boards. To give the stack enough clearance above the IMU, longer brass standoffs were screwed into the chassis.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e6b97573-9a47-4159-a0aa-4e61ae9af255" width="45%" />
</p>
<p align="center"><em>Figure 7: IMU (very bottom), Geekworm UPS HAT (second from bottom), Pi 5 (middle), Robot HAT (top).</em></p>

The completion of the power delivery system marks the end of the assembly process. The figure below gives a detailed overview of the car's physical wiring and power delivery architecture.

<p align="center">
  <img src="https://github.com/user-attachments/assets/977cab67-161c-4bcc-b948-7787cf372ea7" width="65%" />
</p>
<p align="center"><em>Figure 8: Complete wiring and power delivery architecture of the car.</em></p>

## OS Setup & Software Installation:
Initially, I booted the system from Raspberry Pi OS to test the sensors, servos, and DC motors included with the PiCar. After confirming that all vital components could be digitally controlled using SunFounder's _RobotHAT_ Python library, I fully wiped the system and installed Ubuntu 24.04 Server on the Pi. My decision to use Ubuntu over Raspberry Pi OS was based on two primary considerations:

1. ROS 2 Compatibility: ROS 2 is officially supported and thoroughly tested on Ubuntu LTS releases. Raspberry Pi OS, being Debian-based, is known to have general compatibility issues with ROS. Rather than running ROS 2 inside a Docker container on top of Pi OS, I chose the path of least resistance and installed Ubuntu. Although the SunFounder Robot HAT was not specifically designed to work with Ubuntu, I concluded that debugging Python libraries now would be far less difficult than troubleshooting Docker later when installing ROS.

2. Performance: Thanks to Ubuntu 24.04 Server being entirely terminal-based, operating without a visual desktop environment, I found that installing it over Pi OS would maximize both performance and power efficiency for the Pi. Running a continuous desktop environment provided no operational benefit for the project, as the Pi would be accessed headlessly via SSH and all visualization would be done on my laptop. Additionally, having both the Pi and my laptop running on Ubuntu simplified dependency management and kept the OS consistent across the project.

With the Ubuntu 24.04 Server running on the Pi and Ubuntu 22.04 on my laptop, I installed ROS 2 Jazzy on the Pi and ROS 2 Humble on the laptop and began building my workspace. Given this two-device approach to integrating ROS with the PiCar, I kept all hardware-interface nodes, those directly accessing sensors, servos, and motors, on the Pi. Higher-level processes such as sensor fusion, calibration, SLAM, and visualization were kept on the laptop. This way the Pi could function as an embedded controller executing low-level actuation and data acquisition, while the laptop could act as a remote workstation receiving data from the Pi and managing mapping, navigation, and visualization.

After a long-winded debugging process installing the required Python libraries, enabling i2c and spi using raspi-config, copying hardware dtoverlays, and verifying pin mappings with i2cdetect, the Pi was ready to use with ROS 2.

## Motor & Servo Control:

Developing a motor and steering control node in ROS was very straightforward, largely thanks to the reference Robot HAT [test scripts](https://github.com/sunfounder/robot-hat/tree/v2.0/tests) available on SunFounder's GitHub. The integration process boiled down to identifying the correct motor and servo pin mappings on the HAT and using the _RobotHAT_ Python library to implement simple control logic. `car_driver_node.py` was written to receive steering and throttle commands from geometry_msgs/Twist on the cmd_vel topic, converting linear and angular velocity commands into motor and servo outputs.

## IMU Calibration & ROS Integration:

Modifying an existing [ROS node](https://github.com/norlab-ulaval/ros2_icm20948) integrating the SparkFun ICM-20948 with ROS 2, I developed a custom node `ros2_icm20948` to publish raw accelerometer, gyroscope, and magnetometer readings from the Adafruit ICM-20948 under the /imu/data_raw and /imu/mag topics in the Pi ROS workspace. Utilizing the [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools) ROS package, raw IMU data was passed through a Madgwick filter to produce a fused orientation estimate published under /imu/data. For calibration, [imucal](https://github.com/mad-lab-fau/imucal) was used to correct accelerometer and gyroscope biases, and [magnetometer_calibration](https://github.com/italocjs/magnetometer_calibration) was used to compute hard- and soft-iron compensation values for the magnetometer. The resulting calibration coefficients were integrated directly into `icm20948_node` to linearly transform raw sensor measurements into calibrated vectors, reducing IMU drift and improving overall orientation accuracy.

<p align="center">
  <img src="https://github.com/user-attachments/assets/f8975096-a542-4c99-9e33-054fb8bfa1d5" width="65%" />
</p>
<p align="center"><em>Figure 9: Accelerometer and gyroscope calibration with imucal.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/ed190b49-46cc-4de4-8f75-b07777f51195" width="45%" />
</p>
<p align="center"><em>Figure 10: Hard- and soft-iron Magnetometer calibration with magnetometer_calibration.</em></p>

## LiDAR Visualization & Scan Matching:
Integrating the RPLiDAR C1 with ROS 2 was very straightforward thanks to [rplidar_ros](https://github.com/Slamtec/rplidar_ros), an ROS node developed by Slamtec for their line of LiDAR sensors. Launching `rplidar_c1_launch.py` starts the LiDAR sensor and publishes the readings under the /laser topic. Creating static transforms connecting **laser** and **imu_icm20948** to **base_link** allows for both the LiDAR point cloud and IMU pose estimate to be visualized in RViz.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d4254fb0-0133-4f71-9a01-43dd639429eb" width="65%" />
</p>
<p align="center"><em>Figure 11: LiDAR point cloud & pose estimate shown in RViz.</em></p>

Due to the lack of wheel encoders on the PiCar, LiDAR scan matching was employed to estimate odometry and improve overall pose estimation accuracy. To implement scan matching, [ros2_laser_scan_matcher](https://github.com/AlexKaravaev/ros2_laser_scan_matcher) along with a ported version of [csm](https://github.com/AlexKaravaev/csm) for ROS 2 was used. These nodes worked as is, needing no modification.

## Sensor Fusion & SLAM:

With LiDAR providing reliable odometry and the IMU delivering calibrated orientation estimates, both sensors were utilized within a unified fusion pipeline to produce a consistent and accurate state estimate of the car. Through an Extended Kalman Filter (EKF) LiDAR scan matching served as the primary source of planar pose and velocity information, providing ${x}$, ${y}$, and yaw (${\psi}$) estimates along with the corresponding linear velocities $\dot{x}$ and $\dot{y}$. The IMU was configured to provide high-frequency orientation and angular velocity measurements, with the gyroscope being the main source of yaw rate ($\dot{\psi}$) estimates to stabilize heading between LiDAR updates. Altogether, the EKF parameters were written into `ekf.yaml` and implemented using [robot_localization](https://github.com/cra-ros-pkg/robot_localization).

To perform SLAM, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) was utilized to integrate EKF pose estimates with incoming LiDAR point clouds, enabling occupancy grid mapping and localization. Prior to any autonomy, the car was driven around my dorm room using [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) for keyboard control. Running the EKF along with slam_toolbox produced a detailed 2D map of the dorm, delineating objects such as desks or dressers in its output.

<p align="center">
  <img src="https://github.com/user-attachments/assets/1b7b68e1-9689-4a20-9dba-5fed71a1ddc7" width="65%" />
</p>
<p align="center"><em>Figure 12: 2D map of dorm room produced by slam_toolbox, overlayed with LiDAR point cloud.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/7b891af8-63dd-400f-930a-6fd2087f90be" width="45%" />
</p>
<p align="center"><em>Figure 13: Comparison of dorm room floor plan with map produced by slam_toolbox (with annotations).</em></p>

## Autonomous Navigation with Nav2:

The final phase of the project was to enable autonomous navigation using [Nav2](https://github.com/ros-navigation/navigation2). This process was very straightforward, consisting of writing a detailed yaml `nav2_params.yaml` outlining velocity and acceleration constraints as well as vehicle footprint information and steering geometry (Ackermann). The constraints were determined through iterative testing aimed at preventing the car from becoming immobilized by static obstacles, as well as to mitigate tipping caused excessive acceleration.

Finally, two launch scripts `laptop_bringup.launch.py`, `pi_bringup.launch.py` were written for the laptop and the Pi 5, respectfully. These scripts launch all of the required ROS nodes, from servo and motor control to sensor fusion, necessary for SLAM and autonomous navigation. Running Nav2 enabled the car to follow user-defined waypoints placed in RViz. As the room was explored, a detailed global costmap was generated depicting the difficulty, or cost, of traversing certain areas between or around objects. As the costmap updated, waypoint navigation got smarter, prioritizing object avoidance while determining the most efficient possible route between points.

<p align="center">
  <img src="https://github.com/user-attachments/assets/13cdd5c6-b426-4855-8c09-09d53146a302" width="65%" />
</p>
<p align="center"><em>Figure 13: Costmap with trajectory planning as well as pose and heading estimates.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f08c08f0-3a6f-4153-bc01-14702efc61c5" width="65%" />
</p>
<p align="center"><em>Figure 14: Costmap with LiDAR point cloud overlay.</em></p>

## Takeaways & Potential Further Improvements:

The completion of this project left me with an enhanced understanding of how theoretical robotics concepts, such as sensor fusion, trajectory planning, and state estimation, translate into practical engineering challenges. Looking back at my work, I can attribute much of the success of this project to one, choosing the right materials and components, and two, having a defined system architecture. 

Deeply researching both the reliability and compatibility of components beforehand resulted in the assembly and software integration processes being highly streamlined and painless. The only real hurdles in project came from ROS dependency version mismatches and certain Raspberry Pi OS functions being different or unavailable on Ubuntu. Thanks to the two-device system architecture maintaining separate ROS workspaces on the laptop and the Pi, bugs could be easily isolated and identified as either hardware or software related.

With the SLAM car project complete for now, I propose a list of potential further improvements:

- **Wheel Encoders**: Incorporating wheel encoders into the project provides direct velocity feedback, improving odometry and state estimation while eliminating reliance on scan matching.

- **Camera, Ultrasonic, Greyscale Sensors**: While LiDAR and IMU were used as the primary sensors in this project for the sake of simplicity, incorporating the camera, ultrasonic, and greyscale sensors included with the PiCar-X enables line tracking, cliff detection, and image tagging via computer vision. As the sensors included with the PiCar lack out of the box support for Ubuntu found in Raspberry Pi OS, debugging libcamera, V4L2 and rpicam is likely needed. Using a different HAT than the SunFounder Robot HAT or bypassing the HAT altogether and accessing the Pi's GPIO pins directly may resolve some bugs.

- **Autonomous Navigation**: Developing more advanced navigation algorithms, such as an autonomous 'wander' function for mapping, could help the car operate with required less user involvement. Implementing and training a neural network on larger, more complex environments could significantly improve the car's local planner.

- **Misc. Hardware Upgrades/Changes**:
  1. Relocating the IMU to a location with less magnetic interference, preferably away from the batteries and Pi, could reduce IMU drift and orientation estimation inaccuracies.
  2. Using a depth camera instead of a standard Raspberry Pi camera could improve object detection, path planning, and overall SLAM accuracy.
  3. Swapping the Raspberry Pi 5 for an Nvidia Jetson Nano could improve hardware and thermal performance, as well as enable advanced machine learning integration with the project.
