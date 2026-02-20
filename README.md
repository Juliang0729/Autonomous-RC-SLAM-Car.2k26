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
The assembly process for the SLAM car began with the Sunfounder PiCar-X. The PiCar was chosen as the base for this project for three primary reasons:
1. Price: The Sunfounder PiCar-X retails at ~ $90 (Pi 5 NOT included), which is a relatively low price for a vehicle with multiple included sensors (Camera, Ultrasonic, Greyscale), Ackermann steering, and a completely aluminium alloy build. While cheaper alternatives do exist, many of the lower-cost vehicles I found during the research phase of the project are made with low quality materials that may fracture due to the many collisions and roll-overs occurring in the physical testing phase of the project.

2. Size: From the outset, I knew that size would be a determining factor in the success of this project. As development began at my home in Atlanta, GA and ended at my dorm room in Washington, D.C., minimizing the car's size and weight while maximizing portability was a crucial step in the assembly process. On top of this, I understood that needlessly increasing the size of the car would one, raise the total cost of the project due to the need for progessively larger motors, servos, batteries, etc., and two, would not improve the overall SLAM process. The PiCar, coming in at ~ 26 x 17 x 11 cm and 800g, fell perfectly within these constraints.

3. Purpose & Versatility: Unlike other RC cars on the market, the PiCar was specifically created to be a platform for Raspberry Pi-based personal projects much like my own. Thanks to this, the vehicle works as intended right out of the box, eliminating the need to rewire factory RC electronics, modify RF communication hardware, or perform custom soldering jobs. This purpose-built design by the car allowed me to skip much of the electrical work unfamiliar to me as a mechanical engineering student.

<p align="center">
  <img src="https://github.com/user-attachments/assets/808b555f-9fc4-4afd-8139-edf70d404c09" width="25%" />
</p>
<p align="center"><em>Figure 1: Fully assembled base PiCar-X.</em></p>

With the PiCar-X now fully assembled, I brainstormed ways to physically attach the RPLIDAR C1 to the chassis of the car. After much deliberation, I came to the final decision of 3D printing a custom stand for the LiDAR and attaching it to the chassis with foam velcro tape. To avoid capturing the car itself in the point cloud, the LiDAR stand needed to raise the sensor ~ 90mm above the chassis. A truss design was used to achieve this while maintaining structural stability. Apart from creating an easily detachable connection, the foam tape also served the purpose of dampening vibrations caused by the DC motors.

<p align="center">
  <img src="https://github.com/user-attachments/assets/05f4acf8-b94f-4c64-8248-77fe3c8f0b3f" width="35%" />
</p>
<p align="center"><em>Figure 2: LiDAR mount static stress test.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/55bd55cc-8e83-4420-b0c1-10864ae8001e" width="35%" />
</p>
<p align="center"><em>Figure 3: LiDAR attachment to stand w/ 4x M2.5 screws.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/4a077228-db19-4785-87c5-02175781b2d3" width="35%" />
</p>
<p align="center"><em>Figure 4: LiDAR mount attachment to chassis.</em></p>

Attaching the IMU to the chassis was very straightforward. Two 2.5mm holes were drilled into the base of the car and the IMU was mounted via standoffs.

<p align="center">
  <img src="https://github.com/user-attachments/assets/bc4a4ffb-3cf9-44f4-b7e5-5cb48b2f2d59" width="25%" />
</p>
<p align="center"><em>Figure 5: 2.5mm holes drilled into chassis.</em></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/71bab5be-4592-4cf6-98f9-e2c995a5c503" width="25%" />
</p>
<p align="center"><em>Figure 6: IMU attachment to chassis.</em></p>


To deliver sufficient power to the LiDAR and IMU, a second battery must be connected to the Pi in addition to the 2000mAh Li-ion battery pack inluded with the PiCar. Initially, I thought of combining a standalone LiPo battery pack with a DC-DC buck converter to deliver a constant 5V @ 5A to the Pi. While this method works theoretically, it introduces unforseen factors such as battery over-discharge, brownouts and overheating that could potentially damage vital components of the car.

As a safer alternative, I went with using the Geekworm X1200 UPS HAT paired with two 18650 3000mAh cells. This approach both simplified the power delivery architecture and met the tight spatial constraints of the PiCar chassis. Physically, the UPS HAT mounts directly under the Pi, delivering a regulated 5.1V @ 5A through the board's pogo pins and requiring no additional wiring or external cabling. With both the Geekworm UPS HAT and Sunfounder Robot HAT mounted directly to the Pi, the power delivery system formed a compact "stack" with the Pi 5 sitting between the two boards. To give the stack enough clearance above the IMU, longer brass standoffs were screwed into the chassis.

<p align="center">
  <img src="https://github.com/user-attachments/assets/e6b97573-9a47-4159-a0aa-4e61ae9af255" width="35%" />
</p>
<p align="center"><em>Figure 7: IMU (very bottom), Geekworm UPS HAT (second from bottom), Pi 5 (middle), Robot HAT (top).</em></p>

The completion of the power delivery system marks the end of the assembly process. The figure below gives a detailed overview of the car's physical wiring and power delivery architecture.

<p align="center">
  <img src="https://github.com/user-attachments/assets/977cab67-161c-4bcc-b948-7787cf372ea7" width="65%" />
</p>
<p align="center"><em>Figure 8: Complete wiring and power delivery architecture of the car.</em></p>

## OS Setup & Software Installation:
Initially, I booted the system from Raspberry Pi OS to test the sensors, servos, and DC motors included with the PiCar. Verifying that all of the vital components can be controlled digitally via Sunfounder's "RobotHAT" Python library, I fully wiped the system and installed Ubuntu 24.04 Server on the Pi. My choice of Ubuntu over Pi OS for the project boils down to two primary reasons:

1. ROS 2 Compatibility: ROS 2 is officially supported and thoroughly tested on Ubuntu LTS releases. Pi OS, on the other hand, is Debian-based and is known to have general compatibility issues with ROS. To avoid having to manage running ROS 2 in a Docker container on top of Pi OS, I chose the path of least resistance and installed Ubuntu. While the Sunfounder Robot HAT was not designed to work in conjunction with Ubuntu, I concluded that debugging Python libraries now would be far less difficult than dealing with Docker, which I have limited knowledge on, later when installing ROS.

2. Performance: Thanks to the server edition of Ubuntu 24.04 lacking a visual desktop and being completely terminal-based, I found that installing it over Pi OS would maximize performance and power efficiency going forward. Running a desktop environment continuously provided no operational benefit for the project, as I would be communicating with the Pi headlessly via SSH and visualizing everything on my laptop. Additionally, having both the Pi and my laptop running on Ubuntu simplified dependency management and kept the OS consistent across the project.

With the Pi running on Ubuntu 24.04 Server and my laptop running on Ubuntu 22.04, I installed ROS 2 Jazzy and Humble, respectfully, on both devices and began building my workspace. Due to my two-device approach to integrating ROS with the PiCar, I chose to keep all nodes directly accessing the sensors, servos, and motors locally on the Pi and the fusion, calibration, and visualization nodes on the laptop. This way the Pi could act as an embedded microcontroller executing low-level command outputs for the car and the laptop could act as a remote workstation recieving data from the Pi and managing SLAM as well as visualization.

## IMU Calibration & ROS Integration:

Modifying an existing Python script integrating the SparkFun ICM-20948 with ROS 2, I developed a custom node to publish accelerometer, gyroscope, and magnetometer readings from the IMU under the /??? topic in the Pi ROS workspace. Accessing the imu_tools ROS package, raw readings were then sent into a madgwick filter to produce a combined pose estimate. To calibrate the accelerometer and gyroscope, 
