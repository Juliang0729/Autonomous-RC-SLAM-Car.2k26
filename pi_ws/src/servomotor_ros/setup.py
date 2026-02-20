from setuptools import setup

package_name = "servomotor_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "rclpy", "geometry_msgs", "robot_hat"],
    zip_safe=True,
    maintainer="Julian",
    maintainer_email="your_email@example.com",
    description="ROS 2 node to control RC car motors and steering using Robot HAT",
    license="MIT",
    entry_points={
        "console_scripts": [
            "car_driver_node = servomotor_ros.car_driver_node:main",
        ],
    },
)
