#!/usr/bin/env python3

import math
from tf_transformations import quaternion_from_euler
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RobotTFNode(Node):
    def __init__(self):
        super().__init__('robot_tf_node')

        self.broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        t_laser = TransformStamped()
        t_laser.header.stamp = self.get_clock().now().to_msg()
        t_laser.header.frame_id = "base_link"
        t_laser.child_frame_id = "laser"

        t_laser.transform.translation.x = 0.12   # forward
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.08   # height

        # Rotate 180 degrees around Z
        q = quaternion_from_euler(0.0, 0.0, math.pi)
        t_laser.transform.rotation.x = q[0]
        t_laser.transform.rotation.y = q[1]
        t_laser.transform.rotation.z = q[2]
        t_laser.transform.rotation.w = q[3]

        transforms.append(t_laser)

        t_imu = TransformStamped()
        t_imu.header.stamp = self.get_clock().now().to_msg()
        t_imu.header.frame_id = "base_link"
        t_imu.child_frame_id = "imu_icm20948"

        t_imu.transform.translation.x = 0.0
        t_imu.transform.translation.y = 0.0
        t_imu.transform.translation.z = 0.05

        t_imu.transform.rotation.w = 1.0

        transforms.append(t_imu)

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info("Static TFs published: base_link → laser, imu_icm20948")


def main():
    rclpy.init()
    node = RobotTFNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
