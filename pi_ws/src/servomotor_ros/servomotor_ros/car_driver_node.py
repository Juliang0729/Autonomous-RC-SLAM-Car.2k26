#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_hat import Servo, Motor, PWM, Pin
from time import time

class CarDriverNode(Node):
    def __init__(self):
        super().__init__('car_driver_node')

        # ---- Servo Steering ----
        self.steer = Servo(2)  # channel 2
        self.STEER_OFFSET = -3  # degrees, dependent on neutral position
        # ---- Motors ----
        self.left  = Motor(PWM("P13"), Pin("D4"))
        self.right = Motor(PWM("P12"), Pin("D5"))

        # Motor direction correction
        self.LEFT_DIR  = -1
        self.RIGHT_DIR = 1

        self.MAX_SPEED = 100
        self.MAX_STEER = 22  # degrees

        self.last_cmd_time = time()
        self.TIMEOUT = 1.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.create_timer(0.1, self.check_timeout)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time()

        # linear.x -> motor speed
        speed = max(min(msg.linear.x * 100, self.MAX_SPEED), -self.MAX_SPEED)
        self.left.speed(self.LEFT_DIR * speed)
        self.right.speed(self.RIGHT_DIR * speed)

        # angular.z -> steering angle
        steer_angle = max(min(-msg.angular.z * self.MAX_STEER, self.MAX_STEER), -self.MAX_STEER)
        self.steer.angle(steer_angle + self.STEER_OFFSET)


    def check_timeout(self):
        if time() - self.last_cmd_time > self.TIMEOUT:
            self.left.speed(0)
            self.right.speed(0)
            self.steer.angle(0)

def main(args=None):
    rclpy.init(args=args)
    node = CarDriverNode()
    rclpy.spin(node)
    node.left.speed(0)
    node.right.speed(0)
    node.steer.angle(0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
