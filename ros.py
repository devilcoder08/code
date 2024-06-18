#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class XController(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
        self.current_pose = None
        self.Kp = 0.5
        self.distance_threshold = 0.1
        self.linear_speed = 1.0

    def pose_callback(self, msg):
        self.current_pose = msg

    def run(self):
                twist_msg = Twist()
                twist_msg.angular.z = 0.7853
                self.publisher_.publish(twist_msg)
                time.sleep(1.0)
                twist_msg = Twist()
                self.cpx=5.544445
                self.cpy=5.544445
                self.tpx=9.544445
                self.tpy=9.544445
                self.distance = math.sqrt((self.tpx - self.cpx)**2 + (self.tpy - self.cpy)**2)
                while True:
                    error_x = self.tpx - self.cpx
                    error_y = self.tpy - self.cpy
                    linear_vel = self.Kp * math.sqrt(error_x**2 + error_y**2)
                    twist_msg.linear.x =  linear_vel * self.linear_speed
                    self.publisher_.publish(twist_msg)
                    time.sleep(1.0)
                    self.cpx=self.cpx+error_x
                    self.cpy=self.cpy+error_y
                    if linear_vel < self.distance_threshold:
                        break
                twist_msg = Twist()
                twist_msg.angular.z = 3.14
                self.publisher_.publish(twist_msg)
                time.sleep(1.0)
                twist_msg = Twist()
                self.cpx=9.544445
                self.cpy=9.544445
                self.tpx=2.544445
                self.tpy=2.544445
                self.distance = math.sqrt((self.tpx - self.cpx)**2 + (self.tpy - self.cpy)**2)
                while True:
                    error_x = self.tpx - self.cpx
                    error_y = self.tpy - self.cpy
                    linear_vel = self.Kp * math.sqrt(error_x**2 + error_y**2)
                    twist_msg.linear.x =  linear_vel * self.linear_speed
                    self.publisher_.publish(twist_msg)
                    time.sleep(1.0)
                    self.cpx=self.cpx+error_x
                    self.cpy=self.cpy+error_y
                    if linear_vel < self.distance_threshold:
                        break
                twist_msg = Twist()
                twist_msg.angular.z = 3.14
                self.publisher_.publish(twist_msg)
                time.sleep(1.0)
                twist_msg = Twist()
                self.cpx=2.544445
                self.cpy=2.544445
                self.tpx=5.544445
                self.tpy=5.544445
                self.distance = math.sqrt((self.tpx - self.cpx)**2 + (self.tpy - self.cpy)**2)
                while True:
                    error_x = self.tpx - self.cpx
                    error_y = self.tpy - self.cpy
                    linear_vel = self.Kp * math.sqrt(error_x**2 + error_y**2)
                    twist_msg.linear.x =  linear_vel * self.linear_speed
                    self.publisher_.publish(twist_msg)
                    time.sleep(1.0)
                    self.cpx=self.cpx+error_x
                    self.cpy=self.cpy+error_y
                    if linear_vel < self.distance_threshold:
                        break
                twist_msg = Twist()
                twist_msg.angular.z = 1.57
                self.publisher_.publish(twist_msg)
                time.sleep(1.0)
                twist_msg = Twist()
                self.cpx=5.544445
                self.cpy=5.544445
                self.tpx=2.544445
                self.tpy=9.544445
                self.distance = math.sqrt((self.tpx - self.cpx)**2 + (self.tpy - self.cpy)**2)
                while True:
                    error_x = self.tpx - self.cpx
                    error_y = self.tpy - self.cpy
                    linear_vel = self.Kp * math.sqrt(error_x**2 + error_y**2)
                    twist_msg.linear.x =  linear_vel * self.linear_speed
                    self.publisher_.publish(twist_msg)
                    time.sleep(1.0)
                    self.cpx=self.cpx+error_x
                    self.cpy=self.cpy+error_y
                    if linear_vel < self.distance_threshold:
                        break
                twist_msg = Twist()
                twist_msg.angular.z = 3.14
                self.publisher_.publish(twist_msg)
                time.sleep(1.0)
                twist_msg = Twist()
                self.cpx=2.544445
                self.cpy=9.544445
                self.tpx=9.544445
                self.tpy=2.544445
                self.distance = math.sqrt((self.tpx - self.cpx)**2 + (self.tpy - self.cpy)**2)
                while True:
                    error_x = self.tpx - self.cpx
                    error_y = self.tpy - self.cpy
                    linear_vel = self.Kp * math.sqrt(error_x**2 + error_y**2)
                    twist_msg.linear.x =  linear_vel * self.linear_speed
                    self.publisher_.publish(twist_msg)
                    time.sleep(1.0)
                    self.cpx=self.cpx+error_x
                    self.cpy=self.cpy+error_y
                    if linear_vel < self.distance_threshold:
                        break
                rclpy.spin_once(self)
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = XController()
    controller.run()

if __name__ == '__main__':
    main()
