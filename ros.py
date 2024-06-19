#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


class XController(Node):
    def __init__(self):
        super().__init__("turtle_mover")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.current_pose = None
        self.Kp = 0.5
        self.distance_threshold = 0.1
        self.linear_speed = 1.0

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_to_point(self, target_x, target_y):
        twist_msg = Twist()
        error = [0, 0]
        cur_pos = [self.current_pose.x, self.current_pose.y]
        trgt_pos = [target_x, target_y]
        """
        From your previous code example it was found that the cp can be auto fetched
        and needs no mannual specifiaction
        If this approach does not work go back to declaring it like this
        
        def move_to_point(self, target_x, target_y, current_x, current_y):
            cp =[current_x, current_y]
            
        Callback example :- self.move_to_point(9.544445, 9.544445, 5.544445, 5.544445)
        """
        while True:
            # Sync values
            rclpy.spin_once(self)

            for i in range(2):
                error[i] = trgt_pos[i] - cur_pos[i]
            distance = math.sqrt(error[0] ** 2 + error[1] ** 2)
            linear_vel = self.Kp * distance
            twist_msg.linear.x = linear_vel * self.linear_speed
            self.publisher_.publish(twist_msg)
            time.sleep(1.0)
            cur_pos = [self.current_pose.x, self.current_pose.y]
            """ Another way of doing above line
            
            for i in range(2):
                cp[i] = cp[i] + error[i]
            """
            if linear_vel < self.distance_threshold:
                break
        twist_msg.linear.x = 0.0
        self.publisher_.publish(twist_msg)

    def rotate(self, angular_speed, duration):
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed
        self.publisher_.publish(twist_msg)
        time.sleep(duration)
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

    def run(self):
        rclpy.spin_once(self)

        self.rotate(math.radians(45), 1.0)
        self.move_to_point(9.544445, 9.544445)  # Move to point 1

        self.rotate(math.radians(180), 1.0)
        self.move_to_point(2.544445, 2.544445)  # Move to point 2

        self.rotate(math.radians(180), 1.0)
        self.move_to_point(5.544445, 5.544445)  # Move to point 3

        self.rotate(math.radians(90), 1.0)  # Rotate 90 degrees
        self.move_to_point(2.544445, 9.544445)  # Move to point 4

        self.rotate(math.radians(180), 1.0)
        self.move_to_point(9.544445, 2.544445)  # Move to point 5

        rclpy.spin_once(self)
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = XController()
    controller.run()


if __name__ == "__main__":
    main()
