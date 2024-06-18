#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class XController(Node):
    def _init_(self):
        super()._init_('turtle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()
        self.Kp = 0.5
        self.distance_threshold = 0.1
        self.linear_speed = 1.0

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_to_goal(self, goal_x, goal_y):
        twist_msg = Twist()
        self.distance = math.sqrt((goal_x - self.current_pose.x)**2 + (goal_y - self.current_pose.y)**2)

        while self.distance >= self.distance_threshold and rclpy.ok():
            error_x = goal_x - self.current_pose.x
            error_y = goal_y - self.current_pose.y
            self.distance = math.sqrt((error_x)**2 + (error_y)**2)
            linear_vel = self.Kp * self.distance

            twist_msg.linear.x = linear_vel * self.linear_speed
            self.publisher_.publish(twist_msg)

            time.sleep(0.1)

        twist_msg.linear.x = 0.0
        self.publisher_.publish(twist_msg)

    def run(self):
        rclpy.spin_once(self)
        
        
        self.move_to_goal(9.544445, 9.544445)
        self.turn_turtle(0.7853)
        
        self.move_to_goal(2.544445, 2.544445)
        self.turn_turtle(3.14)
        
        self.move_to_goal(5.544445, 5.544445)
        self.turn_turtle(3.14)
        
        self.move_to_goal(2.544445, 9.544445)
        self.turn_turtle(1.57)
        
        self.move_to_goal(9.544445, 2.544445)
        self.turn_turtle(3.14)
        self.destroy_node()
        rclpy.shutdown()

    def turn_turtle(self, angle):
        twist_msg = Twist()
        twist_msg.angular.z = angle
        self.publisher_.publish(twist_msg)
        time.sleep(1.0)
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = XController()
    controller.run()

if __name__ == '__main__':
    main()
