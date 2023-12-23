#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from functools import partial
from turtlesim.msg import Pose
import random

class RobotNode(Node):
    
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info('Robot Controller Started')
        self.spawn_x_ = 2.0
        self.spawn_y_ = 2.0
        self.spawn_exists_ = False
        self.call_spawn_service(self.spawn_x_, self.spawn_y_, 0.0, self.spawn_exists_)

        self.cmd_vel_pub_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

    def pose_callback(self, pose:Pose):
        # self.get_logger().info("[ " + str(pose.x) + "," + str(pose.y) + " ]")

        cmd = Twist()

        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub_.publish(cmd)

        if (0.0 <= abs(pose.x-self.spawn_x_)) <= 0.3 and (0.0 <= abs(pose.y-self.spawn_y_)) <= 0.3:
            self.get_logger().info("[ " + str(pose.x) + "," + str(pose.y) + " ]")
            self.spawn_x_ = round(random.uniform(2.0,9.0),1)
            self.spawn_y_ = round(random.uniform(2.0,9.0),1)
            self.call_spawn_service(self.spawn_x_, self.spawn_y_, 0.0, self.spawn_exists_)
            self.spawn_exists_ = True
        else:
            self.spawn_exists_ = False

    def call_spawn_service(self, x, y, theta, spawn_exists):
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        if not spawn_exists:
            request = Spawn.Request()
            request.x = x
            request.y = y
            request.theta = theta

            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_set_spawn))

    def callback_set_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()