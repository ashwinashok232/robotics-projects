#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from functools import partial
from turtlesim.msg import Pose
import random
import math
from turtlesim.srv import Kill
import numpy as np

class RobotNode(Node):
    
    def __init__(self):
        super().__init__("robot_controller")
        self.get_logger().info('Robot Controller Started')
        self.spawn_x_ = 2.0
        self.spawn_y_ = 2.0
        self.counter_ = 0
        self.turtle_name_ = "spawn_turtle" + str(self.counter_)
        self.turning_ = False
        self.call_spawn_service(self.spawn_x_, self.spawn_y_, 0.0, self.turtle_name_)

        self.cmd_vel_pub_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

    def pose_callback(self, pose:Pose):
        self.get_logger().info("[ " + str(pose.x) + "," + str(pose.y) + " ]")
        self.motion_control(pose)

        if (0.0 <= abs(pose.x-self.spawn_x_) <= 1.0 and
            0.0 <= abs(pose.y-self.spawn_y_) <= 1.0):

            self.call_kill_service(self.turtle_name_)
            self.spawn_x_ = round(random.uniform(2.0,9.0),1)
            self.spawn_y_ = round(random.uniform(2.0,9.0),1)
            self.counter_ += 1
            self.turtle_name_ = "spawn_turtle" + str(self.counter_)
            self.call_spawn_service(self.spawn_x_, self.spawn_y_, 0.0, self.turtle_name_)


    def motion_control(self, pose):
        cmd = Twist()
        goalVector = (self.spawn_x_- pose.x,
                        self.spawn_y_ - pose.y)
        
        currentVector = (math.cos(pose.theta), 
                         math.sin(pose.theta))
        
        angleToGoal = angle_between(currentVector, goalVector)
        
        if angleToGoal >= 0.2:
            turnDirection = find_turn_direction(currentVector, goalVector)
            if turnDirection == "Clockwise":
                cmd.linear.x = 0.0
                cmd.angular.z = -1.5
            elif turnDirection == "Counter Clockwise":
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5
        else:
            cmd.linear.x = 3.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub_.publish(cmd)


    def call_spawn_service(self, x, y, theta, turtle_name): 
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_spawn))

    def callback_set_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

    def call_kill_service(self, turtle_name):
        client = self.create_client(Kill, '/kill')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill))

    def callback_kill(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()


### Helper Functions ###
    
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def find_turn_direction(currentVector, goalVector):
    if np.cross(currentVector, goalVector) < 0:
        return "Clockwise"
    else:
        return "Counter Clockwise"

    