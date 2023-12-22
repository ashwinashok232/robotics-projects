#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS2")

def main(args = None):
    rclpy.init(args=args) #initialize ros2 comms
    node = MyNode()
    rclpy.spin(node) #node will be kept alive until you click ctrl+c
    rclpy.shutdown() #shutdown ros2 comms

if __name__ == '__main__':
    main()