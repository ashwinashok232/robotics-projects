#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub = self.create_publisher
        self.get_logger().info("Draw circle node has been started")


def main(args=None):
    rclpy.__init__(args=args)
    rclpy.shutdown

if __name__ == '__main__':
    main()