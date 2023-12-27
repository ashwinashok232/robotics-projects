#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial #CAN DELETE - IF NOT USING SERVICES

from turtlesim.msg import Pose #CAN DELETE
from geometry_msgs.msg import Twist #CAN DELETE
from turtlesim.srv import SetPen #CAN DELETE


class NAMENode(Node): #MODIFY ME
    
    def __init__(self):
        super().__init__("NAME") #MODIFY ME
        
        self.NAME_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) #MODIFY ME
        self.NAME_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.NAME_callback, 10) #MODIFY ME
        
    def NAME_callback(self, pose: Pose): #MODIFY ME
        cmd = Twist() #CAN DELETE
        self.cmd_vel_pub_.publish(cmd) #MODIFY ME
    
        self.call_NAME_service() #MODIFY ME

    def call_NAME_service(self): #MODIFY ME

        client = self.create_client(SetPen, "/turtle1/set_pen") #MODIFY ME
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = SetPen.Request() #MODIFY ME
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_NAME))

    def callback_NAME(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args = None):
    rclpy.init(args=args)
    node = NAMENode() #MODIFY ME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()