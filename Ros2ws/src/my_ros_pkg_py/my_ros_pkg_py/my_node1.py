#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode1(Node):
    def __init__(self):
        super().__init__("custom_node_1")
        #self.get_logger().info("Hello from ROS2\n Custom Node Running:")
        self.counter = 0
        self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello from ROS - Executing Custom Node:"+str(self.counter))
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()