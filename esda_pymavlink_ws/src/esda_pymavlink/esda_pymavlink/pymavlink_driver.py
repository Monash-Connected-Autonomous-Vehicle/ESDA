#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class PymavlinkDriver(Node):
    def __init__(self):
        super().__init__('pymavlink_driver')
        self.get_logger().info('Pymavlink driver node has started.')

def main(args=None):
    rclpy.init(args=args)
    node = PymavlinkDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
