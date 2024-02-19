import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import math

class MCU_Interface(Node):
    def __init__(self):
        super().__init__('mcu_interface')

        # /cmd_vel subscriber
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        #TODO CAN message publisher here

    def listener_callback(self,msg):
        self.get_logger().info("Received Twist message: Linear.x=%f" %(msg.linear.x))

    """
    Publishes the /cmd_vel's linear.x value to the MCU chip
    """
    # def publish_CAN_message(self,msg):
    #     linear = msg.linear.x

def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface()
    rclpy.spin(mcu_interface)
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()