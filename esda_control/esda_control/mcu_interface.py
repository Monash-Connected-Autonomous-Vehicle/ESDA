import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MCU_Interface(Node):
    def __init__(self):
        super().__init__('mcu_interface')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.send_can_msg, 10)
        self.connect_to_usb_to_can_device()

    def connect_to_usb_to_can_device(self):
        # todo: connection to usb-to-can device implemented here

    def send_can_msg(self,msg):
        # get the linear velocity
        linear_vel = msg.linear.x
        self.get_logger().info("Received Twist message: Linear.x=%f" %(msg.linear.x))

        # todo: convert the linear velocity to a CAN frame

        # todo: send the CAN frame to the device we connected to in __init__ line 13
        # note: we send 1 linear cmd to left wheel and 1 to right wheel so 2 CAN frames are sent


def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface()
    rclpy.spin(mcu_interface)
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
