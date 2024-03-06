import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import struct
import serial
import numpy as np

class MCU_Interface(Node):
    def __init__(self):
        super().__init__('mcu_interface')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.send_can_msg, 10)
        self.connect_to_usb_to_can_device()

    def connect_to_usb_to_can_device(self):
        # TODO: might need to adjust values
        serial_port = '/dev/ttyUSB0'
        baud_rate = 115200
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1, bytesize=8)

    def send_can_msg(self,msg):
        # get the linear velocity
        linear_vel = msg.linear.x
        self.get_logger().info("Received Twist message: Linear.x=%f" %(msg.linear.x))

        packet = bytearray()

        # Packet ID
        packet.append(21)

        # Packet Data
        packet.extend(bytearray(struct.pack('f', linear_vel)))

        try:
            self.serial_connection.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial connection: {e}")


def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface()
    rclpy.spin(mcu_interface)
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
