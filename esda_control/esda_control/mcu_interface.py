import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
import struct
import serial

class MCU_Interface(Node):
    def __init__(self):
        super().__init__('mcu_interface')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.send_can_msg, 10)
        self.connect_to_usb_to_can_device()

    def connect_to_usb_to_can_device(self):
        # TODO: might need to adjust values
        serial_port = '/dev/ttyUSB0'
        baud_rate = 2000000
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)

    def send_can_msg(self,msg):
        # get the linear velocity
        linear_vel = msg.linear.x
        self.get_logger().info("Received Twist message: Linear.x=%f" %(msg.linear.x))

        can_frame = Frame()
        # TODO: not sure what id is
        can_frame.id = 0x123
        can_frame.is_rtr = False
        can_frame.is_extended = False
        can_frame.is_error = False
        can_frame.dlc = 8
        # convert the linear velocity to 8 bytes of data
        can_frame.data = list(struct.pack('!d', linear_vel))

        # convert the CAN frame to binary data so that it can be sent over the serial port
        binary_data = struct.pack('B', can_frame.id, can_frame.is_rtr, can_frame.is_extended, can_frame.is_error, can_frame.dlc, *can_frame.data)
        self.serial_connection.write(binary_data)


def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface()
    rclpy.spin(mcu_interface)
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
