import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import struct
import serial
from enum import Enum

class ESDACANMessage(Enum):
    SetTargetVelLeft = 1
    SetTargetVelRight = 2
    CurrentVelLeft = 3
    CurrentVelRight = 4
    CurrentDSPLeft = 5
    CurrentDSPRight = 6
    MCUState = 16
    MCUErrorState = 17
    ESTOP = 8

class MCU_Interface(Node):
    def __init__(self):
        super().__init__('mcu_interface')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.connect_to_usb_to_can_device()

    def connect_to_usb_to_can_device(self):
        serial_port = '/dev/ttyUSB0'
        baud_rate = 115200
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1, bytesize=8)
    
    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        self.send_can_msg(linear_vel, ESDACANMessage.SetTargetVelLeft)
        self.send_can_msg(linear_vel, ESDACANMessage.SetTargetVelRight)


    def send_can_msg(self, msg, ID):
        # get the linear velocity
        self.get_logger().info("Received Twist message:\nLinear.x = %f" %(msg.linear.x))

        packet = bytearray()

        # Packet ID
        packet.append(ID)

        # Packet Data
        data = bytearray(struct.pack('f', msg))
        packet.extend(data)

        # Printing for double checking
        '''
        print("Data bytes:")
        for byte in data:
            print(format(byte, '02X'), end=' ')
        print()
        print("Unpacked: {:.12f}".format(struct.unpack('d', data)[0]))
        print()
        '''

        try:
            self.serial_connection.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial connection: {e}")


    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            try:
                log_message = "Receiving data from CAN:\n"
                CAN_data = self.serial_connection.read(5)
                if CAN_data:
                    ID = struct.unpack('B', CAN_data[0:1])[0]
                    data = struct.unpack('f', CAN_data[1:5])[0]
                    log_message += f"ID: {ID}\nValue: {data}\n"
                else:
                    log_message += "No message received"
                self.get_logger().info(log_message)

            except Exception as e:
                self.get_logger().error("Error reading from serial: %s" % str(e))

    def destroy_node(self):
        if self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface()
    #rclpy.spin(mcu_interface)
    mcu_interface.spin()
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
