import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from enum import Enum
import serial
import struct
import serial.tools.list_ports

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
    def __init__(self, use_serial=True):
        super().__init__('mcu_interface')
        self.use_serial = use_serial
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Log available USB ports
        self.list_serial_ports()

        if self.use_serial:
            self.connect_to_usb_to_can_device()

    def list_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        if ports:
            self.get_logger().info("Available Serial Ports:")
            for port in ports:
                self.get_logger().info(f"Port: {port.device}, Description: {port.description}")
        else:
            self.get_logger().info("No serial ports found.")

    def connect_to_usb_to_can_device(self):
        serial_port = '/dev/ttyACM1'
        baud_rate = 115200
        self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1, bytesize=8)
    
    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        self.send_can_msg(linear_vel, ESDACANMessage.SetTargetVelLeft)
        self.send_can_msg(linear_vel, ESDACANMessage.SetTargetVelRight)

    

    def send_can_msg(self, msg, ID):
        # Log the received message instead of sending it via serial
        self.get_logger().info("Received Twist message: Linear.x = %f" % msg)

        if self.use_serial:
            packet = bytearray()
            # packet.append(ID.value)  # First byte: ID
            # data = bytearray(struct.pack('<f', msg))  # Sending float as little-endian

            packet.extend(struct.pack('<I', ID.value))  # Pack ID as 4-byte unsigned integer (little-endian)

            # Add the float data to the packet (bytes 2 to 5)
            packet.extend(struct.pack('<I', 255))  # Pack 255 as 4-byte unsigned integer (little-endian)

            # Add 3 padding bytes (or any other values) to make the total length 8 bytes
            # while len(packet) < 8:
            #     packet.append(0)  # Pad with zeros
            packet.append(0x0D)
            packet.append(0x0A)
                

            try:
                self.serial_connection.write(packet)
                self.get_logger().info(f"Sent packet: {list(packet)}")  # Log the sent packet for debugging
            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial connection: {e}")


    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.use_serial:
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
        if self.use_serial and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mcu_interface = MCU_Interface(use_serial=True)  # Set use_serial=False for testing without serial
    mcu_interface.spin()
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
