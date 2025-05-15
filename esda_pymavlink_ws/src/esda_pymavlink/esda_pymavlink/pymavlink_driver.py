#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil

serial_bool = False  # Toggle this to True if you want to use serial (MAVLink)

class PymavlinkDriver(Node):
    def __init__(self):
        super().__init__('pymavlink_driver')
        self.get_logger().info('Pymavlink driver node has started.')

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Only set up MAVLink connection if serial_bool is True
        if serial_bool:
            self.get_logger().info('Using MAVLink serial connection.')
            self.mavlink_connection = mavutil.mavlink_connection(
                "/dev/ttyACM1", baud=57600, source_system=255,
                source_component=0, input=False, dialect=None,
                robust_parsing=False, force_connected=False,
                use_native=False, status_handler=None, autoreconnect=0,
                heartbeat_timeout=30
            )
            self.timer = self.create_timer(1.0, self.send_heartbeat)
        else:
            self.get_logger().info('Serial disabled — only testing ROS 2 subscription.')

    def cmd_vel_callback(self, msg):
        linear = msg.linear
        angular = msg.angular

        throttle_no_serial = int(1500 + linear.x * 100)
        steer_no_serial = int(1500 + angular.z * 100)

        self.get_logger().info(
            f'Received cmd_vel:\n'
            f'  Linear:  x={linear.x:.2f}, y={linear.y:.2f}, z={linear.z:.2f}\n'
            f'  Angular: x={angular.x:.2f}, y={angular.y:.2f}, z={angular.z:.2f}'
        )

        self.get_logger().info(
            f"Test throttle value: {throttle_no_serial}, Test Steer Value: {steer_no_serial}"
        )


        if serial_bool:
            # Send heartbeat
            self.mavlink_connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            self.get_logger().info('Sent MAVLink heartbeat.')

            # Convert cmd_vel to PWM-style signals (adjust scaling to match your setup)
            throttle = int(1500 + linear.x * 500)  # Channel 2
            steer = int(1500 + angular.z * 500)   # Channel 1

            self.mavlink_connection.mav.rc_channels_override_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                steer, throttle, 0, 0, 0, 0, 0, 0  # RC channels 1-8
            )
            self.get_logger().info(f'Sent RC Override: steer={steer}, throttle={throttle}')

def main(args=None):
    rclpy.init(args=args)
    node = PymavlinkDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
