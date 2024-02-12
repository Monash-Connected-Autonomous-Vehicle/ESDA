import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from msg import MotorCommand
from dynamixel_sdk_custom_interfaces.msg import SetPosition

import math

class ParallelogramSteeringController(Node):
    def __init__(self):
        super().__init__('paralellogram_steering_controller')

        # /cmd_vel subscriber
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        # /target_motor_cmd publisher (not sure if MotorCommand is the right message type)
        self.motor_publisher = self.create_publisher(MotorCommand, '/target_motor_cmd', 10)

        # /set_position publisher for dynamixelsdk servo
        self.servo_publisher = self.create_publisher(ServoPosition, '/set_position', 10)
    
    '''
    Gets message from /cmd_vel, gets required data and calls relevant publishers
    '''
    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.publish_motor_command(linear)
        self.publish_servo_position(linear, angular)
    
    '''
    Publishes MotorCommand message to /target_motor_cmd for mcu chip
    '''
    def publish_motor_command(self, linear):
        # No limit check is needed
        motor_command = MotorCommand()
        motor_command.linear_velocity = linear
        self.motor_publisher.publish(motor_command)

    '''
    Publishes SetPosition message to /set_position for servo
    '''
    def publish_servo_position(self, linear, angular):
        servo_position = SetPosition()
        # only one servo is used
        servo_position.id = 1

        steering_angle = 0
        min_pwm = 45
        max_pwm = min_pwm + 511

        if angular != 0:
            radius = linear / angular
            # TODO: Might want to get this from a parameter so that it can be changed
            wheel_base_length = 0.5
            steering_angle = math.atan(wheel_base_length / radius) * 180 / math.pi

        # convert degrees to pwm
        pwm = round(turn_degree * 4000 / 360) + min_pwm

        # limit pwm
        if pwm < min_pwm:
            pwm = min_pwm
        elif pwm > max_pwm:
            pwm = max_pwm
        
        servo_position.position = pwm

        self.servo_publisher.publish(servo_position)

def main(args=None):
    rclpy.init(args=args)
    paralellogram_steering_controller = ParallelogramSteeringController()
    rclpy.spin(paralellogram_steering_controller)
    paralellogram_steering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()