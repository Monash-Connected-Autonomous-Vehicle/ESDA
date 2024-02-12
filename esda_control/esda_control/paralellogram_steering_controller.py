import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from msg import MotorCommand
from msg import ServoPosition

import math

'''
Are the message contents correct?
Is the limit for wheel angle 46 in both directions or combined?
Assuming that for travelling in a straight line, servo position is 2000 (middle of 0 to 4000)
'''

class ParallelogramSteeringController(Node):
    def __init__(self):
        super().__init__('paralellogram_steering_controller')

        # /cmd_vel subscriber
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        # /target_motor_cmd publisher (not sure if MotorCommand is the right message type)
        self.publisher = self.create_publisher(MotorCommand, '/target_motor_cmd', 10)

        # /target_servo_pos publisher (not sure if ServoPosition is the right message type)
        self.publisher = self.create_publisher(ServoPosition, '/target_servo_pos', 10)
    
    '''
    Gets message from /cmd_vel, gets required data and calls relevant publishers
    '''
    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.publish_motor_command(linear)
        self.publish_servo_position(linear, angular)
    
    '''
    Publishes MotorCommand message to /target_motor_cmd for back wheels
    Subscribe units not sure
    Publish units in m/s
    '''
    def publish_motor_command(self, linear):
        # No limit check is needed
        motor_command = MotorCommand()
        motor_command.left_wheel = linear
        motor_command.right_wheel = linear
        self.publisher.publish(motor_command)

    '''
    Publishes ServoPosition message to /target_servo_pos for front wheels
    '''
    def publish_servo_position(self, linear, angular):
        servo_position = ServoPosition()

        # If angular = 0, it is going in a straight line
        if angular == 0:
            servo_position.servo_position = 2000
        else:
            radius = linear / angular
            # TODO: Might want to get this from a parameter so that it can be changed
            wheel_base_length = 0.5
            turn_degree = math.atan(wheel_base_length / radius) * 180 / math.pi

            turn_degree = max(min(turn_degree, 46), -46)
            
            # convert degrees to pwm
            servo_position.servo_position = turn_degree * 4000 / 360 + 2000

        self.publisher.publish(servo_position)

def main(args=None):
    rclpy.init(args=args)
    paralellogram_steering_controller = ParallelogramSteeringController()
    rclpy.spin(paralellogram_steering_controller)
    paralellogram_steering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()