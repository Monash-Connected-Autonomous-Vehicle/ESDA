import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
# TODO: Check/change these to the correct message types
# TODO: Fill in message types
from msg import MotorCommand
from msg import ServoPosition

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
        self.publish_motor_command(linear)

        angular = msg.angular.z
        self.publish_servo_position(angular)
    
    '''
    Publishes MotorCommand message to /target_motor_cmd for back wheels
    Subscribe units not sure
    Publish units in m/s
    '''
    def publish_motor_command(self, linear):
        # TODO: Check for linear limits
        # limit the linear velocities
        max_vel = 10
        min_vel = -10
        if linear > max_vel:
            linear = max_vel        
        elif linear < min_vel:
            linear = min_vel
        
        motor_command = MotorCommand()
        # TODO: Change message variables depending on the message contents
        motor_command.motor_left = linear
        motor_command.motor_right = linear
        self.publisher.publish(motor_command)

    '''
    Publishes ServoPosition message to /target_servo_pos for front wheels - Ackermann steering
    '''
    def publish_servo_position(self, angular):
        # TODO: Not sure how servo commands work
        # TODO: Check if any limits for angular are required

        servo_position = ServoPosition()
        # TODO: Change message variables depending on the message contents
        servo_position.servo_position = angular
        self.publisher.publish(servo_position)

def main(args=None):
    rclpy.init(args=args)
    paralellogram_steering_controller = ParallelogramSteeringController()
    rclpy.spin(paralellogram_steering_controller)
    paralellogram_steering_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()