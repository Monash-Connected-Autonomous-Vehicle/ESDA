import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from dynamixel_sdk_custom_interfaces.msg import SetPosition

import math

class ParallelogramSteeringController(Node):
    def __init__(self):
        super().__init__('paralellogram_steering_controller')

        # /cmd_vel subscriber
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.publish_servo_position, 10)

        # /set_position publisher for dynamixelsdk servo
        self.servo_publisher = self.create_publisher(SetPosition, '/set_position', 10)
    
    '''
    This below is just a debug line to make sure the node is receiving the twist message properly
    '''
    #def listener_callback(self,msg):
    #    self.get_logger().info('Received Twist message: Linear.x=%f, Angular.z=%f' % (msg.linear.x, msg.angular.z))

    '''
    Publishes SetPosition message to /set_position for servo
    '''
    def publish_servo_position(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        servo_position = SetPosition()
        # only one servo is used
        servo_position.id = 1

        steering_angle = 0
        min_pwm = 45
        max_pwm = min_pwm + 511

        if angular != 0:
            radius = linear / angular
            # Wheel_base_length parameter for distance between the width of the car (the length is the longer part of it)
            wheel_base_length = 0.932
            steering_angle = math.atan(wheel_base_length / radius) * 180 / math.pi

        # convert degrees to pwm
        pwm = round(steering_angle * 4000 / 360) + min_pwm

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