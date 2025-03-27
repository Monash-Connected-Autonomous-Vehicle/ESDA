import math
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from enum import Enum
import serial
import struct
import serial.tools.list_ports

import RPi.GPIO as GPIO;
from rope.base.change import Change

WHEEL_SEPARATION = 1;
WHEEL_RADIUS = 0.1325

# See [](https://developer.download.nvidia.com/assets/embedded/secure/jetson/orin_nano/docs/Jetson-Orin-Nano-DevKit-Carrier-Board-Specification_SP-11324-001_v1.3.pdf)
# GPIO12, Module Pin # 218, GP88_PWM1
ESC_PWM_PIN_LEFT = -1;
# GPIO07, Module Pin # 206, GP113_PWM7
ESC_PWM_PIN_RIGHT = -1;
# THESE ARE ACTIVE LOW
ESC_ENABLE_PIN_LEFT = -1;
ESC_ENABLE_PIN_RIGHT = -1;


# Ranging maths:
# 1000us +width => Full reversed
# 1500us +width => Neutral
# 2000us +width => Full Throttle
#
# 50Hz => T = 1/50 = 0.02s = 20,000us
# (2000/20,000)*100 = 10% duty
ESC_DUTY_FULL_FORWARD = 10;
# (1500/20,000)*100 = 7.5% duty
ESC_DUTY_NEUTRAL = 7.5;
# (1000/20,000)*100 = 5% duty
ESC_DUTY_FULL_REVERSE = 5;

ESC_PWM_FREQUENCY = 50; # Hz

class ESC_Interface(Node):

    def __init__(self, use_serial=True):
        super().__init__('orin_esc_interface')
        # Setup GPIO Pins for Driving ESCs
        GPIO.setmode(GPIO.BOARD);
        # Setup GPIO Pins for Enabling/Disabling ESCs, These are Active Low
        GPIO.setup(ESC_ENABLE_PIN_LEFT, GPIO.OUT, initial = GPIO.HIGH);
        GPIO.setup(ESC_ENABLE_PIN_RIGHT, GPIO.OUT, initial = GPIO.HIGH);
        # Configure pwm pins as outputs with initial state set to high
        GPIO.setup(ESC_PWM_PIN_LEFT, GPIO.OUT, initial=GPIO.HIGH);
        GPIO.setup(ESC_PWM_PIN_RIGHT, GPIO.OUT, initial=GPIO.HIGH);

        # Create PWM drivers for GPIO Pins at 50Hz (Standard frequency for controlling servos/ESCs)
        self.left_esc_pwm = GPIO.PWM(ESC_PWM_PIN_LEFT, ESC_PWM_FREQUENCY);
        self.right_esc_pwm = GPIO.PWM(ESC_PWM_PIN_RIGHT, ESC_PWM_FREQUENCY);

        self.calibrate_and_arm_escs();

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Finished ESC Calibration");

    def calibrate_and_arm_escs(self):
        # Cut power to the ESCs if it isnt already cut
        GPIO.output(ESC_ENABLE_PIN_LEFT, GPIO.HIGH);
        GPIO.output(ESC_ENABLE_PIN_RIGHT, GPIO.HIGH);

        # Start the pwm outputs at full forward throttle for first phase of calibration
        self.left_esc_pwm.start(ESC_DUTY_FULL_FORWARD);
        self.right_esc_pwm.start(ESC_DUTY_FULL_FORWARD);

        # Allow 300ms to apply (very very generous)
        sleep(0.3);

        # Enable power to the ESCs
        GPIO.output(ESC_ENABLE_PIN_LEFT, GPIO.LOW);
        GPIO.output(ESC_ENABLE_PIN_RIGHT, GPIO.LOW);

        # Allow 8 Seconds for ESC power on and first calibration tone
        sleep(8)

        # Set ESCS to full reverse for second part of calibration
        self.left_esc_pwm.ChangeDutyCycle(ESC_DUTY_FULL_REVERSE);
        self.right_esc_pwm.ChangeDutyCycle(ESC_DUTY_FULL_REVERSE);

        # Allow 8 Seconds for second calibration tone
        sleep(8)

        # Set ESCS to full reverse for final part of calibration
        self.left_esc_pwm.ChangeDutyCycle(ESC_DUTY_NEUTRAL);
        self.right_esc_pwm.ChangeDutyCycle(ESC_DUTY_NEUTRAL);

        # Allow 10 Seconds for third and fourth calibration tones
        sleep(8)

    def cmd_vel_callback(self, msg: Twist):
        """
        Takes a twist message and converts it to throttle adjustments for the orin nano's pwm pins
        """
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Convert the robot motion command to wheel velocities
        (w_left, w_right) = robot_motion_to_wheel_angular_velocity(linear_vel, angular_vel);

        # Convert those to throttles and update accordingly
        throttle_left_duty = wheel_angular_velocity_to_throttle_duty(w_left);
        throttle_right_duty = wheel_angular_velocity_to_throttle_duty(w_right);

        # Apply the throttles to the escs
        self.left_esc_pwm.ChangeDutyCycle(throttle_left_duty);
        self.right_esc_pwm.ChangeDutyCycle(throttle_right_duty);

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

    def destroy_node(self):
        if self.use_serial and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def wheel_angular_velocity_to_throttle_duty(angular_velocity: float) -> float:
    """
    Real world conversion to be determined experimentally
    """
    # Arbitrary implementation assumes maximum rotation speed is 19,000RPM ~= 1990rad/sec

    # Clamp angular velocity within the range
    abs_max_angular_velocity = (19000/60)*2*math.pi;
    angular_velocity = max(-abs_max_angular_velocity, min(angular_velocity, abs_max_angular_velocity));
    # Map to between 5 and 10 (2*(19000/60)*2*pi)/5 = 796
    # => (5*angular_velocity)/(2*abs_max_angular_velocity)+7.5
    # => (5*angular_velocity)/(2*(19000/60)*2*math.pi)+7.5
    # => (5*angular_velocity)/((3800/3)*math.pi)+7.5
    # => (3*angular_velocity)/(760*math.pi)+7.5
    return (3*angular_velocity)/(760*math.pi)+7.5

def robot_motion_to_wheel_angular_velocity(linear_velocity: float, angular_velocity: float):
    """
    Equations sourced from https://www.roboticsbook.org/S52_diffdrive_actions.html Equation 5.11
    """
    left_wheel_angular_velocity = -(WHEEL_SEPARATION/2)*(angular_velocity/WHEEL_RADIUS);
    right_wheel_angular_velocity = (WHEEL_SEPARATION/2)*(angular_velocity/WHEEL_RADIUS);

    return (left_wheel_angular_velocity, right_wheel_angular_velocity);

def main(args=None):
    rclpy.init(args=args)
    mcu_interface = ESC_Interface()
    mcu_interface.spin()
    mcu_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
