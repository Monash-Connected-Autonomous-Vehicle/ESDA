import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def publish_cmd_vel(self):
        twist_msg = Twist()
        self.linear_velocity += 0.1  # Increment linear velocity
        self.angular_velocity += 0.1  # Increment angular velocity

        # Reset velocities if they exceed a threshold
        if self.linear_velocity > 3.0:
            self.linear_velocity = 0.0
        if self.angular_velocity > 1.0:
            self.angular_velocity = -1.0  # Reverse direction

        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Published Twist message: linear={twist_msg.linear.x}, angular={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestPublisher()
    rclpy.spin(test_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()