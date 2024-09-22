import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('twist_publisher')

    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    msg = Twist()
    msg.linear.x = 0.5  # Example linear velocity in x-direction
    msg.angular.z = 0.3  # Example angular velocity about z-axis

    while rclpy.ok():
        node.get_logger().info('Publishing: %s' % msg)
        publisher.publish(msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()