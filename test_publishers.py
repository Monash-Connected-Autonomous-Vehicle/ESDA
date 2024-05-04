import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from array import array as Array
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')

        qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.zed_publisher = self.create_publisher(OccupancyGrid, '/zed', qos_profile = qos)
        self.lidar_publisher = self.create_publisher(OccupancyGrid, '/lidar', qos_profile = qos)
        self.timer = self.create_timer(2.0, self.publish_obstacles)

    def publish_obstacles(self):
        zed = OccupancyGrid()
        lidar = OccupancyGrid()

        zed.header.stamp = self.get_clock().now().to_msg()
        lidar.header.stamp = self.get_clock().now().to_msg()

        zed.header.frame_id = 'map'
        lidar.header.frame_id = 'map'

        zed.info.resolution = 1.0
        lidar.info.resolution = 0.5

        zed.info.width = 10
        zed.info.height = 10
        lidar.info.width = 20
        lidar.info.height = 20

        zed_grid = self.generate_obstacle_grid(10, 10, 5)
        zed.data = Array('b', zed_grid.ravel().astype(np.int8))
        lidar_grid = self.generate_obstacle_grid(20, 20, 0)
        lidar.data = Array('b', lidar_grid.ravel().astype(np.int8))

        self.zed_publisher.publish(zed)
        time.sleep(1.0)
        self.lidar_publisher.publish(lidar)
    
    def generate_obstacle_grid(self, width, height, row):
        grid = np.zeros((width, height), dtype=np.int8)
        # Place a single occupied cell in each row
        for i in range(width):
            grid[i, row] = 100  # Occupied cell in the middle of each row
        return grid

def main(args=None):
    rclpy.init(args=args)
    occupancy_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_publisher)
    occupancy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
