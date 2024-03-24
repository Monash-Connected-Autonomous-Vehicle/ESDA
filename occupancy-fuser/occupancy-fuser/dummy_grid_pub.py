from array import array as Array
from nav_msgs.msg import OccupancyGrid, MapMetaData

import rclpy
from rclpy.node import Node
import numpy as np


class OccupancyPub(Node):
    def __init__(self):
        super().__init__('occupancy_pub')
        self.grid1_pub= self.create_publisher(
            OccupancyGrid,
            "grid1",
        	 0) 
        ## The different occupancy grids may of different sizes but are assumed to be local centred???
        self.grid2_pub = self.create_publisher(
            OccupancyGrid,
            "grid2",
            0
        )
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        grid1, grid2  = self.make_grid(), self.make_grid()
        grid1.header.stamp = self.get_clock().now().to_msg()
        grid2.header.stamp = self.get_clock().now().to_msg()
        
        self.grid1_pub.publish(grid1)
        self.grid2_pub.publish(grid2)
		
    def make_grid(self, info:MapMetaData=None) -> OccupancyGrid:
        grid = OccupancyGrid()

        data = np.ones((51, 51), dtype='int8')*50
        grid.data = Array('b', data.ravel().astype(np.int8))
        grid.header.frame_id = "base_link"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = info or MapMetaData()
        grid.info.width = data.shape[0]
        grid.info.height = data.shape[1]

        return grid


def main(args=None):
    rclpy.init(args=args)

    pub = OccupancyPub()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


