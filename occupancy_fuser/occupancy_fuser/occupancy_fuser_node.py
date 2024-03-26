from array import array as Array
import scipy as sp
from nav_msgs.msg import OccupancyGrid, MapMetaData
import rclpy
from rclpy.node import Node
import numpy as np

class OccupancyFuser(Node):
    def __init__(self):
        super().__init__('occupancy_fuser')
        self.grid1_sub= self.create_subscription(
            OccupancyGrid,
            "grid1",
            self._update_callback, 1)
        
        ## The different occupancy grids may of different sizes but are assumed to be local centred???
        self.grid2_sub = self.create_subscription(
            OccupancyGrid,
            "grid2",
            self._update_callback,
            1
        )
        self.pub = self.create_publisher(OccupancyGrid,
                                         "/grid_fused",
                                         1)
        self.timer = self.create_timer(1, self.timer_callback)
        self.recent_grids = dict()

    def _update_callback(self, msg: OccupancyGrid):
        metadata: MapMetaData = msg.info
        if not (metadata.height % 2 == 1 and metadata.width % 2 ==1):
            self.get_logger().warn(f"Occupancy grid from {msg.header.frame_id} has no centre, ignoring")

        self.get_logger().info(f"Updating grid from frame {msg.header.frame_id}")
        self.recent_grids[msg.header.frame_id] = msg 

    def timer_callback(self):
        if len(self.recent_grids) == 0:
            self.get_logger().info("Not ready, no occupancy grids received")
        if not self.recent_grids:
            return
        grid = OccupancyGrid()
        grid.header.frame_id = "base_link"
        grid.header.stamp = self.get_clock().now().to_msg()

        width = max(self.recent_grids.values(), key=lambda x: x.info.width).info.width
        height = max(self.recent_grids.values(), key=lambda x:x.info.height).info.height
        data = np.zeros((width, height))
        # Create a kernel which has a single island in the centre, use this to splat all submaps into main map
        centre_kernel = np.zeros(data.shape)
        centre_kernel[width//2+1, height//2+1] = 1
        for map in self.recent_grids.values():
            if self.get_clock().now().seconds_nanoseconds()[0] - map.header.stamp.sec > 1:
                self.get_logger().info(f"Discarding old map data from frame {map.header.frame_id}")
                continue
            map_data = np.array(map.data)
            map_data.shape = (map.info.width, map.info.height)
            data += sp.signal.convolve2d(centre_kernel, map_data, mode='same')
        grid.data = Array('b', map_data.ravel().astype(np.int8))
        grid.info = MapMetaData()
        grid.info.height = height
        grid.info.width = width
        self.pub.publish(grid)
        


def main(args=None):
    rclpy.init(args=args)

    fuser = OccupancyFuser()

    rclpy.spin(fuser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fuser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

