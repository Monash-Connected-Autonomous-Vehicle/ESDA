from nav_msgs.msg import OccupancyGrid, MapMetaData
import rclpy
from rclpy.node import Node
import numpy as np

class OccupancyFuser(Node):
    def __init__(self):
        super().__init__('occupancy_fuser')
        self.grid1_sub= self.create_subscription(
            OccupancyGrid,
            "map",
            self._update_callback, 1)
        
        ## The different occupancy grids may of different sizes but are assumed to be local centred???
        self.grid2_sub = self.create_subscription(
            OccupancyGrid,
            "camera_map",
            self._update_callback,
            1
        )
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
        grid = OccupancyGrid()
        grid.header.frame_id = "base_link"
        grid.header.timestamp = self.get_clock().now().to_msg()
        data = np.array(size=(51, 51))
        for map in self.recent_grids:
            if self.get_clock().now() - map.header.timestamp > 1:
                self.get_logger().info(f"Discarding old map data from frame {map.header.frame_id}")
                continue
             map_data = np.array(map.data)



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

