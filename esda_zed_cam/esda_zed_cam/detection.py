import numpy as np
import cv2
from array import array as Array

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

        self.br = CvBridge()

        video_qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE
        )

        self.camera_subscription = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.publish_grid, qos_profile = video_qos)
    
    def publish_grid(self, data):
        occupancy_grid = self.frame_processor(data)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1 # Grid size (meters)
        msg.info.width = occupancy_grid.shape[1]
        msg.info.height = occupancy_grid.shape[0]
        msg.data = Array('b', occupancy_grid.ravel().astype(np.int8))
        self.occupancy_grid_publisher.publish(msg)


    def frame_processor(self, data):
        # Convert data to black and white image for road and obstacles
        image = self.br.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_road_colour = np.array([0, 0, 0])
        upper_road_colour = np.array([179, 255, 100])
        lower_ramp_green_colour = np.array([40, 40, 40])
        upper_ramp_green_colour = np.array([80, 255, 255])

        mask_road = cv2.inRange(hsv, lower_road_colour, upper_road_colour)
        mask_ramp = cv2.inRange(hsv, lower_ramp_green_colour, upper_ramp_green_colour)

        combined_mask = cv2.bitwise_or(mask_road, mask_ramp)
        mask = cv2.bitwise_and(image, image, mask = combined_mask)

        # Convert black and white image to birds eye view 
        rows, cols = image.shape[:2]
        bottom_left = [cols * 0.1, rows]
        top_left = [cols * 0.4, rows * 0.7]
        bottom_right = [cols * 0.9, rows]
        top_right = [cols * 0.6, rows * 0.7]

        src = np.float32([bottom_left, top_left, bottom_right, top_right])

        # 1 pixel for 1cm
        WIDTH = 1000
        LENGTH = 1000
        dst = np.float32([[0, WIDTH], [0, 0], [LENGTH, WIDTH], [LENGTH, 0]])

        matrix = cv2.getPerspectiveTransform(src, dst)

        birdseye_view = cv2.warpPerspective(mask, matrix, (LENGTH, WIDTH))

        # Create occupancy grid from black and white birds eye view
        OCCUPANCY_WIDTH = 10
        OCCUPANCY_HEIGHT = 10

        grid_width = image.shape[1] // OCCUPANCY_WIDTH
        grid_height = image.shape[0] // OCCUPANCY_HEIGHT

        occupancy_grid = np.zeros((OCCUPANCY_HEIGHT, OCCUPANCY_WIDTH), dtype=np.uint8)

        for i in range(OCCUPANCY_HEIGHT):
            for j in range(OCCUPANCY_WIDTH):
                roi = birdseye_view[i * grid_height: (i + 1) * grid_height,
                                    j * grid_width: (j + 1) * grid_width]
                
                if np.any(roi == 255):
                    occupancy_grid[i, j] = 1

        return occupancy_grid

def main():
    rclpy.init()
    occupancy_grid_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_grid_publisher)
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()