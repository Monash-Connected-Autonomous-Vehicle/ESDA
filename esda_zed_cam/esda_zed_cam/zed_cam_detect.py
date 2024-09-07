import numpy as np
import cv2
from array import array as Array

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from geometry_msgs.msg import Quaternion

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.br = CvBridge()

        ### Defining Quality of Service parameters
        ### This must match the ZED ROS node specifications, refer to ZED ROS2 Tutorial online
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

        ### TODO: Change the frame id to something like "Left Camera frame" to relate to transform from rest of ESDA
        msg.header.frame_id = 'map'

        msg.info.resolution = 0.05 # Grid size (meters)

        ### It just works, tried swapping and it didn't work so this is fine.
        ### DON'T TOUCH
        msg.info.width = occupancy_grid.shape[1]
        msg.info.height = occupancy_grid.shape[0]
        msg.data = Array('b', occupancy_grid.ravel().astype(np.int8))
        ### This defines the center position to be in the center of the RVIZ grid
        msg.info.origin.position.x = 3.28
        msg.info.origin.position.y = 2.36
        ### 0.49, 2.46

        ### Quaternion defines a 90 degree rotation about z axis here which aligns the forwards
        ### of the occupancy grid with the positive x axis direction
        q = Quaternion()
        q.w = 0.0
        q.x = -0.707
        q.y = 0.707
        q.z = 0.0
        msg.info.origin.orientation = q
        self.occupancy_grid_publisher.publish(msg)

    def frame_processor(self, data):
        ### Obtaining Image from ZED ROS Node
        frame = self.br.imgmsg_to_cv2(data)

        cv2.imwrite('default.png', frame)

        ## Convert image into HSV
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        ## Creating a mask to detect white (lane-lines and potholes)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 30, 255])

        mask_white = cv2.inRange((frame_hsv), lower_white, upper_white)

        # Defining colors for road or obstacles
        ROAD_COLOUR = [0, 0, 0, 255]
        OBSTACLE_COLOUR = [255, 255, 255, 255]
        UNKNOWN_COLOUR = [0, 0, 0, 0]

        # Initializing mask for only the road
        masked_image = np.full_like(frame, ROAD_COLOUR, dtype=np.uint8)
        # Replacing obstacles found with obstacle color ont he image
        masked_image[mask_white != 0] = OBSTACLE_COLOUR

        ## Defining Region of Interest (ROI)
        y, x = frame.shape[:2]
        bottom_left = [0, y]
        top_left = [0, int(y * 0.2)]
        bottom_right = [x, y]
        top_right = [x, int(y * 0.2)]

        # Defining the array of scanning
        src = np.float32([top_left, top_right, bottom_left, bottom_right])

        ### Defining 1 pixel on occupancy grid as 1cm
        ## Defining the distortion matrix
        BASE_WIDTH = 124
        WIDTH = 463
        LENGTH = 279
        dst = np.float32([[0, 0], [WIDTH, 0], [(WIDTH - BASE_WIDTH)//2, LENGTH], [(WIDTH + BASE_WIDTH)//2, LENGTH]])

        ## Generating matrix for warping ROI
        matrix = cv2.getPerspectiveTransform(src, dst)

        ## Generating image of birds eye view
        birdseye_view = cv2.warpPerspective(masked_image, matrix, (WIDTH, LENGTH))
        cv2.imwrite('birdseye.png', birdseye_view)

        ## The image generated will show some smudges where the obstacle, road and unknown objects blend
        # The script below sets the "smudges" as either obstacles or unknowns
        obstacle_indices = np.any(birdseye_view[:, :, :3] != 0, axis=2)
        birdseye_view[obstacle_indices] = OBSTACLE_COLOUR

        unknown_indices = birdseye_view[:, :, 3] != 255
        birdseye_view[unknown_indices] = UNKNOWN_COLOUR

        ## Generating image after post-processing smudges
        cv2.imwrite('birdseye_postfix.png', birdseye_view)

        ### Creating the occupancy grid
        OCCUPANCY_WIDTH = WIDTH//5
        OCCUPANCY_LENGTH = LENGTH//5

        ## Resize the image so that each pixel is 5cm x 5cm
        resized = cv2.resize(birdseye_view, (OCCUPANCY_WIDTH, OCCUPANCY_LENGTH))
        obstacle_indices = np.any(resized[:, :, :3] != 0, axis=2)
        resized[obstacle_indices] = OBSTACLE_COLOUR

        unknown_indices = resized[:, :, 3] != 255
        resized[unknown_indices] = UNKNOWN_COLOUR

        # Creating array for occupancy grid
        # resized_array = np.full_like(resized[:, :, 0], -1, dtype=int)
        resized_array = np.full_like(resized[:, :, 0], -1, dtype=int)

        # Scan through image to determine pixel color in occupancy grid
        for y in range(resized.shape[0]):
            for x in range(resized.shape[1]):
                if np.array_equal(resized[y, x], ROAD_COLOUR):
                    resized_array[y, x] = 0
                elif np.array_equal(resized[y, x], OBSTACLE_COLOUR):
                    resized_array[y, x] = 100

        return resized_array


def main():
    rclpy.init()
    occupancy_grid_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_grid_publisher)
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()







