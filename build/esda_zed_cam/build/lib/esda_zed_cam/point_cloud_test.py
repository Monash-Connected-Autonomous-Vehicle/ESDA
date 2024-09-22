import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import Image
import cv2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')

        # self.subscription = self.create_subscription(
        #     PointCloud2,
        #     '/zed/zed_node/point_cloud/cloud_registered',
        #     self.pc_callback,
        #     10)
        # self.subscription  # prevent unused variable warning

        video_qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()

        self.camera_subscription = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.image_display, qos_profile = video_qos)

    def pc_callback(self,msg):
        ## Reading the data points from the point cloud topic, converting it into a (height x width) by 1 array
        data = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
        # Initializing a points array with size height by width
        # At 720p it's 360, 640 and at 1080p it's 540, 960
        points = data.reshape(360, 640, 3)

        ## Retriving points of interest to find BASE_WIDTH, WIDTH AND LENGTH
        # For BASE_WIDTH
        bottom_left = points[349][10]
        bl_x = bottom_left[0]
        bl_y = bottom_left[1]
        bl_z = bottom_left[2]
        bottom_right = points[349][629]
        br_x = bottom_right[0]
        br_y = bottom_right[1]
        br_z = bottom_right[2]

        base_width = np.sqrt((br_x - bl_x)*(br_x - bl_x)+ (br_y-bl_y)*(br_y-bl_y) + (br_z - bl_z)*(br_z - bl_z))

        # For WIDTH
        top_left = points[10][10]
        tl_x = top_left[0]
        tl_y = top_left[1]
        tl_z = top_left[2]
        top_right = points[10][629]
        tr_x = top_right[0]
        tr_y = top_right[1]
        tr_z = top_right[2]

        width = np.sqrt((tr_x - tl_x)*(tr_x - tl_x)+ (tr_y-tl_y)*(tr_y-tl_y) + (tr_z - tl_z)*(tr_z - tl_z))

        # For LENGTH
        bottom_center = points[349][319]
        bc_x = bottom_center[0]
        bc_y = bottom_center[1]
        bc_z = bottom_center[2]
        top_center = points[10][319]
        tc_x = top_center[0]
        tc_y = top_center[1]
        tc_z = top_center[2]

        length = np.sqrt((bc_x - tc_x)*(bc_x - tc_x)+ (bc_y-tc_y)*(bc_y-tc_y) + (bc_z - tc_z)*(bc_z - tc_z))

        print([base_width, width, length])
        return base_width, width, length

    def image_display(self,data):
        frame = self.bridge.imgmsg_to_cv2(data)
        # bottom left marker
        cv2.circle(frame, (10, 349), 5, (0, 0, 255), -1)
        # bottom right marker
        cv2.circle(frame, (629, 349), 5, (0, 0, 255), -1)
        # top left marker
        cv2.circle(frame, (10, 10), 5, (0, 0, 255), -1)
        # top right marker
        cv2.circle(frame, (629, 10), 5, (0, 0, 255), -1)
        # bottom center marker
        cv2.circle(frame, (319, 349), 5, (0, 0, 255), -1)
        # top center marker
        cv2.circle(frame, (319, 10), 5, (0, 0, 255), -1)

        cv2.imshow("image",frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
