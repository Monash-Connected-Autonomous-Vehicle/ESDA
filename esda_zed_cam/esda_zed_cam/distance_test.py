import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        video_qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            video_qos)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        self.camera_subscription = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.image_display, qos_profile = video_qos)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg)
        # Process depth_image as numpy array
        depth_data = np.array(depth_image)
        # You can now work with depth_data
        # u = int(len(depth_data)/2)
        # v = int(len(depth_data[0])/2)
        # print("Distance at center is: ", depth_data[u][v])
        u = 359
        v = 639
        print("Distance at center is: ", depth_data[u][v])
        return u, v

    def image_display(self,data):
        frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow("image",frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
