import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

import numpy as np

class ESDACameraInterface(Node):
    def __init__(self):
        super().__init__('esda_camera_interface')

        # Topic variables (For easier input, and change the bit after zed_node if you want a different camera topic)
        topic_name = '/zed/zed_node/left/image_rect_color'

        #Defining QoS Profile
        video_qos = QoSProfile(
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.VOLATILE
        )
        
        #Defining the OpenCV bridge
        self.br = CvBridge()

        # Creating the subscriber node
        self.subscription = self.create_subscription(Image, topic_name, self.lane_detection, qos_profile=video_qos)

    # This function is meant for debugging to ensure the camera's original video feed
    def img_callback(self, data):
        # self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)   
        cv2.waitKey(1)
    
    # Main function for lane detection
    def lane_detection(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        
        #Applying grayscale to the image
        gray_image = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)

        #Converting image from RGB to HSV (Hue, Saturation, Value)
        img_hsv = cv2.cvtColor(current_frame, cv2.COLOR_RGB2HSV)

        #Defining yellow values
        lower_yellow = np.array([20, 100, 100], dtype = "uint8")
        upper_yellow = np.array([30, 255, 255], dtype="uint8")

        #Masking the image to show only colors of focus
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(gray_image, 200, 255)
        mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
        mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)

        #Applying Gaussian Blur
        kernel_size = 5
        gauss_gray = cv2.GaussianBlur(mask_yw_image, (kernel_size, kernel_size), 0)

        # img_threshold = np.zeros_like(current_frame)
        # img_threshold [(current_frame >= 180)] = 255

        cv2.imshow("thresholding",gauss_gray)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    esda_camera_interface = ESDACameraInterface()
    rclpy.spin(esda_camera_interface)
    esda_camera_interface.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()