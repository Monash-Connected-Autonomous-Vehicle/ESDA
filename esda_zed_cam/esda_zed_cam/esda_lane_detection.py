import cv2
import numpy as np
from util import Info
from bev_tools import transform_to_bev 
from lanedet_tools import generate_lane_mask

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def ensure_3channel_uint8(img):
    """Convert grayscale or BGRA images to 3-channel BGR uint8."""
    if img.ndim == 2:  # Grayscale
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif img.shape[2] == 4:  # BGRA
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img.astype(np.uint8)

class ZEDLaneDetection(Node):
    def __init__(self):
        super().__init__('zed_lane_depth_detector')
        
        # Bridge for converting ROS images to OpenCV
        self.br = CvBridge()
        
        # ZED image susbcriber
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )

    def image_callback(self,msg):
        # Obtain frame from camera
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        
        # Debugging purposes, test whether the frame is being displayed
        # cv2.imshow("camera", frame)
        # cv2.waitKey(1)

        height, width = frame.shape[:2]

        # Define camera parameters
        camera_params = {
            "focalLengthX": 954.76,
            "focalLengthY": 955.065,
            "opticalCenterX": width // 2,
            "opticalCenterY": height // 2,
            "cameraHeight": 48.3,
            "pitch": 0,
            "yaw": 0,
            "roll": 0
        }
        cameraInfo = Info(camera_params)

        # Define IPM parameters
        ipm_params = {
            "left": 100,
            "right": width - 100,
            "top": 260,
            "bottom": height
        }
        ipmInfo = Info(ipm_params)

        # Define HSV thresholds for lane detection (tune as needed)
        lower = np.array([0, 0, 200])
        upper = np.array([255, 50, 255])

        # Target size for display
        target_size = (640, 360)

        # BEV transformation
        bevImg = transform_to_bev(frame, cameraInfo, ipmInfo)

        # Generate lane masks
        mask, msk = generate_lane_mask(bevImg, lower, upper)

        # Convert all to 3-channel BGR and uint8
        frame_bgr = ensure_3channel_uint8(frame)
        bev_bgr = ensure_3channel_uint8(bevImg)
        mask_bgr = ensure_3channel_uint8(mask)
        msk_bgr = ensure_3channel_uint8(msk)

        # Resize to uniform size
        frame_resized = cv2.resize(frame_bgr, target_size)
        bev_resized = cv2.resize(bev_bgr, target_size)
        mask_resized = cv2.resize(mask_bgr, target_size)
        msk_resized = cv2.resize(msk_bgr, target_size)

        # Create 2x2 grid
        top_row = cv2.hconcat([frame_resized, bev_resized])
        bottom_row = cv2.hconcat([mask_resized, msk_resized])
        grid = cv2.vconcat([top_row, bottom_row])

        # Show grid
        cv2.imshow("Lane Detection Grid", grid)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ZEDLaneDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()