import numpy as np
import cv2
from array import array as Array

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import time

def frame_processor(input_image):
	cv2.imwrite("input_image.png", input_image)
	kernel_size = 5
	blur = cv2.GaussianBlur(input_image, (kernel_size, kernel_size), 0)
	# Thresholds for the hysteresis procedure
	low_t = 50
	high_t = 150

	# Edge detection
	edges = cv2.Canny(blur, low_t, high_t)
	
	# Select the region of interest
	region_image, region = region_selection(edges)

	# Apply hough transform to get straight lines
	hough = hough_transform(region_image)
	if hough is None:
		return input_image, np.zeros((1000, 2500)), np.zeros((1000, 2500))

	# Draw lines for visualization
	image_with_lines, line_image = draw_lane_lines(input_image, lane_lines(input_image, hough))
	cv2.imwrite("test.png", line_image)
	image_with_lines_birdseye = create_birdseye_view(image_with_lines, region)
	line_image_birdseye = create_birdseye_view(line_image, region)
	return image_with_lines, image_with_lines_birdseye, line_image_birdseye

def region_selection(image):
	# Create empty array of shape image
	mask = np.zeros_like(image)

	# If you pass an image with more then one channel
	if len(image.shape) > 2:
		channel_count = image.shape[2]
		ignore_mask_color = (255,) * channel_count
	else:
		ignore_mask_color = 255
	
	# Define the region of interest
	rows, cols = image.shape[:2]
	bottom_left = [cols * 0.1, rows]
	top_left	 = [cols * 0.4, rows * 0.7]
	bottom_right = [cols * 0.9, rows]
	top_right = [cols * 0.6, rows * 0.7]
	vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)

	# Fill the region of interest with white color
	cv2.fillPoly(mask, vertices, ignore_mask_color)

	# performing Bitwise AND on the input image and mask to get only the edges on the road
	masked_image = cv2.bitwise_and(image, mask)
	return masked_image, [bottom_left, top_left, top_right, bottom_right]

def hough_transform(image):
	# Distance resolution of the accumulator in pixels.
	rho = 1			
	# Angle resolution of the accumulator in radians.
	theta = np.pi/180
	# Only lines that are greater than threshold will be returned.
	threshold = 40	
	# Line segments shorter than that are rejected.
	minLineLength = 40
	# Maximum allowed gap between points on the same line to link them
	maxLineGap = 400	
	# function returns an array containing dimensions of straight lines 
	# appearing in the input image
	return cv2.HoughLinesP(image, rho = rho, theta = theta, threshold = threshold,
						minLineLength = minLineLength, maxLineGap = maxLineGap)

def average_slope_intercept(lines):
	
	left_lines = [] #(slope, intercept)
	left_weights = [] #(length)
	right_lines = [] #(slope, intercept)
	right_weights = [] #(length)

	for line in lines:
		for x1, y1, x2, y2 in line:
			if x1 == x2:
				continue

			slope = (y2 - y1) / (x2 - x1)
			intercept = y1 - (slope * x1)
			length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
			# Slope of left lane is negative and for right lane slope is positive
			if abs(slope) < 0.2:
				continue
			if slope < 0:
				left_lines.append((slope, intercept))
				left_weights.append((length))
			else:
				right_lines.append((slope, intercept))
				right_weights.append((length))
	
	left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
	right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
	return left_lane, right_lane

def pixel_points(y1, y2, line):
	# Converts the slope and intercept of each line into pixel points.
	if line is None:
		return None
	slope, intercept = line
	x1 = int((y1 - intercept)/slope)
	x2 = int((y2 - intercept)/slope)
	y1 = int(y1)
	y2 = int(y2)
	return ((x1, y1), (x2, y2))

def lane_lines(image, lines):
	# Create full lenght lines from pixel points.
	left_lane, right_lane = average_slope_intercept(lines)
	y1 = image.shape[0]
	y2 = y1 * 0.6
	left_line = pixel_points(y1, y2, left_lane)
	right_line = pixel_points(y1, y2, right_lane)
	return left_line, right_line
	
def draw_lane_lines(image, lines, color=[255, 255, 255], thickness=5):
	# Draw lines onto the image.
	line_image = np.zeros_like(image, dtype=np.uint8)
	for line in lines:
		if line:
			cv2.line(line_image, *line, color, thickness)

	image_with_lines = cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)
	
	return image_with_lines, line_image

def create_birdseye_view(image, region):
	# Define the region of interest
	roi = np.float32(region)
	# Define the desired output size
	length = 100*10
	width = 2500*10
	dim = np.float32([[0, width], [0, 0], [length, 0], [length, width]])
	# Compute the perspective transform matrix
	matrix = cv2.getPerspectiveTransform(roi, dim)
	# Apply the perspective transform
	birdseye = cv2.warpPerspective(image, matrix, (length, width//10))
	
	return birdseye

def convert_to_occupancy_grid(image, grid_resolution):
	height, width = image.shape[:2]

	grid_height = int(np.ceil(height / grid_resolution))
	grid_width = int(np.ceil(width / grid_resolution))

	occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.int8)

	for i in range(grid_height):
		for j in range(grid_width):
			y1 = i * grid_resolution
			y2 = min((i + 1) * grid_resolution, height)
			x1 = j * grid_resolution
			x2 = min((j + 1) * grid_resolution, width)

			if np.any(image[y1:y2, x1:x2] == 255):
				occupancy_grid[i, j] = 100
			else:
				occupancy_grid[i, j] = 0

	return occupancy_grid

class OccupancyGridPublisher(Node):
	def __init__(self):
		super().__init__('occupancy_grid_publisher')
		self.publisher_ = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

		self.br = CvBridge()

		video_qos = QoSProfile(
			history = QoSHistoryPolicy.KEEP_LAST,
			depth = 5,
			reliability = QoSReliabilityPolicy.RELIABLE,
			durability = QoSDurabilityPolicy.VOLATILE
		)
		self.subscription = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.publish_grid, qos_profile=video_qos)
	
	def publish_grid(self, data):
		image = self.br.imgmsg_to_cv2(data)
		image_with_lines, image_with_lines_birdseye, line_image_birdseye = frame_processor(image)
		cv2.imwrite("image.png", line_image_birdseye)
		occupancy_grid = convert_to_occupancy_grid(line_image_birdseye, 50)
		msg = OccupancyGrid()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'map'
		msg.info.resolution = 0.05
		msg.info.width = occupancy_grid.shape[1]
		msg.info.height = occupancy_grid.shape[0]
		msg.data = Array('b', occupancy_grid.ravel().astype(np.int8))
		self.publisher_.publish(msg)
		time.sleep(0.2)

def main():
	rclpy.init()
	occupancy_grid_publisher = OccupancyGridPublisher()
	rclpy.spin(occupancy_grid_publisher)
	occupancy_grid_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
