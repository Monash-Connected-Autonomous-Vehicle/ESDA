def main():
#NOTE: Just following tutorial -> http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_birdseye/

#To run this node, go to ros2_ws in lidar directory, run: |colcon build --packages-select lidarbev|.
#Open a new terminal and run |source install/local_setup.bash| and |ros2 run lidarbev lidar_bev_node| where you should see the hi message. if you update the file, save, colcon build it again then ros2 run it again

	print('Hi from lidarbev... ')

	import numpy as np
	# Specifying region of the point cloud in metres
	side_range=(-10, 10)     # left-most to right-most
	fwd_range=(0, 20)       # back-most to forward-most
	
	# EXTRACT THE POINTS FOR EACH AXIS
	x_points = points[:, 0]
	y_points = points[:, 1]
	z_points = points[:, 2]

# FILTER - To return only indices of points within desired cube
# Three filters for: Front-to-back, side-to-side, and height ranges
# Note left side is positive y axis in LIDAR coordinates
f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
filter = np.logical_and(f_filt, s_filt)
indices = np.argwhere(filter).flatten()

# KEEPERS
x_points = x_points[indices]
y_points = y_points[indices]
z_points = z_points[indices]

if __name__ == '__main__':
	main()
