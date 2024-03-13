import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
    swiftnav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('esda_launch'), 'launch'),
           '/piksi_launch.py'])
        )

    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('esda_launch'), 'launch'),
            '/lidar_launch.py'])
        )

    velodyne_pcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_pointcloud'), 'launch'),
            '/velodyne_transform_node-VLP16-launch.py'])
        )
      
    velodyne_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_laserscan'), 'launch'),
            '/velodyne_laserscan_node-launch.py'])
        )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py']),
        launch_arguments={'camera_model': 'zed2'}.items(),
    )

    return LaunchDescription([
        swiftnav_launch,
        velodyne_driver_launch,
        velodyne_pcl_launch,
        velodyne_scan_launch,
        zed_wrapper_launch
    ])
