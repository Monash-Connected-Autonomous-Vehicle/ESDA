import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
    # TODO pass custom config into swiftnav
    swiftnav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swiftnav_ros2_driver'), 'launch'),
           '/start.py'])
        )

    # TODO pass custom config into driver
    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_driver'), 'launch'),
            '/velodyne_driver_node-VLP16-launch.py'])
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
