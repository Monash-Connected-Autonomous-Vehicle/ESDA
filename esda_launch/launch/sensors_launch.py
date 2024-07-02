import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    swiftnav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swiftnav-ros2'), 'launch'),
           '/start.py']
        )
    )

    velodyne_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne'), 'launch'),
            '/velodyne-all-nodes-VLP16-launch.py']
        )
    )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch'),
            '/zed_camera.launch.py']
        ),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    return LaunchDescription([
        swiftnav_launch,
        velodyne_nodes_launch,
        zed_wrapper_launch
    ])
