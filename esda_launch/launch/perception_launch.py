import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py'])
        )

    # lane_detection_launch

    # pothole_detection_launch
  
    return LaunchDescription([
        slam_toolbox_launch
    ])
