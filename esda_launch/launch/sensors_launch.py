import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
   swiftnav_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('swiftnav_ros2_driver'), 'launch'),
         '/start.py'])
      )

   velodyne_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('velodyne_driver'), 'launch'),
         '/velodyne_driver_node-VLP16-launch.py'])
      )


    # todo Z Cam launch
    

   return LaunchDescription([
      swiftnav_launch,
      velodyne_launch
   ])
