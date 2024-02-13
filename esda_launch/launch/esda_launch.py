import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   
   sensors_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('esda_launch'), 'launch'),
         '/sensors_launch.py'])
      )
   
   localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_localization'), 'launch'),
         '/dual_ekf_navsat_example.launch.py'])
      )
   
   # perception launch = ...
   
   # planning launch = ...
   
   # control launch = ...
   
   return LaunchDescription([
      sensors_launch,
      localization_launch
   ])
