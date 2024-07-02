import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
   # TODO remove based on where it is launched elsewhere
   zed_wrapper_launch = IncludeLaunchDescription(
       PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('zed_wrapper'), 'launch'),
           '/zed_camera.launch.py']
       ),
       launch_arguments={'camera_model': 'zed2i'}.items()
   )

   rsp_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('esda_sim'),'launch','rsp.launch.py'
      )]), launch_arguments={'use_sim_time': 'true'}.items()
   )
   
   sensors_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('esda_launch'), 'launch'),
         '/sensors_launch.py'])
   )
   
   localization_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('esda_launch'), 'launch'),
         '/localization_launch.py'])
   )
   
   perception_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('esda_launch'), 'launch'),
         '/perception_launch.py'])
   )

   # planning launch = ...
   
   # control launch = ...
   
   return LaunchDescription([
      zed_wrapper_launch,
      rsp_launch,
      sensors_launch,
      localization_launch,
      perception_launch
   ])
