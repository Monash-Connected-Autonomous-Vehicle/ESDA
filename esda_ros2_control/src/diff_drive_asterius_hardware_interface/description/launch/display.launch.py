from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_name = '08-macroed.urdf.xacro'
    urdf = os.path.join(get_package_share_directory('diff_drive_asterius_hardware_interface'), 'description', 'urdf', urdf_file_name)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ),

        Node(
            package='diff_drive_asterius_hardware_interface',
            executable='diffbot_system',
            name='diff_drive_hardware_interface',
            output='screen',
        ),

        Node(
            package='diff_drive_asterius_controller',
            executable='diff_drive_asterius_controller',  # Name of your controller executable
            name='diff_drive_controller',
            output='screen',
        ),
    ])
