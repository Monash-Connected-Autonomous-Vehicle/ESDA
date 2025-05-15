from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esda_pymavlink',
            executable='pymavlink_driver',
            name='pymavlink_driver',
            output='screen'
        )
    ])
