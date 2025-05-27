from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/velodyne_voxel_filtered'),
                ('scan', '/laserscan')
            ],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,  # fixed typo from "incrment"
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 4.0,
                'use_inf': True,
                'concurrency_level': 1
            }]
        )
    ])
