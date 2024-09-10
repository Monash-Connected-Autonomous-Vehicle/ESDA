import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
import launch_ros.actions
import yaml
import pathlib
import launch.actions


def generate_launch_description():
    package_name='esda_launch'
    launch_configs_dir = os.path.join(package_name, 'config')
    loc_parameters_file_path = os.path.join(launch_configs_dir, 'robot_loc_dual_efk_config.yaml')
    os.environ['FILE_PATH'] = str(launch_configs_dir) 
    
    # obtained from https://github.com/joshnewans/articubot_one
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                     launch_arguments={'use_sim_time': 'true'}.items() 
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'esda'],
                        output='screen')

    # Launch localisation nodes obtained from https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel
    localization_nodes = [
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'
        ),
            
        launch.actions.DeclareLaunchArgument(
            'output_location',
	        default_value='~/dual_ekf_navsat_example_debug.txt'
	    ),
	    
        launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[loc_parameters_file_path],
            remappings=[('imu/data', 'imu'),
                        ('odometry/filtered', 'odometry/local')
        ]),
               
        launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[loc_parameters_file_path],
            remappings=[('imu/data', 'imu'),
                        ('gps/fix', 'navsatfix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')
        ]),    
                      
        launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[loc_parameters_file_path],
            remappings=[('imu/data', 'imu'),
                        ('gps/fix', 'navsatfix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')
        ])           
    ]
    
    ld = [rsp, gazebo, spawn_entity] + localization_nodes

    # Launch them all!
    return LaunchDescription(ld)

