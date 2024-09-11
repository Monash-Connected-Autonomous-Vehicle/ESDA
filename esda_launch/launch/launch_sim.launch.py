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
    loc_parameters_file_path = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'robot_loc_sim.yaml'])
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
    localization_node = launch_ros.actions.Node(
           package='robot_localization',
           executable='ekf_node',
           name='ekf_filter_node',
           output='screen',
           parameters=[loc_parameters_file_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
    
    # launch PCL2 to laserscan converter node
    '''cloud_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_laserscan'), 'launch'),
            '/velodyne_laserscan_node-launch.py']
        )
    )'''
    
    # obstacle detection nodes
    slam_toolbox_params_file = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'sim_slam.yaml'])
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch',
            'online_async_launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'params_file': slam_toolbox_params_file}.items()
    )
    
    # launch planning systems
    nav2_params = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'nav2_sim.yaml'])
    map_file_path = PathJoinSubstitution([FindPackageShare(package_name), 'maps', 'map_config.yaml'])
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch',
            'bringup_launch.py')]),
        launch_arguments={
        	'params_file': nav2_params,
        	'use_sim_time': 'true',
        	'map': map_file_path
        }.items()
    )
       
    ld = [rsp, gazebo, spawn_entity, localization_node, slam_toolbox_launch, nav2]

    # Launch them all!
    return LaunchDescription(ld)

