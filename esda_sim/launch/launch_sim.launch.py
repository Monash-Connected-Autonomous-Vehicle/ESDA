import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition


def generate_launch_description():
    # obtained from https://github.com/joshnewans/articubot_one

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='esda_sim'

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
                        
    '''controllers = LaunchConfiguration('controllers')
                        
    controllers_arg = DeclareLaunchArgument(
        name="controllers",
        default_value=PathJoinSubstitution([
            FindPackageShare('esda_sim'), 
            "config", 
            "ros2_control_config.yaml"
        ]),
        description="Path of the controller params file"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
    )

    ackermann = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont"]
    )

    joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )'''


    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity
    ])
