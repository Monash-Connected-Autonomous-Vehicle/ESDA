from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    servo_interface = Node(
            package='dynamixel_sdk_examples',
            executable='read_write_node',
            name='dynamixel_sdk_node',
            output='screen'
        )
        
    steering_controller = Node(
            package='esda_control',
            executable='parallelogram_steering_controller', 
            name='parallelogram_steering_controller',
            output='screen'
        )
        
    mcu_interface = Node(
            package='esda_control',
            executable='mcu_interface', 
            name='mcu_interface',
            output='screen'
        )

    return LaunchDescription([
        servo_interface,
        steering_controller,
        mcu_interface
    ])
