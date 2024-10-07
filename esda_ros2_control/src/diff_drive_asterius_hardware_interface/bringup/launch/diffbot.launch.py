# # # Copyright 2020 ros2_control Development Team
# # #
# # # Licensed under the Apache License, Version 2.0 (the "License");
# # # you may not use this file except in compliance with the License.
# # # You may obtain a copy of the License at
# # #
# # #     http://www.apache.org/licenses/LICENSE-2.0
# # #
# # # Unless required by applicable law or agreed to in writing, software
# # # distributed under the License is distributed on an "AS IS" BASIS,
# # # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # # See the License for the specific language governing permissions and
# # # limitations under the License.

# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument, RegisterEventHandler
# # from launch.conditions import IfCondition
# # from launch.event_handlers import OnProcessExit
# # from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

# # from launch_ros.actions import Node
# # from launch_ros.substitutions import FindPackageShare


# # def generate_launch_description():
# #     # Declare arguments
# #     declared_arguments = []
# #     declared_arguments.append(
# #         DeclareLaunchArgument(
# #             "gui",
# #             default_value="true",
# #             description="Start RViz2 automatically with this launch file.",
# #         )
# #     )
# #     declared_arguments.append(
# #         DeclareLaunchArgument(
# #             "use_mock_hardware",
# #             default_value="false",
# #             description="Start robot with mock hardware mirroring command to its states.",
# #         )
# #     )

# #     # Initialize Arguments
# #     gui = LaunchConfiguration("gui")
# #     use_mock_hardware = LaunchConfiguration("use_mock_hardware")

# #     # Get URDF via xacro
# #     robot_description_content = Command(
# #         [
# #             PathJoinSubstitution([FindExecutable(name="xacro")]),
# #             " ",
# #             PathJoinSubstitution(
# #                 [FindPackageShare("diff_drive_asterius_hardware_interface"), "description", "urdf", "diffbot.urdf.xacro"]
# #             ),
# #             " ",
# #             "use_mock_hardware:=",
# #             use_mock_hardware,
# #         ]
# #     )
# #     robot_description = {"robot_description": robot_description_content}

# #     robot_controllers = PathJoinSubstitution(
# #         [
# #             FindPackageShare("diff_drive_asterius_hardware_interface"),
# #             "config",
# #             "diffbot_controllers.yaml",
# #         ]
# #     )
# #     rviz_config_file = PathJoinSubstitution(
# #         [FindPackageShare("diff_drive_asterius_hardware_interface_description"), "diffbot/rviz", "diffbot.rviz"]
# #     )

# #     control_node = Node(
# #         package="controller_manager",
# #         executable="ros2_control_node",
# #         parameters=[robot_controllers],
# #         output="both",
# #         remappings=[
# #             ("~/robot_description", "/robot_description"),
# #             ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
# #         ],
# #     )
# #     robot_state_pub_node = Node(
# #         package="robot_state_publisher",
# #         executable="robot_state_publisher",
# #         output="both",
# #         parameters=[robot_description],
# #     )
# #     rviz_node = Node(
# #         package="rviz2",
# #         executable="rviz2",
# #         name="rviz2",
# #         output="log",
# #         arguments=["-d", rviz_config_file],
# #         condition=IfCondition(gui),
# #     )

# #     joint_state_broadcaster_spawner = Node(
# #         package="controller_manager",
# #         executable="spawner",
# #         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
# #     )

# #     robot_controller_spawner = Node(
# #         package="controller_manager",
# #         executable="spawner",
# #         arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
# #     )

# #     # Delay rviz start after `joint_state_broadcaster`
# #     delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
# #         event_handler=OnProcessExit(
# #             target_action=joint_state_broadcaster_spawner,
# #             on_exit=[rviz_node],
# #         )
# #     )

# #     # Delay start of joint_state_broadcaster after `robot_controller`
# #     # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
# #     delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
# #         event_handler=OnProcessExit(
# #             target_action=robot_controller_spawner,
# #             on_exit=[joint_state_broadcaster_spawner],
# #         )
# #     )

# #     nodes = [
# #         control_node,
# #         robot_state_pub_node,
# #         robot_controller_spawner,
# #         delay_rviz_after_joint_state_broadcaster_spawner,
# #         delay_joint_state_broadcaster_after_robot_controller_spawner,
# #     ]

# #     return LaunchDescription(declared_arguments + nodes)

# # Copyright 2020 ros2_control Development Team
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# from launch import LaunchDescription
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def generate_launch_description():
#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [FindPackageShare("diff_drive_asterius_hardware_interface"), "description", "urdf", "diffbot.urdf.xacro"]
#             ),
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     robot_controllers = PathJoinSubstitution(
#         [
#             FindPackageShare("diff_drive_asterius_hardware_interface"),
#             "config",
#             "diffbot_controllers.yaml",
#         ]
#     )
#     rviz_config_file = PathJoinSubstitution(
#         [FindPackageShare("ros2_control_demo_description"), "rviz", "diffbot.rviz"]
#     )

#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description, robot_controllers],
#         output="both",
#     )
#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#         remappings=[
#             ("/diff_drive_asterius_controller/cmd_vel_unstamped", "/cmd_vel"),
#         ],
#     )
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#     )

#     robot_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["diff_drive_asterius_controller", "--controller-manager", "/controller_manager"],
#     )

#     # Delay rviz start after `joint_state_broadcaster`
#     delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=joint_state_broadcaster_spawner,
#             on_exit=[rviz_node],
#         )
#     )

#     # Delay start of robot_controller after `joint_state_broadcaster`
#     delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=joint_state_broadcaster_spawner,
#             on_exit=[robot_controller_spawner],
#         )
#     )

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_description}],
#         output='screen'
#     )


#     nodes = [
#         control_node,
#         robot_state_pub_node,
#         joint_state_broadcaster_spawner,
#         delay_rviz_after_joint_state_broadcaster_spawner,
#         delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
#     ]

#     return LaunchDescription(nodes)

# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ros2_control_demo_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="diffbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_2"), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "diffbot/rviz", "diffbot_view.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
