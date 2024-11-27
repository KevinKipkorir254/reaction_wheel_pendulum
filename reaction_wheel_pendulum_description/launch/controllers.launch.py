from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                )
    
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controllers", "--controller-manager", "/controller_manager"],
    )

    nodes = [
                    joint_state_broadcaster_spawner,
                    robot_controller_spawner,
                ]

    return LaunchDescription(nodes)