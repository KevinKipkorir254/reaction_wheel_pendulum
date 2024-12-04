#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)


def generate_launch_description():
    map_package = get_package_share_directory("reaction_wheel_pendulum_description")
    world_file = PathJoinSubstitution([map_package, "worlds", "empty_with_plugins.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
    )

    # Environment Variable
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.join(get_package_prefix("reaction_wheel_pendulum_description"), "share")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("reaction_wheel_pendulum_description"),
                    "reaction_wheel_pendulum",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "cart_pole",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
             "-x",
             "0.0",
             "-y",
             "0.0",
             "-z",
             "0.2",
        ],
        output="screen",
    )

    launch_everything = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("reaction_wheel_pendulum_description"),
                    "launch",
                    "all.launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from Gazebo)
            SetParameter(name="use_sim_time", value=True),
            declare_world_arg,
            robot_state_pub_node,
            gz_sim,
            #gz_bridge,
            gz_spawn_entity,
            launch_everything,
        ]
    )