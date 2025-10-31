#!/usr/bin/env python3
"""
RoboCollector Gazebo Simulation Launch
Spawns JetRover with colored blocks in Gazebo for full physics testing
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Environment setup
    compiled = os.environ.get("need_compile", "False")

    # Get package paths
    if compiled == "True":
        jetrover_desc_path = get_package_share_directory("jetrover_description")
        robocollector_path = get_package_share_directory("robocollector_bringup")
    else:
        jetrover_desc_path = (
            "/home/maher/JetRover/ros2_ws/simulations/jetrover_description"
        )
        robocollector_path = (
            "/home/maher/JetRover/ros2_ws/robocollector/robocollector_bringup"
        )

    # URDF file
    urdf_path = os.path.join(jetrover_desc_path, "urdf/jetrover.xacro")

    # Set environment variables for xacro
    os.environ["LIDAR_TYPE"] = "A1"  # or A2, G4
    os.environ["MACHINE_TYPE"] = "JetRover_Mecanum"

    # Robot description
    robot_description = Command(["xacro ", urdf_path])

    # World file path
    world_file = os.path.join(robocollector_path, "worlds", "robocollector.world")

    # Launch Gazebo with custom world
    gazebo_launch = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "jetrover",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Blocks and collection box are defined in the world file

    # Vision node
    vision_node = Node(
        package="robocollector_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[{"debug": True, "target_color": "red", "use_sim_time": True}],
    )

    # Pickup node (real one - will try to control Gazebo joints)
    pickup_node = Node(
        package="robocollector_manipulation",
        executable="pickup_node",
        name="pickup_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Controller node
    controller_node = Node(
        package="robocollector_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{"target_color": "red", "use_sim_time": True}],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher,
            spawn_robot,
            vision_node,
            pickup_node,
            controller_node,
        ]
    )
