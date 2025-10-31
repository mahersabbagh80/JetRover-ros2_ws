#!/usr/bin/env python3
"""
Launch file for RoboCollector Vision Node
Follows JetRover dual-environment pattern
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):
    # Dual environment setup (development vs production)
    compiled = os.environ.get("need_compile", "False")
    if compiled == "True":
        package_path = get_package_share_directory("robocollector_vision")
    else:
        package_path = "/home/maher/JetRover/ros2_ws/robocollector/robocollector_vision"

    config_file = os.path.join(package_path, "config", "vision_params.yaml")

    # Vision node
    vision_node = Node(
        package="robocollector_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[config_file],
        emulate_tty=True,
    )

    return [vision_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
