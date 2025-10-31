#!/usr/bin/env python3
"""
Launch file for RoboCollector Controller Node
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

    # Controller node
    controller_node = Node(
        package="robocollector_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{"target_color": "red"}],
        emulate_tty=True,
    )

    return [controller_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
