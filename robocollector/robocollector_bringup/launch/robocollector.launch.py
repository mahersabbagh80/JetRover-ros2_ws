#!/usr/bin/env python3
"""
RoboCollector Main Launch File
Starts all necessary nodes for the block collection mission.
Follows JetRover hierarchical launch pattern.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context):
    # Dual environment setup (development vs production)
    compiled = os.environ.get("need_compile", "False")

    if compiled == "True":
        vision_path = get_package_share_directory("robocollector_vision")
        manipulation_path = get_package_share_directory("robocollector_manipulation")
        controller_path = get_package_share_directory("robocollector_controller")
    else:
        vision_path = "/home/maher/JetRover/ros2_ws/robocollector/robocollector_vision"
        manipulation_path = (
            "/home/maher/JetRover/ros2_ws/robocollector/robocollector_manipulation"
        )
        controller_path = (
            "/home/maher/JetRover/ros2_ws/robocollector/robocollector_controller"
        )

    # Include vision node
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_path, "launch", "vision_node.launch.py")
        ),
    )

    # Include manipulation node
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulation_path, "launch", "pickup_node.launch.py")
        ),
    )

    # Include controller node
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_path, "launch", "controller_node.launch.py")
        ),
    )

    return [vision_launch, manipulation_launch, controller_launch]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
