#!/usr/bin/env python3
"""
RoboCollector Simulation Launch File
Tests vision and controller with simulated camera and mock arm
"""

import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):

    # Simulation test node (publishes fake camera images)
    sim_camera = Node(
        package="robocollector_vision",
        executable="simulation_test_node",
        name="simulation_camera",
        output="screen",
        emulate_tty=True,
    )

    # Vision node (detects colors from simulated camera)
    vision_node = Node(
        package="robocollector_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[
            {
                "debug": True,  # Enable OpenCV window
                "target_color": "red",
                "min_area": 500,
            }
        ],
        emulate_tty=True,
    )

    # Mock pickup node (simulates arm movements)
    mock_pickup = Node(
        package="robocollector_manipulation",
        executable="mock_pickup_node",
        name="pickup_node",  # Same name as real node for compatibility
        output="screen",
        emulate_tty=True,
    )

    # Controller node (runs full state machine)
    controller_node = Node(
        package="robocollector_controller",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{"target_color": "red"}],
        emulate_tty=True,
    )

    return [sim_camera, vision_node, mock_pickup, controller_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
