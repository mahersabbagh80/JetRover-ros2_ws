import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'True')
    if compiled == 'True':
        pkg_path = get_package_share_directory('robocollector_navigation')
    else:
        pkg_path = '/home/ubuntu/ros2_ws/navigation/robocollector_navigation'

    params_file = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    return [slam]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
