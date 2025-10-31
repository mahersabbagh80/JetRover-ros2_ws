import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'True')
    map_yaml = os.environ.get('ROBOCOLLECTOR_MAP', '/home/maher/JetRover/ros2_ws/slam/maps/myRoom.yaml')

    if compiled == 'True':
        nav_pkg_path = get_package_share_directory('robocollector_navigation')
    else:
        nav_pkg_path = '/home/ubuntu/ros2_ws/robocollector/robocollector_navigation'

    nav2_launch = os.path.join(nav_pkg_path, 'launch', 'nav2_bringup.launch.py')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_yaml
        }.items()
    )

    return [nav2]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
