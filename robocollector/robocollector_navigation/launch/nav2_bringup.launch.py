import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'True')
    if compiled == 'True':
        pkg_path = get_package_share_directory('robocollector_navigation')
    else:
        pkg_path = '/home/ubuntu/ros2_ws/robocollector/robocollector_navigation'

    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_path, 'launch', 'bringup_launch.py')

    map_yaml = LaunchConfiguration('map').perform(context)

    launch_args = {
        'use_sim_time': 'false',
        'params_file': params_file,
    }
    if map_yaml and len(map_yaml) > 0:
        launch_args['map'] = map_yaml

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments=launch_args.items()
    )

    return [nav2]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
