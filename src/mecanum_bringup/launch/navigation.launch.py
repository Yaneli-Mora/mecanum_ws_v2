import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('mecanum_bringup')
    nav2_dir    = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # Nav2 stack (Jazzy compatible)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
              'params_file':   nav2_params,
              'use_sim_time':  'false',
              'autostart':     'true',
            }.items()),

        # Position verifier
        Node(package='position_verifier', executable='position_verifier_node'),

        # Plow controller
        Node(package='duck_operations', executable='plow_controller_node'),
    ])
