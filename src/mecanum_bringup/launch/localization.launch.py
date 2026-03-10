import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('mecanum_bringup')
    ekf_cfg     = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    map_yaml    = os.path.join(bringup_dir, 'map', 'arena_map.yaml')

    return LaunchDescription([

        # Odometry publisher (wheel kinematics + TF broadcaster)
        Node(package='mecanum_localization', executable='odometry_publisher',
             parameters=[{
               'wheel_radius':  0.050,
               'wheel_base_x':  0.150,
               'wheel_base_y':  0.170,
             }]),

        # EKF — fuses odom + IMU
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=[ekf_cfg]),

        # White line drift corrector
        Node(package='mecanum_localization', executable='white_line_corrector'),

        # Static map → odom transform (replaces AMCL — no LiDAR needed)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_tf',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),

        # Map server
        Node(package='nav2_map_server', executable='map_server',
             name='map_server',
             parameters=[{'yaml_filename': map_yaml}]),

        # Lifecycle manager for map server
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),

        # Initial pose publisher (2s delay to let EKF start)
        TimerAction(period=2.0, actions=[
            Node(package='mecanum_localization', executable='initial_pose_publisher',
                 parameters=[{
                   'x':        0.15,
                   'y':        0.15,
                   'yaw':      1.5708,
                   'cov_x':    0.01,
                   'cov_y':    0.01,
                   'cov_yaw':  0.05,
                   'delay_ms': 0,
                 }])
        ]),
    ])
