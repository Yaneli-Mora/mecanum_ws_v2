
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', '..', 'mecanum_description',
        'urdf', 'mecanum_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    # ── Starting pose ─────────────────────────────────────────────────────
    START_X   = 0.15
    START_Y   = 0.15
    START_YAW = 1.5708

    nav2_params = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml')
    arena_map = os.path.join(
        os.path.dirname(__file__), '..', 'map', 'arena_map.yaml')
    ekf_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'ekf.yaml')

    return LaunchDescription([

        # ── Robot description ─────────────────────────────────────────────
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # ── Teensy 1 bridge ───────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='teensy_bridge_node',
             parameters=[{'serial_port': '/dev/teensy'}]),

        # ── Teensy 2 bridge ───────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='teensy2_bridge_node',
             parameters=[{'serial_port': '/dev/teensy2'}]),

        # ── IMU ───────────────────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='imu_node',
             parameters=[{
               'i2c_bus':    '/dev/i2c-1',
               'i2c_addr':   0x68,
               'frame_id':   'imu_link',
               'publish_hz': 100.0,
             }]),

        # ── Optical flow odometry ─────────────────────────────────────────
        Node(package='mecanum_localization', executable='flow_odometry_node',
             parameters=[{
               'sensor_height_m': 0.05,
               'loop_rate_hz':    100.0,
             }]),

        # ── Camera ───────────────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='camera_node',
             parameters=[{
               'device_id': 1,
               'width':     640,
               'height':    480,
               'fps':       30,
             }]),

        # ── Duck operations ───────────────────────────────────────────────
        Node(package='duck_operations', executable='duck_detector_node'),
        Node(package='duck_operations', executable='plow_controller_node'),

        # ── EKF localisation ──────────────────────────────────────────────
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=[ekf_config]),

        # ── Nav2 ─────────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('nav2_bringup').find('nav2_bringup'),
                    'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': arena_map,
                'params_file': nav2_params,
                'use_sim_time': 'false',
            }.items()
        ),

        # ── Task manager ──────────────────────────────────────────────────
        Node(
            package='task_manager',
            executable='task_manager_node',
            parameters=[{
                'arena_positions_yaml': os.path.join(
                    FindPackageShare('task_manager').find('task_manager'),
                    'config', 'arena_positions.yaml')
            }]
        ),

        # ── Position verifier ─────────────────────────────────────────────
        Node(package='position_verifier', executable='position_verifier_node'),

        # ── Initial pose publisher (after 3s) ─────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mecanum_bringup',
                    executable='set_initial_pose',
                    parameters=[{
                        'x':   START_X,
                        'y':   START_Y,
                        'yaw': START_YAW,
                    }]
                )
            ]
        ),
    ])
