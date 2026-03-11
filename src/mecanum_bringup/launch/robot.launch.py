
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from geometry_msgs.msg import PoseWithCovarianceStamped

def generate_launch_description():

    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', '..', 'mecanum_description',
        'urdf', 'mecanum_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    # ── Starting pose (bottom-left of arena) ──────────────────────────────
    # Matches starting_pose in arena_positions.yaml
    # ⚠️ Update these to match your actual starting position on the field
    START_X   = 0.15
    START_Y   = 0.15
    START_YAW = 1.5708  # facing up (toward field)

    return LaunchDescription([

        # ── Robot description ──────────────────────────────────────────────
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # ── ros2_control ───────────────────────────────────────────────────
        Node(package='controller_manager',
             executable='ros2_control_node',
             parameters=[
               {'robot_description': robot_description},
               os.path.join(os.path.dirname(__file__), '..', 'config', 'controllers.yaml')
             ]),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager']),

        # ── Teensy bridge ──────────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='teensy_bridge_node',
             parameters=[{'serial_port': '/dev/teensy'}]),

        # ── IMU ────────────────────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='imu_node',
             parameters=[{
               'i2c_bus':    '/dev/i2c-1',
               'i2c_addr':   0x68,
               'frame_id':   'imu_link',
               'publish_hz': 100.0,
             }]),

        # ── Optical flow odometry ──────────────────────────────────────────
        Node(package='mecanum_localization', executable='flow_odometry_node',
             parameters=[{
               'sensor_height_m': 0.05,  # ⚠️ measure actual height above floor
               'loop_rate_hz':    100.0,
             }]),

        # ── Camera ─────────────────────────────────────────────────────────
        Node(package='mecanum_sensors', executable='camera_node',
             parameters=[{
               'device_id':    1,
               'width':        640,
               'height':       480,
               'fps':          30,
             }]),

        # ── Duck operations ────────────────────────────────────────────────
        Node(package='duck_operations', executable='duck_detector_node'),
        Node(package='duck_operations', executable='plow_controller_node'),

        # ── Teensy 2 bridge (replaces Arduino) ────────────────────────────
        # Handles: ultrasonics, line sensor, RGB sensor, IR transmitter, crank DC motor
        Node(package='mecanum_sensors', executable='teensy2_bridge_node',
             parameters=[{'serial_port': '/dev/teensy2'}]),

        # ── EKF localisation ───────────────────────────────────────────────
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=[
               os.path.join(os.path.dirname(__file__), '..', 'config', 'ekf.yaml')
             ]),

        # ── Nav2 ───────────────────────────────────────────────────────────
        Node(package='nav2_bringup', executable='bringup_launch.py',
             parameters=[
               os.path.join(os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'),
               {'map': os.path.join(os.path.dirname(__file__), '..', 'map', 'arena_map.yaml')}
             ]),

        # ── Task manager ───────────────────────────────────────────────────
        Node(package='task_manager', executable='task_manager_node'),

        # ── Position verifier ──────────────────────────────────────────────
        Node(package='position_verifier', executable='position_verifier_node'),

        # ── Set initial pose for EKF after 3s (gives EKF time to start) ───
        # Publishes to /initialpose so EKF knows where robot starts on map
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
