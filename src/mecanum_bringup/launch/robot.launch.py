import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_desc   = FindPackageShare('mecanum_description')
    pkg_bringup = FindPackageShare('mecanum_bringup')

    # Load URDF via xacro
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', '..', 'mecanum_description',
        'urdf', 'mecanum_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([

        # Robot state publisher
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        # ros2_control controller manager
        Node(package='controller_manager',
             executable='ros2_control_node',
             parameters=[
               {'robot_description': robot_description},
               os.path.join(os.path.dirname(__file__), '..', 'config', 'controllers.yaml')
             ]),

        # Spawn controllers (delayed via spawner)
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager']),

        # Teensy bridge — reads serial packet and publishes all sensor topics
        Node(package='mecanum_sensors', executable='teensy_bridge_node',
             parameters=[{'serial_port': '/dev/teensy'}]),

        # Camera driver — Logitech Carl Zeiss Tessar HD 1080p
        Node(package='usb_cam', executable='usb_cam_node_exe',
             name='camera_forward',
             parameters=[{
               'video_device':    '/dev/video1',
               'image_width':     1920,
               'image_height':    1080,
               'framerate':       30.0,
               'pixel_format':    'mjpeg2rgb',
               'camera_name':     'forward_camera',
               'camera_info_url': 'file:///home/tigertronics/mecanum_ws_v2/src/mecanum_bringup/calibration/forward_camera.yaml',
             }],
             remappings=[('/image_raw', '/camera/forward/image_raw')]),

        # Fixture and duck detection
        Node(package='mecanum_sensors',   executable='fixture_detector_node'),
        Node(package='duck_operations',   executable='duck_detector_node'),

        # Attachment interface (Arduino serial bridge)
        Node(package='attachment_interface', executable='attachment_node',
             parameters=[{'serial_port': '/dev/arduino_attachment'}]),

        # VL53L1X ToF sensors (3x) — connected to Pi I2C, publishes /tof_distances
        Node(package='mecanum_sensors', executable='tof_node',
             parameters=[{
               'i2c_bus':       '/dev/i2c-1',
               'gpio_chip':     'gpiochip4',  # Pi 5 main GPIO chip
               'xshut_gpio_0':  17,   # GPIO17 = Pi Pin 11 — Sensor 0
               'xshut_gpio_1':  27,   # GPIO27 = Pi Pin 13 — Sensor 1
               'xshut_gpio_2':  22,   # GPIO22 = Pi Pin 15 — Sensor 2
               'addr_sensor_0': 0x30,
               'addr_sensor_1': 0x31,
               'addr_sensor_2': 0x32,
               'num_sensors':   1,    # increase to 3 when all sensors wired
               'publish_hz':    50.0,
               'num_sensors':   1,  # increase to 2 or 3 when all wired
             }]),

        # MPU-9250 IMU — publishes /imu/data consumed by EKF
        Node(package='mecanum_sensors', executable='imu_node',
             parameters=[{
               'i2c_bus':    '/dev/i2c-1',
               'i2c_addr':   0x68,
               'frame_id':   'imu_link',
               'publish_hz': 100.0,
             }]),

        # Optical flow odometry — converts CJMCU-3901 pixels to /odometry/flow for EKF
        Node(package='mecanum_localization', executable='flow_odometry_node',
             parameters=[{
               'sensor_height_m': 0.05,   # ← measure actual height above floor in meters
               'loop_rate_hz':    100.0,
             }]),
    ])
