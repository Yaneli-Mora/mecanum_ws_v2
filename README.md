# mecanum_ws — ROS 2 Jazzy Competition Robot

## Hardware
- Raspberry Pi 5 (Ubuntu 24.04)
- Teensy 4.1  → /dev/teensy           (drive + all sensors)
- Arduino Uno → /dev/arduino_attachment (crank + keypad only)
- Forward RGB USB webcam → /dev/video0
- IMU (I2C)

## First-time setup
```bash
./install_deps.sh
source install/setup.bash
```

## Run competition mission
```bash
ros2 launch mecanum_bringup full_mission.launch.py
```

## Package build order
1. mecanum_description
2. mecanum_hardware
3. mecanum_sensors
4. mecanum_localization
5. position_verifier
6. attachment_interface
7. duck_operations
8. task_manager
9. mecanum_bringup

## Before every competition
1. Update arena_positions.yaml with measured coordinates
2. Update white_lines.yaml with measured line positions
3. Update starting_pose in localization.launch.py
4. Calibrate color sensor thresholds in sensor_calibration.yaml
5. Run camera calibration and update calibration/forward_camera.yaml
6. colcon build && ros2 launch mecanum_bringup full_mission.launch.py

## Key topics
| Topic                  | Publisher              | Subscriber           |
|------------------------|------------------------|----------------------|
| /start_signal          | teensy_bridge_node     | task_manager         |
| /odom                  | odometry_publisher     | ekf_filter_node      |
| /odometry/filtered     | ekf_filter_node        | Nav2, task_manager   |
| /cmd_vel               | Nav2 / plow_controller | mecanum_drive_ctrl   |
| /color_sensor/zone     | teensy_bridge_node     | white_line_corrector |
| /tof_distances         | teensy_bridge_node     | position_verifier    |
| /duck_detections       | duck_detector_node     | task_manager         |
| /plow_command          | task_manager           | plow_controller      |
| /plow_state            | plow_controller        | task_manager         |
| /task_command          | task_manager           | attachment_interface |
| /task_status           | attachment_interface   | task_manager         |
| /position_verified     | position_verifier      | task_manager         |
| /mission_phase         | task_manager           | all nodes            |
