#!/bin/bash
# install_deps.sh — run once on Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy)
set -e

echo "=== Installing ROS 2 Jazzy dependencies ==="
sudo apt update

sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-mecanum-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-usb-cam \
  ros-jazzy-camera-calibration \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-pluginlib \
  ros-jazzy-rclcpp-action \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-xacro \
  python3-colcon-common-extensions \
  python3-rosdep

echo "=== Installing udev rules ==="
sudo cp udev_rules/99-mecanum-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "=== Running rosdep ==="
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building workspace ==="
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== Done! ==="
echo "Run: source install/setup.bash"
echo "Run: ros2 launch mecanum_bringup full_mission.launch.py"
