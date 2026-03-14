#!/bin/bash
# install_autostart.sh — installs mecanum_robot systemd service
# Run with: sudo bash install_autostart.sh

set -e

SERVICE_FILE="/etc/systemd/system/mecanum_robot.service"

echo "Installing mecanum_robot systemd service..."

cat > $SERVICE_FILE << 'EOF'
[Unit]
Description=Mecanum Robot ROS2 Launch
After=network.target
# Wait for USB serial devices (Teensy 1 and 2) to be ready
After=dev-teensy.device dev-teensy2.device
Wants=dev-teensy.device dev-teensy2.device

[Service]
Type=simple
User=tigertronics
WorkingDirectory=/home/tigertronics

# Source ROS2 and workspace, then launch
ExecStart=/bin/bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source /home/tigertronics/mecanum_ws_v2/install/setup.bash && \
  ros2 launch mecanum_bringup robot.launch.py'

# Restart on crash — important for competition
Restart=on-failure
RestartSec=5

# Give hardware time to settle before restarting
TimeoutStartSec=60

# Log output to journald (view with: journalctl -u mecanum_robot -f)
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable mecanum_robot
systemctl start mecanum_robot

echo ""
echo "Done! Service installed and started."
echo ""
echo "Useful commands:"
echo "  sudo systemctl status mecanum_robot     — check status"
echo "  journalctl -u mecanum_robot -f          — watch live logs"
echo "  sudo systemctl stop mecanum_robot       — stop robot"
echo "  sudo systemctl disable mecanum_robot    — disable autostart"
echo "  sudo systemctl restart mecanum_robot    — restart after code change"
