#!/usr/bin/env python3
"""
TC12 -- Autostart / Systemd Test (SYS-01)
Tests: mecanum_robot.service is enabled, running, and all topics
       are publishing without manual launch.

Run WITHOUT ros2 launch (service should have started it automatically).
Usage: python3 tc12_autostart.py
"""

import subprocess, time, sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

EXPECTED_TOPICS = [
    '/odom',
    '/odometry/filtered',
    '/imu/data',
    '/ultrasonic_distances',
    '/tof_distances',
    '/flow_raw',
    '/mission_phase',
    '/task_status',
    '/line_state',
    '/camera/forward/image_raw',
]

TIMEOUT = 15.0

class TC12(Node):
    def __init__(self):
        super().__init__('tc12_autostart')
        self.received = set()
        self.create_subscription(Odometry, '/odom', lambda m: self.received.add('/odom'), 10)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.received.add('/odometry/filtered'), 10)
        self.create_subscription(String, '/mission_phase', lambda m: self.received.add('/mission_phase'), 10)
        self.create_subscription(String, '/line_state', lambda m: self.received.add('/line_state'), 10)

def run_cmd(cmd):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return r.stdout.strip(), r.returncode
    except Exception as e:
        return str(e), 1

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")
def INFO(n): print(f"  INFO -- {n}")

print("\n==============================================")
print("  TC12 -- Autostart / Systemd Test")
print("==============================================\n")
print("  Do NOT manually launch robot.launch.py for this test.")
print("  The service should have started everything on boot.\n")

# ── Step 1: Service enabled ────────────────────────────────────────────────
print("Step 1: mecanum_robot service enabled...")
out, rc = run_cmd("systemctl is-enabled mecanum_robot")
print(f"        systemctl is-enabled: {out}")
PASS("Service is enabled") if out == "enabled" else FAIL(f"Service not enabled: {out}")

# ── Step 2: Service running ────────────────────────────────────────────────
print("\nStep 2: mecanum_robot service active/running...")
out, rc = run_cmd("systemctl is-active mecanum_robot")
print(f"        systemctl is-active: {out}")
PASS("Service is active (running)") if out == "active" else FAIL(f"Service not running: {out}")

# ── Step 3: No recent failures ─────────────────────────────────────────────
print("\nStep 3: Checking for recent service errors...")
out, rc = run_cmd("journalctl -u mecanum_robot --since '5 minutes ago' -p err --no-pager")
if out.strip() == "" or "No entries" in out:
    PASS("No errors in last 5 minutes")
else:
    FAIL(f"Errors found in journal:\n{out[:500]}")

# ── Step 4: ROS 2 topics alive ─────────────────────────────────────────────
print("\nStep 4: Checking ROS 2 topics are active...")
out, rc = run_cmd("ros2 topic list")
active_topics = out.split('\n') if out else []
print(f"        Found {len(active_topics)} active topics")
for t in EXPECTED_TOPICS:
    if t in active_topics:
        PASS(f"Topic active: {t}")
    else:
        FAIL(f"Topic missing: {t}")

# ── Step 5: Topics actually publishing ────────────────────────────────────
print("\nStep 5: Verifying key topics are publishing messages...")
rclpy.init()
node = TC12()
deadline = time.time() + TIMEOUT
while time.time() < deadline and len(node.received) < 3:
    rclpy.spin_once(node, timeout_sec=0.2)

for t in ['/odom', '/odometry/filtered', '/mission_phase', '/line_state']:
    PASS(f"{t} publishing") if t in node.received else FAIL(f"{t} not publishing")

# ── Step 6: Udev symlinks ─────────────────────────────────────────────────
print("\nStep 6: Checking Teensy USB symlinks...")
import os
PASS("/dev/teensy exists")  if os.path.exists('/dev/teensy')  else FAIL("/dev/teensy missing")
PASS("/dev/teensy2 exists") if os.path.exists('/dev/teensy2') else FAIL("/dev/teensy2 missing")

print("\n----------------------------------------------")
print("  TC12 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
