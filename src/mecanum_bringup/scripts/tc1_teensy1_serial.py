#!/usr/bin/env python3
"""
TC1 — Teensy 1 Serial Communication Test (T1-SER-01)
Tests: /odom, /tof_distances, /flow_raw publishing from Teensy 1 bridge.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc1_teensy1_serial.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int16MultiArray
import sys, time

TIMEOUT = 10.0   # seconds to wait for each topic

class TC1_Teensy1Serial(Node):
    def __init__(self):
        super().__init__('tc1_teensy1_serial')
        self.results = {}

        # ── Subscribers ────────────────────────────────────────────────
        self.odom_count    = 0
        self.tof_data      = None
        self.flow_data     = None
        self.odom_x_start  = None
        self.odom_x_latest = None

        self.create_subscription(Odometry,             '/odom',          self.cb_odom,  10)
        self.create_subscription(Float32MultiArray,    '/tof_distances', self.cb_tof,   10)
        self.create_subscription(Int16MultiArray,      '/flow_raw',      self.cb_flow,  10)

    def cb_odom(self, msg):
        self.odom_count += 1
        x = msg.pose.pose.position.x
        if self.odom_x_start is None:
            self.odom_x_start = x
        self.odom_x_latest = x

    def cb_tof(self, msg):
        self.tof_data = msg.data

    def cb_flow(self, msg):
        self.flow_data = msg.data

def wait_for(condition_fn, timeout=TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition_fn():
            return True
    return False

def PASS(name): print(f"  ✅ PASS — {name}")
def FAIL(name): print(f"  ❌ FAIL — {name}")

rclpy.init()
node = TC1_Teensy1Serial()

print("\n══════════════════════════════════════════════")
print("  TC1 — Teensy 1 Serial Communication Test")
print("══════════════════════════════════════════════\n")

# ── Test 1: /odom publishing ──────────────────────────────────────────────
print("Step 1: Checking /odom publishes at ~50Hz...")
ok = wait_for(lambda: node.odom_count >= 10)
if ok:
    PASS(f"/odom received {node.odom_count} messages")
else:
    FAIL(f"/odom not publishing (received {node.odom_count} messages in {TIMEOUT}s)")

# ── Test 2: /tof_distances publishing ────────────────────────────────────
print("\nStep 2: Checking /tof_distances publishes 3 values...")
ok = wait_for(lambda: node.tof_data is not None and len(node.tof_data) == 3)
if ok:
    d = node.tof_data
    PASS(f"/tof_distances: rear-L={d[0]:.3f}m  rear-R={d[1]:.3f}m  left-side={d[2]:.3f}m")
else:
    FAIL("/tof_distances not publishing or wrong size")

# ── Test 3: ToF values are in valid range ────────────────────────────────
print("\nStep 3: Checking ToF values are in valid range (0.01–4.0m)...")
if node.tof_data is not None:
    valid = all(0.01 <= v <= 4.0 for v in node.tof_data)
    if valid:
        PASS("All ToF readings in valid range")
    else:
        FAIL(f"ToF out of range: {node.tof_data}")
else:
    FAIL("No ToF data received")

# ── Test 4: /flow_raw publishing ──────────────────────────────────────────
print("\nStep 4: Checking /flow_raw publishes 2 values (vx, vy)...")
ok = wait_for(lambda: node.flow_data is not None and len(node.flow_data) == 2)
if ok:
    PASS(f"/flow_raw: vx={node.flow_data[0]}  vy={node.flow_data[1]}")
else:
    FAIL("/flow_raw not publishing or wrong size")

# ── Test 5: /dev/teensy symlink ───────────────────────────────────────────
print("\nStep 5: Checking /dev/teensy symlink exists...")
import os
if os.path.exists('/dev/teensy'):
    PASS("/dev/teensy symlink exists")
else:
    FAIL("/dev/teensy symlink missing — check udev rules")

print("\n──────────────────────────────────────────────")
print("  TC1 Complete. Fix any FAILs before proceeding.")
print("──────────────────────────────────────────────\n")

node.destroy_node()
rclpy.shutdown()
