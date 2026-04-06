#!/usr/bin/env python3
"""
TC7 -- Crank Module Test (CRK-01)
Tests: TURN_CRANK runs L298N DC motor for fixed duration and replies DONE.
       Left-side ToF approach distance for antenna_2.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc7_crank.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

TIMEOUT = 15.0
CRANK_TARGET_M = 0.10   # expected left-side ToF at crank approach

class TC7(Node):
    def __init__(self):
        super().__init__('tc7_crank')
        self.task_status = None
        self.tof_data = None
        self.create_subscription(String,            '/task_status',   self.cb_status, 10)
        self.create_subscription(Float32MultiArray, '/tof_distances', self.cb_tof,    10)
        self.cmd_pub = self.create_publisher(String, '/task_command', 10)

    def cb_status(self, msg): self.task_status = msg.data
    def cb_tof(self, msg):    self.tof_data = msg.data

    def send_cmd(self, cmd):
        self.task_status = None
        m = String(); m.data = cmd; self.cmd_pub.publish(m)

def wait_for(fn, timeout=TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if fn(): return True
    return False

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC7()

print("\n==============================================")
print("  TC7 -- Crank Module Test (L298N, TURN_CRANK)")
print("==============================================\n")
print("  Motor runs for fixed duration (CRANK_DURATION_MS in crank_handler.h)")
print("  No encoder -- time-based control only\n")

# ── Test 1: Left-side ToF sensor ─────────────────────────────────────────
print("Step 1: Checking left-side ToF sensor (index 2)...")
ok = wait_for(lambda: node.tof_data is not None and len(node.tof_data) >= 3)
if ok:
    left = node.tof_data[2]
    PASS(f"Left-side ToF: {left:.3f}m")
else:
    FAIL("/tof_distances not publishing or less than 3 sensors")

# ── Test 2: TURN_CRANK command ────────────────────────────────────────────
print("\nStep 2: Send TURN_CRANK -- motor runs for CRANK_DURATION_MS then stops...")
print("        Watch crank module -- motor should run then stop automatically...")
node.send_cmd("TURN_CRANK")
ok = wait_for(lambda: node.task_status is not None, timeout=15.0)
if ok and node.task_status == "DONE":
    PASS("TURN_CRANK replied DONE -- motor completed run")
elif ok:
    FAIL(f"TURN_CRANK replied: {node.task_status} (expected DONE)")
else:
    FAIL("TURN_CRANK timed out -- check L298N wiring: IN1=20 IN2=21 ENA=22")

# ── Test 3: Approach position ─────────────────────────────────────────────
print(f"\nStep 3: Approach position -- place left side ~{CRANK_TARGET_M}m from crank face...")
input("        Position robot, then press Enter...")
rclpy.spin_once(node, timeout_sec=0.3)
if node.tof_data is not None and len(node.tof_data) >= 3:
    left = node.tof_data[2]
    print(f"        Left-side ToF: {left:.3f}m (target {CRANK_TARGET_M}m)")
    if abs(left - CRANK_TARGET_M) < 0.03:
        PASS(f"Approach distance correct: {left:.3f}m")
    else:
        FAIL(f"Approach distance off: {left:.3f}m (target {CRANK_TARGET_M}m, tolerance +/-0.03m)")
else:
    FAIL("No left-side ToF data")

# ── Test 4: Turn crank at approach position ───────────────────────────────
print("\nStep 4: Run TURN_CRANK at approach position...")
node.send_cmd("TURN_CRANK")
ok = wait_for(lambda: node.task_status is not None, timeout=15.0)
if ok and node.task_status == "DONE":
    PASS("TURN_CRANK at position replied DONE")
else:
    FAIL(f"Unexpected reply: {node.task_status}")

# ── Test 5: Duration tuning hint ──────────────────────────────────────────
print("\nStep 5: Duration tuning check...")
print("        Did the crank complete the full task? (y/n)")
ans = input("        ").strip().lower()
if ans == 'y':
    PASS("CRANK_DURATION_MS is correctly tuned")
else:
    print("  INFO -- Adjust CRANK_DURATION_MS in crank_handler.h:")
    print("          Increase if crank doesn't finish, decrease if it overshoots")
    FAIL("CRANK_DURATION_MS needs tuning")

print("\n----------------------------------------------")
print("  TC7 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
