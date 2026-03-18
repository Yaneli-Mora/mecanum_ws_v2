#!/usr/bin/env python3
"""
TC7 -- Crank Module Test (CRK-01)
Tests: TURN_CRANK spins 3 full turns and replies DONE.
       Left-side ToF approach distance for antenna_2.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc7_crank.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

TIMEOUT = 15.0
CRANK_TARGET_M = 0.10   # expected left-side ToF at crank

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
print("  TC7 -- Crank Module Test (TURN_CRANK)")
print("==============================================\n")

print("Step 1: Checking left-side ToF sensor (index 2)...")
ok = wait_for(lambda: node.tof_data is not None and len(node.tof_data) >= 3)
if ok:
    left = node.tof_data[2]
    PASS(f"Left-side ToF: {left:.3f}m")
else:
    FAIL("/tof_distances not publishing or less than 3 sensors")

print("\nStep 2: Send TURN_CRANK -- crank should spin exactly 3 turns (timeout 15s)...")
print("        Watch crank module for 3 full rotations...")
node.send_cmd("TURN_CRANK")
ok = wait_for(lambda: node.task_status is not None, timeout=15.0)
if ok and node.task_status == "DONE":
    PASS("TURN_CRANK replied DONE -- 3 turns completed")
elif ok:
    FAIL(f"TURN_CRANK replied: {node.task_status} (expected DONE)")
else:
    FAIL("TURN_CRANK timed out -- check motor wiring and CPR value")

print("\nStep 3: Approach position -- place left side of robot ~0.10m from crank face...")
input("        Position robot, then press Enter...")
rclpy.spin_once(node, timeout_sec=0.3)
if node.tof_data is not None and len(node.tof_data) >= 3:
    left = node.tof_data[2]
    print(f"        Left-side ToF: {left:.3f}m (target {CRANK_TARGET_M}m)")
    if abs(left - CRANK_TARGET_M) < 0.03:
        PASS(f"Approach distance correct: {left:.3f}m")
    else:
        FAIL(f"Approach distance off: {left:.3f}m (target {CRANK_TARGET_M}m, tolerance ±0.03m)")
else:
    FAIL("No left-side ToF data (tof_distances[2])")

print("\nStep 4: Turn crank again at approach position...")
node.send_cmd("TURN_CRANK")
ok = wait_for(lambda: node.task_status is not None, timeout=15.0)
if ok and node.task_status == "DONE":
    PASS("TURN_CRANK at position replied DONE")
else:
    FAIL(f"Unexpected reply: {node.task_status}")

print("\n----------------------------------------------")
print("  TC7 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
