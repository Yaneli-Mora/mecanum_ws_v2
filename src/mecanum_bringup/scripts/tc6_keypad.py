#!/usr/bin/env python3
"""
TC6 -- Keypad Solenoid Test (KPD-01)
Tests: PRESS_KEYPAD fires solenoids in correct order 7-3-7-3-8-#
       with DONE reply. Tests approach positioning via rear ToF sensors.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc6_keypad.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

TIMEOUT = 10.0
KEYPAD_TARGET_M = 0.12   # expected rear ToF distance at keypad

class TC6(Node):
    def __init__(self):
        super().__init__('tc6_keypad')
        self.task_status = None
        self.tof_data = None
        self.create_subscription(String,            '/task_status',      self.cb_status, 10)
        self.create_subscription(Float32MultiArray, '/tof_distances',    self.cb_tof,    10)
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
node = TC6()

print("\n==============================================")
print("  TC6 -- Keypad Solenoid Test (PRESS_KEYPAD)")
print("==============================================\n")
print("  Sequence to fire: 7 -> 3 -> 7 -> 3 -> 8 -> #\n")

# ── Test 1: ToF sensors reading ───────────────────────────────────────────
print("Step 1: Checking rear ToF sensors (0=rear-L, 1=rear-R)...")
ok = wait_for(lambda: node.tof_data is not None and len(node.tof_data) >= 2)
if ok:
    l, r = node.tof_data[0], node.tof_data[1]
    PASS(f"Rear ToF reading: L={l:.3f}m  R={r:.3f}m")
else:
    FAIL("/tof_distances not publishing")

# ── Test 2: PRESS_KEYPAD basic command ───────────────────────────────────
print("\nStep 2: Sending PRESS_KEYPAD command (listen for solenoids firing)...")
print("        Watch/listen for solenoid sequence: 7-3-7-3-8-# ...")
node.send_cmd("PRESS_KEYPAD")
ok = wait_for(lambda: node.task_status is not None, timeout=10.0)
if ok and node.task_status == "DONE":
    PASS("PRESS_KEYPAD replied DONE")
elif ok:
    FAIL(f"PRESS_KEYPAD replied: {node.task_status} (expected DONE)")
else:
    FAIL("PRESS_KEYPAD timed out -- check Teensy 2 connection")

# ── Test 3: Approach position check ──────────────────────────────────────
print(f"\nStep 3: Manual check -- place robot rear ~{KEYPAD_TARGET_M}m from keypad face...")
input("        Position robot, then press Enter...")
rclpy.spin_once(node, timeout_sec=0.3)
if node.tof_data is not None:
    l, r = node.tof_data[0], node.tof_data[1]
    avg = (l + r) / 2.0
    diff = abs(l - r)
    print(f"        ToF: L={l:.3f}m  R={r:.3f}m  avg={avg:.3f}m  diff={diff:.3f}m")
    if abs(avg - KEYPAD_TARGET_M) < 0.03:
        PASS(f"Distance correct: avg={avg:.3f}m (target {KEYPAD_TARGET_M}m)")
    else:
        FAIL(f"Distance off: avg={avg:.3f}m (target {KEYPAD_TARGET_M}m)")
    if diff < 0.015:
        PASS(f"Robot square to keypad: diff={diff:.3f}m")
    else:
        FAIL(f"Robot not square: L-R diff={diff:.3f}m (expected <0.015m)")
else:
    FAIL("No ToF data")

# ── Test 4: Fire again at correct position ────────────────────────────────
print("\nStep 4: Fire PRESS_KEYPAD again at approach position...")
node.send_cmd("PRESS_KEYPAD")
ok = wait_for(lambda: node.task_status is not None, timeout=10.0)
if ok and node.task_status == "DONE":
    PASS("PRESS_KEYPAD at position replied DONE")
else:
    FAIL(f"Unexpected reply: {node.task_status}")

print("\n----------------------------------------------")
print("  TC6 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
