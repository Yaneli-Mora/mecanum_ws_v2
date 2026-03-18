#!/usr/bin/env python3
"""
TC2 — Teensy 2 Serial Communication Test (T2-SER-01)
Tests: /ultrasonic_distances, /line_state, TURN_CRANK, PRESS_KEYPAD commands.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc2_teensy2_serial.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import os, time

TIMEOUT = 10.0

class TC2(Node):
    def __init__(self):
        super().__init__('tc2_teensy2_serial')
        self.us_data = None; self.us_count = 0
        self.line_state = None; self.task_status = None
        self.create_subscription(Float32MultiArray, '/ultrasonic_distances', self.cb_us,     10)
        self.create_subscription(String,            '/line_state',           self.cb_line,   10)
        self.create_subscription(String,            '/task_status',          self.cb_status, 10)
        self.cmd_pub = self.create_publisher(String, '/task_command', 10)

    def cb_us(self, msg):     self.us_data = msg.data; self.us_count += 1
    def cb_line(self, msg):   self.line_state = msg.data
    def cb_status(self, msg): self.task_status = msg.data
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
node = TC2()

print("\n==============================================")
print("  TC2 -- Teensy 2 Serial Communication Test")
print("==============================================\n")

print("Step 1: /dev/teensy2 symlink...")
PASS("/dev/teensy2 exists") if os.path.exists('/dev/teensy2') else FAIL("/dev/teensy2 missing -- check udev rules")

print("\nStep 2: /ultrasonic_distances publishes 4 values...")
ok = wait_for(lambda: node.us_data is not None and len(node.us_data) == 4)
if ok:
    d = node.us_data
    PASS(f"FL={d[0]:.3f}m  FR={d[1]:.3f}m  RL={d[2]:.3f}m  RR={d[3]:.3f}m")
else:
    FAIL("/ultrasonic_distances not publishing")

print("\nStep 3: Ultrasonic values in valid range (0.02-4.0m)...")
if node.us_data is not None:
    PASS("All in range") if all(0.02 <= v <= 4.0 for v in node.us_data) else FAIL(f"Out of range: {list(node.us_data)}")
else:
    FAIL("No data")

print("\nStep 4: Ultrasonic publish rate (~50Hz)...")
c0 = node.us_count; time.sleep(1.0); rclpy.spin_once(node, timeout_sec=0.1)
rate = node.us_count - c0
PASS(f"{rate} Hz") if rate >= 40 else FAIL(f"Rate too low: {rate} Hz")

print("\nStep 5: /line_state publishing...")
ok = wait_for(lambda: node.line_state is not None)
PASS(f"line_state={node.line_state}") if ok else FAIL("/line_state not publishing")

print("\nStep 6: TURN_CRANK -- crank spins 3 turns (timeout 15s)...")
node.send_cmd("TURN_CRANK")
ok = wait_for(lambda: node.task_status is not None, timeout=15.0)
if ok and node.task_status == "DONE": PASS(f"Replied: {node.task_status}")
elif ok: FAIL(f"Replied: {node.task_status} (expected DONE)")
else: FAIL("Timed out -- no response")

print("\nStep 7: PRESS_KEYPAD -- solenoids fire 7-3-7-3-8-# (timeout 10s)...")
node.send_cmd("PRESS_KEYPAD")
ok = wait_for(lambda: node.task_status is not None, timeout=10.0)
if ok and node.task_status == "DONE": PASS(f"Replied: {node.task_status}")
elif ok: FAIL(f"Replied: {node.task_status} (expected DONE)")
else: FAIL("Timed out -- no response")

print("\n----------------------------------------------")
print("  TC2 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
