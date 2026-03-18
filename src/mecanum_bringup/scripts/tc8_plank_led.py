#!/usr/bin/env python3
"""
TC8 -- Plank B Servo & LED Read Test (PLK-01)
Tests: Plank B moves to 90, 120, 150 degrees and retracts.
       TCS34725 RGB sensor correctly identifies LED colors.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc8_plank_led.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

TIMEOUT = 10.0

class TC8(Node):
    def __init__(self):
        super().__init__('tc8_plank_led')
        self.task_status = None
        self.antenna_colors = []
        self.create_subscription(String, '/task_status',   self.cb_status, 10)
        self.create_subscription(String, '/antenna_colors',self.cb_colors, 10)
        self.teensy_pub = self.create_publisher(String, '/teensy_command', 10)
        self.task_pub   = self.create_publisher(String, '/task_command',   10)

    def cb_status(self, msg): self.task_status = msg.data
    def cb_colors(self, msg): self.antenna_colors.append(msg.data)

    def send_teensy(self, cmd):
        m = String(); m.data = cmd; self.teensy_pub.publish(m)
        time.sleep(0.8)  # let servo move
        rclpy.spin_once(self, timeout_sec=0.2)

    def send_task(self, cmd):
        self.task_status = None
        m = String(); m.data = cmd; self.task_pub.publish(m)

def wait_for(fn, timeout=TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if fn(): return True
    return False

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC8()

print("\n==============================================")
print("  TC8 -- Plank B Servo & LED Read Test")
print("==============================================\n")

# ── Test 1: Retract plank B ───────────────────────────────────────────────
print("Step 1: Retract Plank B to 0 degrees...")
node.send_teensy("RETRACT_PLANK_B")
PASS("RETRACT_PLANK_B sent -- verify plank is at 0 degrees")

# ── Test 2: Extend to 90 ──────────────────────────────────────────────────
print("\nStep 2: Extend Plank B to 90 degrees (antenna_1/4 height)...")
node.send_teensy("EXTEND_PLANK_B_90")
PASS("EXTEND_PLANK_B_90 sent -- verify plank at 90 degrees")

# ── Test 3: Extend to 120 ─────────────────────────────────────────────────
print("\nStep 3: Extend Plank B to 120 degrees (IR transmit angle)...")
node.send_teensy("EXTEND_PLANK_B_120")
PASS("EXTEND_PLANK_B_120 sent -- verify plank at 120 degrees")

# ── Test 4: Extend to 150 ─────────────────────────────────────────────────
print("\nStep 4: Extend Plank B to 150 degrees (antenna_2 height)...")
node.send_teensy("EXTEND_PLANK_B_150")
PASS("EXTEND_PLANK_B_150 sent -- verify plank at 150 degrees")

# ── Test 5: Retract again ─────────────────────────────────────────────────
print("\nStep 5: Retract Plank B back to 0 degrees...")
node.send_teensy("RETRACT_PLANK_B")
PASS("RETRACT_PLANK_B sent -- verify plank retracted")

# ── Test 6: READ_LED with known colors ───────────────────────────────────
for ant, angle, color_hint in [(1, "90", "RED/GREEN/BLUE/PURPLE"), (2, "150", "RED/GREEN/BLUE/PURPLE"), (4, "90", "RED/GREEN/BLUE/PURPLE")]:
    print(f"\nStep 6.{ant}: READ_LED {ant} -- Place a known color LED at antenna_{ant}...")
    print(f"        Extend Plank B to {angle} degrees first...")
    node.send_teensy(f"EXTEND_PLANK_B_{angle}")
    input(f"        Place LED ({color_hint}) at antenna_{ant} position, then press Enter...")
    prev_count = len(node.antenna_colors)
    node.send_task(f"READ_LED {ant}")
    ok = wait_for(lambda: node.task_status is not None, timeout=8.0)
    # Check for new color message
    new_colors = node.antenna_colors[prev_count:]
    if new_colors:
        PASS(f"READ_LED {ant} result: {new_colors[-1]}")
    else:
        FAIL(f"No color published for antenna {ant}")
    if ok and node.task_status == "DONE":
        PASS(f"READ_LED {ant} replied DONE")
    elif ok:
        FAIL(f"READ_LED {ant} replied: {node.task_status}")
    else:
        FAIL(f"READ_LED {ant} timed out")
    node.send_teensy("RETRACT_PLANK_B")

print("\n----------------------------------------------")
print("  TC8 Complete.")
print(f"  Colors recorded: {node.antenna_colors}")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
