#!/usr/bin/env python3
"""
TC9 -- IR Transmission Test (IR-01)
Tests: TRANSMIT_IR sends all 4 recorded antenna color codes via NEC IR.
       Plank B moves to 120 degrees for correct transmit angle.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc9_ir_transmit.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

TIMEOUT = 10.0

class TC9(Node):
    def __init__(self):
        super().__init__('tc9_ir_transmit')
        self.task_status = None
        self.antenna_colors = {}
        self.create_subscription(String, '/task_status',    self.cb_status, 10)
        self.create_subscription(String, '/antenna_colors', self.cb_colors, 10)
        self.teensy_pub = self.create_publisher(String, '/teensy_command', 10)
        self.task_pub   = self.create_publisher(String, '/task_command',   10)

    def cb_status(self, msg): self.task_status = msg.data
    def cb_colors(self, msg):
        # Parse COLOR:n:xxx
        parts = msg.data.split(':')
        if len(parts) == 3 and parts[0] == 'COLOR':
            self.antenna_colors[int(parts[1])] = parts[2]

    def send_teensy(self, cmd):
        m = String(); m.data = cmd; self.teensy_pub.publish(m)
        time.sleep(0.8); rclpy.spin_once(self, timeout_sec=0.2)

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
node = TC9()

print("\n==============================================")
print("  TC9 -- IR Transmission Test (TRANSMIT_IR)")
print("==============================================\n")

# ── Step 1: Record colors for all 4 antennas ────────────────────────────
print("Step 1: Record LED colors for all 4 antennas...")
print("        Place LED fixtures at each antenna position.\n")

for ant, angle in [(1, "90"), (2, "150"), (4, "90")]:
    print(f"        Antenna {ant}: Place LED, extend plank B to {angle}...")
    input(f"        Press Enter when LED is ready at antenna_{ant}...")
    node.send_teensy(f"EXTEND_PLANK_B_{angle}")
    time.sleep(0.3)
    node.send_task(f"READ_LED {ant}")
    ok = wait_for(lambda: node.task_status is not None, timeout=8.0)
    if ant in node.antenna_colors:
        PASS(f"Antenna {ant} color recorded: {node.antenna_colors[ant]}")
    else:
        FAIL(f"Antenna {ant} color not recorded")
    node.send_teensy("RETRACT_PLANK_B")
    time.sleep(0.3)

print(f"\n        All recorded colors: {node.antenna_colors}")

# ── Step 2: Plank B to 120 for IR transmit ───────────────────────────────
print("\nStep 2: Extend Plank B to 120 degrees (IR transmit angle)...")
node.send_teensy("EXTEND_PLANK_B_120")
PASS("Plank B at 120 degrees")

# ── Step 3: Send TRANSMIT_IR ──────────────────────────────────────────────
print("\nStep 3: Sending TRANSMIT_IR (4 NEC codes with 200ms gaps)...")
print("        If you have an IR receiver, verify 4 codes are received...")
node.send_task("TRANSMIT_IR")
ok = wait_for(lambda: node.task_status is not None, timeout=10.0)
if ok and node.task_status == "DONE":
    PASS("TRANSMIT_IR replied DONE -- 4 codes sent")
elif ok:
    FAIL(f"TRANSMIT_IR replied: {node.task_status} (expected DONE)")
else:
    FAIL("TRANSMIT_IR timed out -- check IR transmitter on Teensy 2 pin 10")

# ── Step 4: Retract plank B ───────────────────────────────────────────────
print("\nStep 4: Retract Plank B after transmission...")
node.send_teensy("RETRACT_PLANK_B")
PASS("Plank B retracted")

# ── Step 5: Repeat TRANSMIT_IR to confirm repeatability ──────────────────
print("\nStep 5: Repeat TRANSMIT_IR to verify repeatability...")
node.send_teensy("EXTEND_PLANK_B_120")
time.sleep(0.3)
node.send_task("TRANSMIT_IR")
ok = wait_for(lambda: node.task_status is not None, timeout=10.0)
PASS("Repeat TRANSMIT_IR succeeded") if (ok and node.task_status == "DONE") else FAIL(f"Repeat failed: {node.task_status}")
node.send_teensy("RETRACT_PLANK_B")

print("\n----------------------------------------------")
print("  TC9 Complete.")
print(f"  Transmitted colors: {node.antenna_colors}")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
