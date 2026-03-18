#!/usr/bin/env python3
"""
TC11 -- Full Mission Sequence Test (MSN-01)
Tests: All 9 mission steps execute in order and complete successfully.
       Monitors /mission_phase and /task_status throughout.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc11_full_mission.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
import time

MISSION_TIMEOUT = 600.0  # 10 minutes max for full mission

class TC11(Node):
    def __init__(self):
        super().__init__('tc11_full_mission')
        self.mission_phase = "UNKNOWN"
        self.task_statuses = []
        self.antenna_colors = {}
        self.odom = None
        self.phase_history = []

        self.create_subscription(String,   '/mission_phase',   self.cb_phase,  10)
        self.create_subscription(String,   '/task_status',     self.cb_status, 10)
        self.create_subscription(String,   '/antenna_colors',  self.cb_colors, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, 10)

        self.start_pub = self.create_publisher(Bool, '/start_signal', 10)

    def cb_phase(self, msg):
        if not self.phase_history or self.phase_history[-1] != msg.data:
            self.phase_history.append(msg.data)
            print(f"  [PHASE] {msg.data}")
        self.mission_phase = msg.data

    def cb_status(self, msg):
        self.task_statuses.append(msg.data)
        print(f"  [STATUS] {msg.data}")

    def cb_colors(self, msg):
        parts = msg.data.split(':')
        if len(parts) == 3:
            self.antenna_colors[int(parts[1])] = parts[2]
            print(f"  [COLOR] Antenna {parts[1]}: {parts[2]}")

    def cb_odom(self, msg): self.odom = msg

    def send_start(self):
        m = Bool(); m.data = True; self.start_pub.publish(m)

def wait_for(fn, timeout=60.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if fn(): return True
    return False

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC11()

print("\n==============================================")
print("  TC11 -- Full Mission Sequence Test")
print("==============================================\n")
print("  Full arena setup required.")
print("  All fixtures in place: buttons, keypad, crank, LEDs, ducks.")
print("  Robot at start position (0.15, 0.15) yaw=1.5708\n")
input("  Press Enter when ready to start mission...")

# ── Confirm topics alive ──────────────────────────────────────────────────
print("\nStep 1: Confirming all topics alive...")
deadline = time.time() + 10.0
while node.odom is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
PASS("/odometry/filtered publishing") if node.odom else FAIL("/odometry/filtered not publishing")

# ── Send start signal ─────────────────────────────────────────────────────
print("\nStep 2: Sending start signal...")
node.send_start()
ok = wait_for(lambda: node.mission_phase == "RUNNING", timeout=10.0)
PASS("Mission phase changed to RUNNING") if ok else FAIL(f"Mission did not start (phase={node.mission_phase})")

# ── Monitor mission progress ──────────────────────────────────────────────
print("\nStep 3: Monitoring mission progress (timeout 10 minutes)...")
print("        Phase changes and task statuses will print automatically...\n")

start_time = time.time()
ok = wait_for(lambda: node.mission_phase == "COMPLETE", timeout=MISSION_TIMEOUT)

elapsed = time.time() - start_time
print(f"\n  Mission elapsed time: {elapsed:.1f}s")

if ok:
    PASS(f"Mission COMPLETE in {elapsed:.1f}s")
else:
    FAIL(f"Mission did not complete in {MISSION_TIMEOUT}s (last phase: {node.mission_phase})")

# ── Check LED colors recorded ─────────────────────────────────────────────
print("\nStep 4: Checking LED colors were recorded...")
print(f"        Colors: {node.antenna_colors}")
for ant in [1, 2, 4]:
    if ant in node.antenna_colors:
        PASS(f"Antenna {ant} color: {node.antenna_colors[ant]}")
    else:
        FAIL(f"Antenna {ant} color never recorded")

# ── Check DONE statuses ───────────────────────────────────────────────────
print("\nStep 5: Checking task statuses...")
done_count = node.task_statuses.count("DONE")
error_count = node.task_statuses.count("ERROR")
print(f"        DONE: {done_count}  ERROR: {error_count}")
PASS(f"{done_count} DONE statuses received") if done_count >= 3 else FAIL(f"Too few DONE statuses: {done_count}")
PASS("No ERROR statuses") if error_count == 0 else FAIL(f"{error_count} ERROR statuses received")

# ── Phase history ─────────────────────────────────────────────────────────
print("\nStep 6: Phase history:")
for p in node.phase_history:
    print(f"        -> {p}")

print("\n----------------------------------------------")
print("  TC11 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
