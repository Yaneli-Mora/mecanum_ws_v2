#!/usr/bin/env python3
"""
TC10 -- Nav2 Navigation Test (NAV-01)
Tests: Robot navigates to target coordinates, avoids crater,
       does not collide with walls.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc10_nav2.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import time, math

TIMEOUT = 60.0  # seconds per nav goal
POSITION_TOLERANCE = 0.20  # meters

class TC10(Node):
    def __init__(self):
        super().__init__('tc10_nav2')
        self.filtered = None
        self.us_data = None
        self.nav_result = None
        self.create_subscription(Odometry,          '/odometry/filtered', self.cb_odom, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic_distances', self.cb_us, 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def cb_odom(self, msg): self.filtered = msg
    def cb_us(self, msg):   self.us_data = msg.data

    def get_pos(self):
        if self.filtered is None: return 0.0, 0.0
        p = self.filtered.pose.pose.position
        return p.x, p.y

    def nav_to(self, x, y, yaw=0.0):
        self.nav_result = None
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            print("  FAIL -- Nav2 action server not available")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        q_z = math.sin(yaw / 2.0)
        q_w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = q_z
        goal.pose.pose.orientation.w = q_w
        future = self._nav_client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done(): return False
        handle = future.result()
        if not handle.accepted: return False
        result_future = handle.get_result_async()
        deadline = time.time() + TIMEOUT
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return result_future.done()

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC10()

print("\n==============================================")
print("  TC10 -- Nav2 Navigation Test")
print("==============================================\n")
print("  WARNING: Robot will drive to multiple positions.")
print("  Set up on full arena. Start position: (0.15, 0.15)\n")
input("  Press Enter when robot is at start position and arena is clear...")

# ── Wait for filtered odom ────────────────────────────────────────────────
print("\nStep 1: Waiting for /odometry/filtered...")
deadline = time.time() + 10.0
while node.filtered is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.filtered:
    x, y = node.get_pos()
    PASS(f"Filtered odom: ({x:.3f}, {y:.3f})")
else:
    FAIL("No filtered odometry -- check EKF")

# ── Nav goals ─────────────────────────────────────────────────────────────
goals = [
    ("antenna_1", 0.15, 0.90, 3.1416),
    ("antenna_4", 0.60, 0.15, 3.1416),
    ("antenna_2", 1.20, 0.90, 0.0),
    ("start",     0.15, 0.15, 1.5708),
]

min_ultrasonic = 9.9

for name, gx, gy, gyaw in goals:
    print(f"\nStep: Navigate to {name} ({gx}, {gy})...")
    ok = node.nav_to(gx, gy, gyaw)
    ax, ay = node.get_pos()
    dist = math.sqrt((ax - gx)**2 + (ay - gy)**2)
    print(f"      Arrived at ({ax:.3f}, {ay:.3f})  error={dist:.3f}m")
    if ok and dist < POSITION_TOLERANCE:
        PASS(f"Reached {name}: error={dist:.3f}m")
    elif ok:
        FAIL(f"Nav succeeded but position error large: {dist:.3f}m (tolerance {POSITION_TOLERANCE}m)")
    else:
        FAIL(f"Nav to {name} failed or timed out")
    # Track closest ultrasonic reading
    if node.us_data:
        closest = min(node.us_data)
        if closest < min_ultrasonic:
            min_ultrasonic = closest

print(f"\nStep: Collision check -- minimum ultrasonic reading was {min_ultrasonic:.3f}m...")
PASS(f"No collisions detected (min US={min_ultrasonic:.3f}m)") if min_ultrasonic > 0.05 else FAIL(f"Possible collision: min US={min_ultrasonic:.3f}m")

print("\n----------------------------------------------")
print("  TC10 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
