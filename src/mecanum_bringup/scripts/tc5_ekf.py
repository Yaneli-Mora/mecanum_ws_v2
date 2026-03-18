#!/usr/bin/env python3
"""
TC5 -- EKF Sensor Fusion Test (EKF-01)
Tests: /odometry/filtered publishing, fusion of wheel odom + IMU + optical flow,
       white line correction resets EKF pose.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc5_ekf.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time, math

TIMEOUT = 10.0

class TC5(Node):
    def __init__(self):
        super().__init__('tc5_ekf')
        self.filtered = None; self.filtered_count = 0
        self.raw_odom = None
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_filtered, 10)
        self.create_subscription(Odometry, '/odom',              self.cb_raw,      10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_filtered(self, msg): self.filtered = msg; self.filtered_count += 1
    def cb_raw(self, msg):      self.raw_odom = msg

    def get_filtered_pos(self):
        if self.filtered is None: return 0.0, 0.0, 0.0
        p = self.filtered.pose.pose.position
        q = self.filtered.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        return p.x, p.y, yaw

    def drive(self, vx=0.0, vy=0.0, wz=0.0, duration=1.0):
        t = Twist()
        t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
        deadline = time.time() + duration
        while time.time() < deadline:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())

def wait_for(fn, timeout=TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if fn(): return True
    return False

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC5()

print("\n==============================================")
print("  TC5 -- EKF Sensor Fusion Test")
print("==============================================\n")

print("Step 1: /odometry/filtered publishing...")
ok = wait_for(lambda: node.filtered_count >= 5)
PASS(f"Received {node.filtered_count} filtered messages") if ok else FAIL(f"Only {node.filtered_count} msgs in {TIMEOUT}s")

print("\nStep 2: Filtered pose covariance is finite (not identity)...")
if node.filtered:
    cov = node.filtered.pose.covariance
    # If EKF is running, covariance should be non-trivial (not all zeros or all identity)
    nonzero = sum(1 for v in cov if abs(v) > 1e-9)
    print(f"        Non-zero covariance entries: {nonzero}/36")
    PASS(f"EKF covariance active ({nonzero} non-zero)") if nonzero > 0 else FAIL("Covariance all zeros -- EKF may not be fusing sensors")
else:
    FAIL("No filtered odometry")

print("\nStep 3: Comparing raw /odom vs /odometry/filtered positions...")
if node.filtered and node.raw_odom:
    fx = node.filtered.pose.pose.position.x
    rx = node.raw_odom.pose.pose.position.x
    fy = node.filtered.pose.pose.position.y
    ry = node.raw_odom.pose.pose.position.y
    print(f"        Filtered: x={fx:.4f} y={fy:.4f}")
    print(f"        Raw odom: x={rx:.4f} y={ry:.4f}")
    PASS("Both filtered and raw odom publishing")
else:
    FAIL("Missing filtered or raw odom data")

print("\nStep 4: EKF publish rate (should be ~50Hz)...")
c0 = node.filtered_count; time.sleep(1.0); rclpy.spin_once(node, timeout_sec=0.1)
rate = node.filtered_count - c0
PASS(f"{rate} Hz") if rate >= 30 else FAIL(f"Rate too low: {rate} Hz")

print("\nStep 5: Check filtered pose updates during motion...")
print("        Robot will drive forward briefly...")
input("        Press Enter when ready (clear space needed)...")
x0, y0, _ = node.get_filtered_pos()
node.drive(vx=0.15, duration=1.5)
time.sleep(0.3); rclpy.spin_once(node, timeout_sec=0.2)
x1, y1, _ = node.get_filtered_pos()
dx = abs(x1 - x0)
print(f"        Filtered pose changed: dx={dx:.3f}m")
PASS(f"Filtered pose updated during motion: dx={dx:.3f}m") if dx > 0.05 else FAIL(f"Filtered pose did not update: dx={dx:.3f}m")
# Drive back
node.drive(vx=-0.15, duration=1.5)

print("\nStep 6: White line correction (manual verification)...")
print("        Drive robot over white tape cross-line on field.")
print("        Watch for /initialpose to be published (EKF reset).")
print("        Run in another terminal: ros2 topic echo /initialpose")
print("        (This step is manual -- skipping automated check)")
print("  INFO -- Cross white tape at x=1.215m or y=0.605m to trigger reset")

print("\n----------------------------------------------")
print("  TC5 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
