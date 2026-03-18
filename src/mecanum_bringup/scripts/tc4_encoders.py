#!/usr/bin/env python3
"""
TC4 -- Wheel Encoder / Odometry Test (ENC-01)
Tests: All 4 encoders respond to motion, /odom updates correctly when
       robot is driven forward, backward, and rotated.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc4_encoders.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time, math

TIMEOUT = 15.0

class TC4(Node):
    def __init__(self):
        super().__init__('tc4_encoders')
        self.odom = None; self.odom_count = 0
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_odom(self, msg): self.odom = msg; self.odom_count += 1

    def get_pos(self):
        if self.odom is None: return 0.0, 0.0, 0.0
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        return p.x, p.y, yaw

    def drive(self, vx=0.0, vy=0.0, wz=0.0, duration=2.0):
        t = Twist()
        t.linear.x = vx; t.linear.y = vy; t.angular.z = wz
        deadline = time.time() + duration
        while time.time() < deadline:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())  # stop
        rclpy.spin_once(self, timeout_sec=0.2)

    def stop(self):
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
node = TC4()

print("\n==============================================")
print("  TC4 -- Wheel Encoder / Odometry Test")
print("==============================================\n")
print("  WARNING: Robot will move during this test!")
print("  Make sure there is >1m clear space in front.\n")
input("  Press Enter when ready...")

# ── Wait for odom ─────────────────────────────────────────────────────────
print("\nStep 1: Waiting for /odom to publish...")
ok = wait_for(lambda: node.odom_count >= 5)
PASS(f"/odom publishing ({node.odom_count} msgs)") if ok else FAIL("/odom not publishing")

# ── Drive forward ─────────────────────────────────────────────────────────
print("\nStep 2: Drive forward 0.4m at 0.2 m/s (2 seconds)...")
x0, y0, yaw0 = node.get_pos()
print(f"        Start: x={x0:.3f}  y={y0:.3f}  yaw={yaw0:.3f}")
node.drive(vx=0.2, duration=2.0)
time.sleep(0.5); rclpy.spin_once(node, timeout_sec=0.2)
x1, y1, yaw1 = node.get_pos()
dx = x1 - x0
print(f"        End:   x={x1:.3f}  y={y1:.3f}  dx={dx:.3f}")
PASS(f"Forward motion detected: dx={dx:.3f}m") if dx > 0.1 else FAIL(f"Insufficient forward motion: dx={dx:.3f}m")

# ── Drive backward ────────────────────────────────────────────────────────
print("\nStep 3: Drive backward 0.4m at -0.2 m/s (2 seconds)...")
node.drive(vx=-0.2, duration=2.0)
time.sleep(0.5); rclpy.spin_once(node, timeout_sec=0.2)
x2, y2, yaw2 = node.get_pos()
dx_back = x2 - x1
print(f"        End:   x={x2:.3f}  y={y2:.3f}  dx_back={dx_back:.3f}")
PASS(f"Backward motion detected: dx={dx_back:.3f}m") if dx_back < -0.1 else FAIL(f"Insufficient backward motion: dx={dx_back:.3f}m")

# ── Check return to start ─────────────────────────────────────────────────
print("\nStep 4: Check position returned near start...")
total_drift = abs(x2 - x0)
print(f"        Start x={x0:.3f}  Current x={x2:.3f}  Drift={total_drift:.3f}m")
PASS(f"Position returned within 0.1m (drift={total_drift:.3f}m)") if total_drift < 0.10 else FAIL(f"Large drift: {total_drift:.3f}m (expected <0.10m)")

# ── Rotate 360 degrees ────────────────────────────────────────────────────
print("\nStep 5: Rotate 360 degrees in place (~6.28 rad at 0.5 rad/s, 13s)...")
print("        Make sure there is clearance around robot...")
x3, y3, yaw3 = node.get_pos()
node.drive(wz=0.5, duration=12.6)
time.sleep(0.5); rclpy.spin_once(node, timeout_sec=0.2)
x4, y4, yaw4 = node.get_pos()
yaw_diff = abs(yaw4 - yaw3)
# Normalize to 0-2pi
while yaw_diff > math.pi: yaw_diff = abs(yaw_diff - 2*math.pi)
print(f"        Yaw start={yaw3:.3f}  end={yaw4:.3f}  diff={yaw_diff:.3f} rad")
PASS(f"Rotation detected: {yaw_diff:.3f} rad") if yaw_diff > 0.5 else FAIL(f"Insufficient rotation: {yaw_diff:.3f} rad")

# ── Strafe test (mecanum) ─────────────────────────────────────────────────
print("\nStep 6: Strafe right 0.3m (mecanum wheel test)...")
y5_start = node.get_pos()[1]
node.drive(vy=0.15, duration=2.0)
time.sleep(0.5); rclpy.spin_once(node, timeout_sec=0.2)
y5_end = node.get_pos()[1]
dy = abs(y5_end - y5_start)
print(f"        dy={dy:.3f}m")
PASS(f"Strafe motion detected: dy={dy:.3f}m") if dy > 0.05 else FAIL(f"Insufficient strafe: dy={dy:.3f}m (check mecanum wheels)")

node.stop()
print("\n----------------------------------------------")
print("  TC4 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
