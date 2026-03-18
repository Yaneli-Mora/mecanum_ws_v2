#!/usr/bin/env python3
"""
TC3 -- IMU Node Test (IMU-01)
Tests: MPU-9250 detected on I2C, /imu/data publishing with valid orientation
       and acceleration. Gravity vector check on flat surface.

Run AFTER launching: ros2 launch mecanum_bringup robot.launch.py
Usage: python3 tc3_imu.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import subprocess, time, math

TIMEOUT = 10.0

class TC3(Node):
    def __init__(self):
        super().__init__('tc3_imu')
        self.imu_count = 0; self.latest_imu = None
        self.create_subscription(Imu, '/imu/data', self.cb_imu, 10)

    def cb_imu(self, msg): self.imu_count += 1; self.latest_imu = msg

def wait_for(fn, timeout=TIMEOUT):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if fn(): return True
    return False

def PASS(n): print(f"  PASS -- {n}")
def FAIL(n): print(f"  FAIL -- {n}")

rclpy.init()
node = TC3()

print("\n==============================================")
print("  TC3 -- IMU Node Test (MPU-9250)")
print("==============================================\n")

print("Step 1: Checking I2C bus for MPU-9250 at 0x68...")
try:
    result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True, timeout=5)
    if '68' in result.stdout:
        PASS("MPU-9250 detected at 0x68 on /dev/i2c-1")
    else:
        FAIL("0x68 not found -- check wiring: SDA=GPIO2, SCL=GPIO3, VCC=3.3V")
        print("       i2cdetect output:\n" + result.stdout)
except Exception as e:
    FAIL(f"i2cdetect failed: {e}")

print("\nStep 2: /imu/data publishing...")
ok = wait_for(lambda: node.imu_count >= 5)
PASS(f"Received {node.imu_count} IMU messages") if ok else FAIL(f"Only {node.imu_count} messages in {TIMEOUT}s")

print("\nStep 3: Checking gravity vector (robot flat on surface)...")
print("        Place robot flat on surface, then press Enter...")
input()
rclpy.spin_once(node, timeout_sec=0.5)
if node.latest_imu:
    az = node.latest_imu.linear_acceleration.z
    ax = node.latest_imu.linear_acceleration.x
    ay = node.latest_imu.linear_acceleration.y
    print(f"        ax={ax:.3f}  ay={ay:.3f}  az={az:.3f} m/s2")
    if 8.0 <= abs(az) <= 11.0:
        PASS(f"Gravity on Z axis: {az:.3f} m/s2 (expected ~9.81)")
    else:
        FAIL(f"Unexpected gravity vector: az={az:.3f} (expected ~9.81)")
    if abs(ax) < 2.0 and abs(ay) < 2.0:
        PASS("X and Y acceleration near zero on flat surface")
    else:
        FAIL(f"Unexpected tilt: ax={ax:.3f} ay={ay:.3f}")
else:
    FAIL("No IMU data received")

print("\nStep 4: Checking orientation quaternion is normalized...")
if node.latest_imu:
    q = node.latest_imu.orientation
    mag = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    print(f"        Quaternion magnitude: {mag:.6f} (expected 1.0)")
    PASS(f"Quaternion normalized: {mag:.4f}") if 0.99 <= mag <= 1.01 else FAIL(f"Quaternion not normalized: {mag:.4f}")
else:
    FAIL("No IMU data")

print("\nStep 5: IMU publish rate...")
c0 = node.imu_count; time.sleep(1.0); rclpy.spin_once(node, timeout_sec=0.1)
rate = node.imu_count - c0
PASS(f"{rate} Hz") if rate >= 50 else FAIL(f"Rate too low: {rate} Hz (expected ~100Hz)")

print("\n----------------------------------------------")
print("  TC3 Complete.")
print("----------------------------------------------\n")
node.destroy_node(); rclpy.shutdown()
