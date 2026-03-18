#!/bin/bash
# run_all_tests.sh -- Run all unit test scripts in order
# Usage: bash run_all_tests.sh
# Run AFTER: ros2 launch mecanum_bringup robot.launch.py

SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
source ~/mecanum_ws_v2/install/setup.bash

echo ""
echo "=============================================="
echo "  Mecanum Robot -- Full Unit Test Suite"
echo "  Run each test? Press Enter to run, Ctrl+C to skip"
echo "=============================================="
echo ""

run_test() {
    local num=$1
    local name=$2
    local file=$3
    echo ""
    echo "----------------------------------------------"
    echo "  Test $num: $name"
    echo "----------------------------------------------"
    read -p "  Run this test? [Enter=yes, s=skip]: " choice
    if [ "$choice" != "s" ]; then
        python3 "$SCRIPTS_DIR/$file"
    else
        echo "  Skipped."
    fi
}

run_test  1 "Teensy 1 Serial Communication"    "tc1_teensy1_serial.py"
run_test  2 "Teensy 2 Serial Communication"    "tc2_teensy2_serial.py"
run_test  3 "IMU Node (MPU-9250)"              "tc3_imu.py"
run_test  4 "Wheel Encoders / Odometry"        "tc4_encoders.py"
run_test  5 "EKF Sensor Fusion"                "tc5_ekf.py"
run_test  6 "Keypad Solenoid Sequence"         "tc6_keypad.py"
run_test  7 "Crank Module"                     "tc7_crank.py"
run_test  8 "Plank B Servo and LED Read"       "tc8_plank_led.py"
run_test  9 "IR Transmission"                  "tc9_ir_transmit.py"
run_test 10 "Nav2 Navigation"                  "tc10_nav2.py"
run_test 11 "Full Mission Sequence"            "tc11_full_mission.py"
run_test 12 "Autostart / Systemd"             "tc12_autostart.py"

echo ""
echo "=============================================="
echo "  All tests complete!"
echo "=============================================="
echo ""
