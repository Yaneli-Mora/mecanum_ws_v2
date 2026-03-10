"""
full_mission.launch.py
Launches the complete robot system in the correct order:
  0s  — robot.launch.py    (hardware + sensors + controllers)
  3s  — localization.launch.py (odom + EKF + map + TF)
  6s  — navigation.launch.py  (Nav2 + position verifier + plow controller)
  10s — mission.launch.py     (task manager — waits for start signal)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    d = get_package_share_directory('mecanum_bringup')

    def inc(f):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(d, 'launch', f)))

    return LaunchDescription([
        inc('robot.launch.py'),
        TimerAction(period=3.0,  actions=[inc('localization.launch.py')]),
        TimerAction(period=6.0,  actions=[inc('navigation.launch.py')]),
        TimerAction(period=10.0, actions=[inc('mission.launch.py')]),
    ])
