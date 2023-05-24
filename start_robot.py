#!/usr/bin/env python3

import os

os.chdir("/home/robot/ros2_ws/launch")
os.system("ros2 launch launch_basis.py")
print("Robot: All ROS2 Nodes Started Brudi :D")
print("Logs in -> .ros/log/#datum")

