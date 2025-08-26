# -*- coding:utf-8 -*-
"""
File    : space_arm_angle.py
Time    : 2023/11/16
Function: The function of this script is to make the robot arm move with zero space deflection angle.
Atom Version: V1.1
pymycobot Version: 3.3.0
"""

import time
from pymycobot.myarm import MyArm

# Create a MyArm object and specify the serial port '/dev/ttyAMA0'
mc = MyArm('/dev/ttyAMA0')

# Set interpolation mode
mc.set_fresh_mode(0)
# Wait for 0.5 seconds
time.sleep(0.5)

# Send the joint angle command to move the robotic arm to the initial position with a speed of 60
mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
# Wait for 2.5 seconds
time.sleep(2.5)

# Send the joint angle command to move the robotic arm to the specified position with a speed of 60
mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, -76.2, 0.0], 60)
# Wait for 2.5 seconds
time.sleep(2.5)

# Execute a series of actions in a loop, 3 times in total
for i in range(3):
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)

# Send the joint angle command to move the robotic arm to the specified position with a speed of 60
mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)
time.sleep(2.5)

# Execute a series of actions in a loop, 3 times in total
for i in range(3):
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)

# Send the joint angle command to move the robotic arm to the specified position with a speed of 60
mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)

# Send the joint angle command to move the robotic arm to the specified position with a speed of 60
mc.send_angles([40.25, 36.38, -0.52, -67.23, 0.26, -76.37, -0.35], 60)
time.sleep(2.5)

# Execute a series of actions in a loop, 3 times in total
for i in range(3):
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    # Set the zero space deflection angle value to move to the target position with a speed of 20, 70 is the angle of joint 1
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)

# Send the joint angle command to move the robotic arm to the specified position with a speed of 60
mc.send_angles([40.25, 36.38, -0.52, -67.23, 0.26, -76.37, -0.35], 60)
time.sleep(2.5)

# # Send the joint angle command to move the robotic arm to the initial position with a speed of 60
mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
time.sleep(2.5)
