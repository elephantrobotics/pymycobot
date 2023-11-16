# -*- coding:utf-8 -*-
"""
File    : dexterous_hands.py
Time    : 2023/11/16
Function: This script is used to control the switch of the dexterous hand of the robot arm end effector.
Atom Version: V1.1
pymycobot Version: 3.3.0
"""
import time
from pymycobot.myarm import MyArm

# Initialize the MyArm object with the appropriate port
ma = MyArm('/dev/ttyAMA0')

# Perform dexterity hand switch action 3 times
for i in range(3):
    # Open the gripper (0 represents open dexterous hand gripper, 80 represents speed, and 2 represents dexterous hand type.)
    ma.set_gripper_state(0, 80, 2)
    # Wait for 2 seconds
    time.sleep(2)

    # Close the gripper (1 represents close dexterous hand gripper, 80 represents speed, and 2 represents dexterous hand type.)
    ma.set_gripper_state(1, 80, 2)
    # Wait for 2 seconds
    time.sleep(2)
