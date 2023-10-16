#!/usr/bin/python
# -*- coding:utf-8 -*-
# @File    : all_run_test.py
# @Author  : Wang Weijian
# @Time    :  2023/07/11 15:38:45
# @function: the script is used to do something
# @version : V1

import time
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0')

init_angles = [
    [90, 0, 0, 0, 0, 0, 0],  # zero point
    [-90, 0, 0, -90, 0, -90, 0],  # first init point
    [90, 0, 0, -90, 0, -90, 0],  # second init point
    [0, 80, 0, 0, 0, 0, 0],  # four
    [-150, 80, 0, 0, 0, 0, 0],
    [150, 80, 0, 0, 0, 0, 0], # 6
    [155.12, -79.27, -0.26, 76.46, 5.71, 45.0, -0.08], # 7
    [-29.53, -48.11, 165, -89.2, 0.41, -43.56, 0.35], # 8
    [87.62, 80, 165, -82.96, 165, -73.38, 0.26], # 9
    [-114.16, 80, 165, -82.96, 165, -104.58, 0.26], # 10
    [17.84, -2.1, 165, -100, 165, 64.16, -6.24], # 11
    [-11.07, -11.68, 2.1, -93.42, 3.07, -9.14, -45.35], # 12
    [-19.68, -16.96, -165, -84.28, -161.19, 1.05, -43.76] , # 13
    [-108.89, -78.39, -165, -89.64, 165, 110, -0.7], # 14
    [160.57, -78.92, 165, -84.63, -163.3, 1.05, -43.76], # 15
]

low_speed = 10
medium_speed = 50
high_speed = 100
timet = int(3)


def main():
    """
    体现myarm三种不同速度的多种运动姿态
    :return: None
    """
    mc.send_angles(init_angles[0], medium_speed)
    time.sleep(3+timet)

    mc.send_angles(init_angles[1], low_speed)
    time.sleep(10+timet)

    mc.send_angles(init_angles[2], high_speed)
    time.sleep(2+timet)

    mc.send_angles(init_angles[3], high_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[4], high_speed)
    time.sleep(4.5+timet)
    mc.send_angles(init_angles[5], high_speed)
    time.sleep(6+timet)
    mc.send_angles(init_angles[6], medium_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[7], medium_speed)
    time.sleep(8+timet)
    mc.send_angles(init_angles[8], medium_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[9], high_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[10], high_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[11], high_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[12], high_speed)
    time.sleep(4+timet)
    mc.send_angles(init_angles[13], medium_speed)
    time.sleep(8+timet)
    mc.send_angles(init_angles[14], high_speed)
    time.sleep(5+timet)
    mc.send_angles(init_angles[0], medium_speed)
    time.sleep(7.5+timet)
    


if __name__ == '__main__':
    main()
