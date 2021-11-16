#!/usr/bin/env python3
# coding:utf-8

from pymycobot import MyCobotSocket

m = MyCobotSocket("192.168.10.10", "/dev/ttyAMA0", "1000000")
print(m.get_coords())
