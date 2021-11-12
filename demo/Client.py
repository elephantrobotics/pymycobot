#!/usr/bin/env python3
# coding:utf-8
from pymycobot.mycobotsocket import MyCobotSocket
m = MyCobotSocket("192.168.10.115", "/dev/ttyAMA0", "1000000")
print(m.send_command("get_coords"))
