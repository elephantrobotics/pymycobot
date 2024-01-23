# coding=utf-8
from pymycobot import MyCobot

class MechArm(MyCobot):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False, thread_lock=True):
        super(MechArm, self).__init__(port, baudrate, timeout, debug, thread_lock)