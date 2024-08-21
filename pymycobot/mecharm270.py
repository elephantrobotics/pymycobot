# coding=utf-8
from pymycobot import MyCobot280


class MechArm270(MyCobot280):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False, thread_lock=True):
        super(MechArm270, self).__init__(port, baudrate, timeout, debug, thread_lock)
