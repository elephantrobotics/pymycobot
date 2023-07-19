# coding=utf-8
from pymycobot import MyArm

class CobotX(MyArm):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        super().__init__(port, baudrate, timeout, debug)