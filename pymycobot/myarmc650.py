# coding=utf-8

from __future__ import division
from pymycobot.myarm_api import MyArmAPI


class MyArmC650(MyArmAPI):

    def __init__(self, port, baudrate="1000000", timeout=0.1, debug=False):
        super(MyArmC650, self).__init__(port, baudrate, timeout, debug)

