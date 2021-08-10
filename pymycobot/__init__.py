from __future__ import absolute_import

from pymycobot.mycobot import MyCobot
from pymycobot.generate import MycobotCommandGenerater
from pymycobot.genre import Angle, Coord

__all__ = ["MyCobot", "MycobotCommandGenerater", "Angle", "Coord", "utils"]

__version__ = "2.5.8"
__author__ = "Zachary zhang"
__email__ = "lijun.zhang@elephantrobotics.com"
__git_url__ = "https://github.com/elephantrobotics/pymycobot"

PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000
