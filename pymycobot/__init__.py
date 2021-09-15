# coding=utf-8

from __future__ import absolute_import
import datetime

from pymycobot.generate import MycobotCommandGenerator
from pymycobot.mycobot import MyCobot
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle, Coord
from pymycobot import utils

__all__ = [
    "MyCobot",
    "MycobotCommandGenerator",
    "Angle",
    "Coord",
    "utils",
    "MyPalletizer",
]

__version__ = "2.6.0"
__author__ = "Elephantrobotics"
__email__ = "weiquan.xu@elephantrobotics.com"
__git_url__ = "https://github.com/elephantrobotics/pymycobot"
__copyright__ = "CopyRight (c) 2020-{0} Shenzhen Elephantrobotics technology".format(
    datetime.datetime.now().year
)

# For raspberry mycobot 280.
PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000
