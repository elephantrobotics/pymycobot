# coding=utf-8

from __future__ import absolute_import
import datetime

from pymycobot.generate import MyCobotCommandGenerator
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.mycobot import MyCobot
from pymycobot.mybuddy import MyBuddy
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.mycobotsocket import MyCobotSocket
from pymycobot.genre import Angle, Coord
from pymycobot import utils
from pymycobot.mybuddysocket import MyBuddySocket
from pymycobot.mira import Mira
from pymycobot.mybuddybluetooth import MyBuddyBlueTooth
from pymycobot.mybuddyemoticon import MyBuddyEmoticon
from pymycobot.mypalletizersocket import MyPalletizerSocket

__all__ = [
    "MyCobot",
    "MyCobotCommandGenerator",
    "Angle",
    "Coord",
    "utils",
    "MyPalletizer",
    "MyCobotSocket",
    "MyBuddyCommandGenerator",
    "MyBuddy",
    "MyBuddySocket",
    "MyBuddyBlueTooth",
    "Mira",
    "MyBuddyEmoticon",
    "MyPalletizerSocket"
]

__version__ = "2.9.7b3"
__author__ = "Elephantrobotics"
__email__ = "weiquan.xu@elephantrobotics.com"
__git_url__ = "https://github.com/elephantrobotics/pymycobot"
__copyright__ = "CopyRight (c) 2020-{0} Shenzhen Elephantrobotics technology".format(
    datetime.datetime.now().year
)

# For raspberry mycobot 280.
PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000
