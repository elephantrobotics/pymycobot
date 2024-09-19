# coding=utf-8

from __future__ import absolute_import
import datetime
import sys

from pymycobot.generate import CommandGenerator
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.mycobot import MyCobot
from pymycobot.mybuddy import MyBuddy
from pymycobot.mecharm import MechArm
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.mycobotsocket import MyCobotSocket
from pymycobot.genre import Angle, Coord
from pymycobot import utils
from pymycobot.mybuddysocket import MyBuddySocket
from pymycobot.ultraArm import ultraArm
from pymycobot.mybuddybluetooth import MyBuddyBlueTooth
from pymycobot.mypalletizersocket import MyPalletizerSocket
from pymycobot.myarm import MyArm
from pymycobot.myarmsocket import MyArmSocket
from pymycobot.elephantrobot import ElephantRobot
from pymycobot.mercury import Mercury
from pymycobot.myagv import MyAgv
from pymycobot.mecharmsocket import MechArmSocket
from pymycobot.mercurychassis import MercuryChassis
from pymycobot.mercurysocket import MercurySocket
from pymycobot.mycobotpro630 import Phoenix
from pymycobot.myarmm import MyArmM
from pymycobot.myarmc import MyArmC
from pymycobot.myarmm_control import MyArmMControl
__all__ = [
    "MyCobot",
    "CommandGenerator",
    "Angle",
    "Coord",
    "utils",
    "MyPalletizer",
    "MyCobotSocket",
    "MyBuddyCommandGenerator",
    "MyBuddy",
    "MyBuddySocket",
    "MyBuddyBlueTooth",
    "ultraArm",
    "MyPalletizerSocket",
    "MechArm",
    "MyArm",
    "ElephantRobot",
    "Mercury",
    "MyAgv",
    "MechArmSocket",
    "MyArmSocket",
    "MercuryChassis",
    "MercurySocket",
    "Phoenix",
    "MyArmM",
    "MyArmC",
    "MyArmMControl"
]


if sys.platform == "linux":
    from pymycobot.mybuddyemoticon import MyBuddyEmoticon
    __all__.append("MyBuddyEmoticon")

__version__ = "3.5.2"
__author__ = "Elephantrobotics"
__email__ = "weiquan.xu@elephantrobotics.com"
__git_url__ = "https://github.com/elephantrobotics/pymycobot"
__copyright__ = "CopyRight (c) 2020-{0} Shenzhen Elephantrobotics technology".format(
    datetime.datetime.now().year
)

# For raspberry mycobot 280.
PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000
