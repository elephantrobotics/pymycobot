"""pymycobot package initialization module."""

# ruff: noqa: F401
import datetime
import sys
from pymycobot.mycobot280 import MyCobot280
from pymycobot.mycobot280rdkx5 import MyCobot280RDKX5, MyCobot280RDKX5Socket
from pymycobot.mypalletizer260 import MyPalletizer260
from pymycobot.mecharm270 import MechArm270
from pymycobot.mycobot280socket import MyCobot280Socket
from pymycobot.mycobot320socket import MyCobot320Socket
from pymycobot.mycobot320 import MyCobot320
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
from pymycobot.myagvpro import MyAGVPro
from pymycobot.myagvpro_socket import MyAGVProSocket
from pymycobot.myagvpro_bluetooth import MyAGVProBluetooth
from pymycobot.myarmsocket import MyArmSocket
from pymycobot.mecharmsocket import MechArmSocket

# from pymycobot.mycobotpro630 import Phoenix
from pymycobot.mercurysocket import MercurySocket
from pymycobot.myarmm import MyArmM
from pymycobot.myarmc import MyArmC
from pymycobot.pro630 import Pro630
from pymycobot.pro630client import Pro630Client
from pymycobot.pro400 import Pro400
from pymycobot.pro400client import Pro400Client
from pymycobot.pro450client import Pro450Client
from pymycobot.myarmm_control import MyArmMControl
from pymycobot.mercurychassis_api import ChassisControl
from pymycobot.conveyor_api import ConveyorAPI
from pymycobot.ultraArmP340 import ultraArmP340
from pymycobot.ultraArmP1 import UltraArmP1
from pymycobot.exoskeleton import Exoskeleton
from pymycobot.exoskeletonsocket import ExoskeletonSocket
from pymycobot.mybuddyemoticon import MyBuddyEmoticon


__all__ = [
    "MyPalletizer260",
    "MechArm270",
    "MyCobot280",
    "MyCobot320",
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
    "MyAGVPro",
    "MyAGVProSocket",
    "MyAGVProBluetooth",
    "MechArmSocket",
    "MyArmSocket",
    "MercurySocket",
    "MyArmM",
    "MyArmC",
    # "Phoenix",
    "Pro630",
    "Pro630Client",
    "Pro400",
    "Pro400Client",
    "MyCobot280Socket",
    "MyCobot320Socket",
    "MyArmMControl",
    "ChassisControl",
    "ConveyorAPI",
    "MyCobot280RDKX5",
    "MyCobot280RDKX5Socket",
    "ultraArmP340",
    "Exoskeleton",
    "ExoskeletonSocket",
    "MyBuddyEmoticon",
    "Pro450Client",
    "UltraArmP1",
]

__version__ = "4.0.2"
__author__ = "Elephantrobotics"
__email__ = "weiquan.xu@elephantrobotics.com"
__git_url__ = "https://github.com/elephantrobotics/pymycobot"
__copyright__ = f"CopyRight (c) 2020-{datetime.datetime.now().year} Shenzhen Elephantrobotics technology"

# For raspberry mycobot 280.
PI_PORT = "/dev/ttyAMA0"
PI_BAUD = 1000000
