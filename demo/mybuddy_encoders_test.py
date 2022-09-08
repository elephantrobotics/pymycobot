import time
import os
import sys
import serial
import serial.tools.list_ports

from pymycobot import MyBuddy


port: str
mc: MyBuddy
port = 'COM21'
baud = 115200
DEBUG = False
mc = MyBuddy(port, baud, debug=DEBUG)
mc.get_encoders(0)