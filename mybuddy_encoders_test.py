import time
import os
import sys
import serial
import serial.tools.list_ports

from pymycobot.mybuddy import MyBuddy


port: str
mc: MyBuddy
port = 'COM21'
baud = 115200
DEBUG = False

encoders = [2047,2047,2047,2047,2047,2047,2047,2047,2047,2047,2047,2047,2047]
speeds = [500,500,500,500,500,500,500,500,500,500,500,500,500]

mc = MyBuddy(port, baud, debug=DEBUG)
# print(mc.get_angles(1))
print(mc.get_encoders(0))

# mc.set_encoders(0,encoders,speeds)

# mc.set_encoders(1,[1,1,1,1,1,1],[0,2,0,2,0,2])