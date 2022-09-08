#import library
import time
import os
import sys
import serial
import serial.tools.list_ports
import platform
from pymycobot.mybuddy import MyBuddy

#define type
port: str
mc: MyBuddy
DEBUG = False

# auto select system port
if platform.system() == 'Windows':
    port = 'COM21'
    baud = 115200
elif platform.system() == 'Linux':
    port = '/dev/ttyACM0'
    baud = 115200
    
# demo encoders and speeds
encoders = [2047,2047,2047,2047,2047,2047,2047, 2047,2047,2047,2047,2047,2047,2047, 2047]        # Lram + gipper  &  Rram + gipper & Wram  encoders
speeds = [500,500,500,500,500,500,500, 500,500,500,500,500,500,500, 500]                         # Lram + gipper  &  Rram + gipper & Wram  speeds

# Connect Robot
mc = MyBuddy(port, baud, debug=DEBUG)

mc.base_io_to_gpio(5)
iic = mc.set_iic_init(1)
iic.open()
iic.close()

mc.set_gpio_clearup()
# print all encoders and speeds
print(mc.get_encoders(0))

# set run demo encoders，init robot
mc.set_encoders(0,encoders,speeds)
