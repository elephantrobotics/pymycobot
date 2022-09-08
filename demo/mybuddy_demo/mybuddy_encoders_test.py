#import library
import time
import os
import sys
import serial
import serial.tools.list_ports
import platform
sys.path.append(os.getcwd())
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
time.sleep(1)

# Test IO output
mc.set_gpio_mode(1,1)
mc.set_gpio_output(1,1)
time.sleep(2)
mc.set_gpio_output(1,0)
time.sleep(2)
mc.set_gpio_clearup(1)

#Test io_pwm output
mc.set_gpio_pwm_start(3,0.5,50)
time.sleep(2)
mc.set_gpio_pwm_change_dc(100)
time.sleep(2)
mc.set_gpio_pwm_change_freq(100)
time,time.sleep(2)
mc.set_gpio_pwm_stop()
time.sleep(1)

#
iic = mc.set_iic_init(1)



# print all encoders and speeds
print(mc.get_encoders(0))

# set run demo encoders，init robot
mc.set_encoders(0,encoders,speeds)
