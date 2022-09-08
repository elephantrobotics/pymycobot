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
    

# Connect Robot
mc = MyBuddy(port, baud, debug=DEBUG)
time.sleep(1)

# Test IO output
pin_no = 15
mc.set_gpio_init_mode(0)
mc.set_gpio_setup(pin_no,1)

mc.set_gpio_output(pin_no,1)
time.sleep(2)
mc.set_gpio_output(pin_no,0)
time.sleep(2)
mc.set_gpio_clearup(pin_no)

# #Test io_pwm output
# mc.set_gpio_pwm_start(3,0.5,50)
# time.sleep(2)
# mc.set_gpio_pwm_change_dc(100)
# time.sleep(2)
# mc.set_gpio_pwm_change_freq(100)
# time,time.sleep(2)
# mc.set_gpio_pwm_stop()
# time.sleep(1)
