#!/usr/bin/python3

# This code is suitable for the use of the myCobot 
# series product pump provided by Elephant Robotics;
# Please check the instructions for use of the 
# suction pump before use, please refer to here:
# https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.7-accessories/2.7.4-pump.html 
# Running this code requires installing the 'pymycobot' 
# driver library and python driver environment. 
# If you don't installed, please refer to here :
# https://docs.elephantrobotics.com/docs/gitbook-en/7-ApplicationBasePython/

# import library
from pymycobot import MyCobot280               #import mycobot library,if don't have, first 'pip install pymycobot'
import time

# if use PC and M5 control
mc = MyCobot280('COM9', 115200)                 # WINDOWS use ，need check port number when you PC
# mc = MyCobot('/dev/ttyUSB0',115200)        #VM linux use

#init robot
mc.power_on()
time.sleep(1)

#define pump pin and work state
pump_motor_pin = 5                           # control vacuum pump motor work pin, can be modified
pump_relay_pin = 2                           # control vacuum pump relay work pin, can be modified
pump_open = 0                                # control vacuum pump motor work turn on
pump_close = 1                               # control vacuum pump motor work turn off

#define api, If you are using the first generation or 
#earlier suction pump, then define his interface as follows:
def pump_V1_on():
    mc.set_basic_output(pump_motor_pin, pump_open)
    time.sleep(0.05)
    mc.set_basic_output(pump_relay_pin, pump_open)
    time.sleep(0.05)

def pump_V1_off():
    mc.set_basic_output(pump_motor_pin, pump_close)
    time.sleep(0.05)
    mc.set_basic_output(pump_relay_pin, pump_close)
    time.sleep(0.05)

#If you are using the suction pump V2.0 version of 
# the device, then define his interface as follows:
def pump_V2_on():
    mc.set_basic_output(pump_motor_pin, pump_open)
    time.sleep(0.05)

def pump_V2_off():
    mc.set_basic_output(pump_motor_pin, pump_close)
    time.sleep(0.05)
    mc.set_basic_output(pump_relay_pin, pump_open)
    time.sleep(1)
    mc.set_basic_output(pump_relay_pin, pump_close)
    time.sleep(0.05)


# demo control, First turn on the vacuum pump motor, 
# Then turn off the motor and turn on the relay to 
# introduce air.If you do not know the version of 
# your suction pump, please use it after checking 
# the instructions of the suction pump;

# define pump V1.0 work demo
# for i in range (5):
#     pump_V1_on()
#     time.sleep(5)
#     pump_V1_off()

# define pump V2.0 work demo
for i in range (5):
    pump_V2_on()
    time.sleep(5)
    pump_V2_off()