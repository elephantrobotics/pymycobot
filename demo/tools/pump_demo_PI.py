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
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
# if use Pi control
mc = MyCobot280('/dev/ttyAMA0',1000000)        #linux use

#init robot
mc.power_on()
time.sleep(1)

#define pump pin and work state
pump_motor_pin = 20                           # control vacuum pump motor work pin, can be modified
pump_relay_pin = 21                           # control vacuum pump relay work pin, can be modified
pump_open = 0                                # control vacuum pump motor work turn on
pump_close = 1                               # control vacuum pump motor work turn off

def init_gpio(io_number, state):
    if state == 0:
        st = GPIO.OUT
    elif state == 1:
        st = GPIO.IN
    GPIO.setup(io_number, st)

def init_pump_pin():
    init_gpio(pump_motor_pin,0)
    init_gpio(pump_relay_pin,0)
#define api, If you are using the first generation or 
#earlier suction pump, then define his interface as follows:
def pump_V1_on():
    GPIO.output(pump_motor_pin, pump_open)
    time.sleep(0.05)
    GPIO.output(pump_relay_pin, pump_open)
    time.sleep(0.05)

def pump_V1_off():
    GPIO.output(pump_motor_pin, pump_close)
    time.sleep(0.05)
    GPIO.output(pump_relay_pin, pump_close)
    time.sleep(0.05)

#If you are using the suction pump V2.0 version of 
# the device, then define his interface as follows:
def pump_V2_on():
    GPIO.output(pump_motor_pin, pump_open)
    time.sleep(0.05)

def pump_V2_off():
    GPIO.output(pump_motor_pin, pump_close)
    time.sleep(0.05)
    GPIO.output(pump_relay_pin, pump_open)
    time.sleep(1)
    GPIO.output(pump_relay_pin, pump_close)
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
init_pump_pin()
for i in range (5):
    pump_V2_on()
    time.sleep(5)
    pump_V2_off()


