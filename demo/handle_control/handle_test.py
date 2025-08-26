"""
handle_test.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2025-07-17
"""
# coding:utf-8
import pygame
import sys
import time
import platform
import threading
from pymycobot import MyCobot280

# Platform detection
IS_WINDOWS = platform.system() == "Windows"
IS_LINUX = platform.system() == "Linux"

# Port and baudrate config
if IS_WINDOWS:
    SERIAL_PORT = 'COM39'
    BAUDRATE = 115200
elif IS_LINUX:
    SERIAL_PORT = '/dev/ttyAMA0'
    BAUDRATE = 1000000
else:
    print("Unsupported platform")
    sys.exit(1)

# Axis mapping by platform
AXIS_MAP = {
    'x': 1,
    'y': 0,
    'z': 3 if IS_WINDOWS else 4,
    'rz': 2 if IS_WINDOWS else 3
}
RELEASE_AXIS = 4 if IS_WINDOWS else 2
POWER_AXIS = 5

# Initialize robot
mc = MyCobot280(SERIAL_PORT, BAUDRATE)
init_angles = [0, 0, -90, 0, 0, 0]
go_home = [0, 0, 0, 0, 0, 0]

# GPIO for Linux (Raspberry Pi)
if IS_LINUX:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)

# Pygame init
pygame.init()
pygame.joystick.init()
button_pressed = False
hat_pressed = False
previous_state = [0, 0, 0, 0, 0, 0]

# Pump control
def pump_on():
    if IS_LINUX:
        GPIO.output(20, 0)
    else:
        mc.set_basic_output(5, 0)
        time.sleep(0.05)

def pump_off():
    if IS_LINUX:
        GPIO.output(20, 1)
        time.sleep(0.05)
        GPIO.output(21, 0)
        time.sleep(1)
        GPIO.output(21, 1)
        time.sleep(0.05)
    else:
        mc.set_basic_output(5, 1)
        time.sleep(0.05)
        mc.set_basic_output(2, 0)
        time.sleep(1)
        mc.set_basic_output(2, 1)
        time.sleep(0.05)

# Safe stop thread
def safe_stop():
    try:
        mc.stop()
        time.sleep(0.02)
    except Exception as e:
        print("stop error:", e)

# Event handler
def joy_handler():
    global button_pressed, hat_pressed, previous_state

    if event.type == pygame.JOYAXISMOTION:
        axis = event.axis
        value = round(event.value, 2)

        if abs(value) > 0.1:
            previous_state[axis] = value
            if axis == AXIS_MAP['x']:
                mc.jog_coord(1, 1 if value < 0 else 0, 50)
            elif axis == AXIS_MAP['y']:
                mc.jog_coord(2, 0 if value > 0 else 1, 50)
            elif axis == AXIS_MAP['z']:
                mc.jog_coord(3, 1 if value > 0 else 0, 50)
            elif axis == AXIS_MAP['rz']:
                mc.jog_coord(6, 1 if value > 0 else 0, 50)
            elif axis == RELEASE_AXIS and value == 1.00:
                mc.release_all_servos()
                time.sleep(0.03)
            elif axis == POWER_AXIS and value == 1.00:
                mc.power_on()
                time.sleep(0.03)
        else:
            if previous_state[axis] != 0:
                threading.Thread(target=safe_stop).start()
                previous_state[axis] = 0

    elif event.type == pygame.JOYBUTTONDOWN:
        if joystick.get_button(2):
            mc.set_gripper_state(0, 100)
        elif joystick.get_button(3):
            mc.set_gripper_state(1, 100)
        elif joystick.get_button(0):
            pump_on()
        elif joystick.get_button(1):
            pump_off()
        elif joystick.get_button(5):
            mc.send_angles(init_angles, 50)
            time.sleep(2)
        elif joystick.get_button(4):
            mc.send_angles(go_home, 50)
            time.sleep(3)

    elif event.type == pygame.JOYHATMOTION:
        hat_value = joystick.get_hat(0)
        if hat_value == (0, -1):
            mc.jog_coord(4, 0, 50)
        elif hat_value == (0, 1):
            mc.jog_coord(4, 1, 50)
        elif hat_value == (-1, 0):
            mc.jog_coord(5, 1, 50)
        elif hat_value == (1, 0):
            mc.jog_coord(5, 0, 50)
        if hat_value != (0, 0):
            hat_pressed = True
        else:
            if hat_pressed:
                threading.Thread(target=safe_stop).start()
                hat_pressed = False

# Initialize joystick
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    print("No joystick detected")
    pygame.quit()
    sys.exit()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        joy_handler()

pygame.quit()
