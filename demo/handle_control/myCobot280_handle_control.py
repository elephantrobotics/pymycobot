# coding:utf-8
import pygame
import sys
import time
from pymycobot import MyCobot280
from pymycobot.utils import get_port_list
import platform
import threading

IS_LINUX = "linux" in platform.platform().lower()
IS_WINDOWS = platform.system() == "Windows"

if IS_LINUX:
    import RPi.GPIO as GPIO

    # Set GPIO mode
    GPIO.setmode(GPIO.BCM)
    # Set pin 20 (electromagnetic valve) and pin 21 (release valve) as output
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)

# Initialize MyCobot280 with serial port and baud rate
if IS_WINDOWS:
    mc = MyCobot280(get_port_list()[0], 115200)  # M5
else:
    mc = MyCobot280('/dev/ttyAMA0', 1000000)  # PI

init_angles = [0, 0, -90, 0, 0, 0]
go_home = [0, 0, 0, 0, 0, 0]

# Initialize Pygame and joystick modules
pygame.init()
pygame.joystick.init()
button_pressed = False
hat_pressed = False
previous_state = [0, 0, 0, 0, 0, 0]

if IS_WINDOWS:
    AXIS_MAP = {
        'x': 1,
        'y': 0,
        'z': 3,
        'rz': 2,
    }
    RELEASE_AXIS = 4
    POWER_AXIS = 5
else:
    AXIS_MAP = {
        'x': 1,
        'y': 0,
        'z': 4,
        'rz': 3,
    }
    RELEASE_AXIS = 2
    POWER_AXIS = 5

BUTTON_MAP = {
    'gripper_open': 2,
    'gripper_close': 3,
    'pump_on': 0,
    'pump_off': 1,
    'to_init': 5,
    'to_home': 4,
}


# Function to turn on the vacuum pump (electromagnetic valve)
def pump_on():
    # 打开电磁阀
    if IS_LINUX:
        GPIO.output(20, 0)
    else:
        mc.set_basic_output(5, 0)
        time.sleep(0.05)


# Function to turn off the vacuum pump (close electromagnetic valve and open release)
def pump_off():
    if IS_LINUX:
        # Close valve
        GPIO.output(20, 1)
        time.sleep(0.05)
        # Open release valve
        GPIO.output(21, 0)
        time.sleep(1)
        GPIO.output(21, 1)
        time.sleep(0.05)
    else:
        # Close valve
        mc.set_basic_output(5, 1)
        time.sleep(0.05)
        # Open release valve
        mc.set_basic_output(2, 0)
        time.sleep(1)
        mc.set_basic_output(2, 1)
        time.sleep(0.05)


# Function to safely stop the robot (used in a thread)
def safe_stop():
    try:
        mc.stop()
        time.sleep(0.02)
    except Exception as e:
        print("stop 出错：", e)


# Handler for joystick input events
def joy_handler():
    global button_pressed
    global hat_pressed
    global previous_state
    # Joystick axis movement (analog stick)
    if event.type == pygame.JOYAXISMOTION:
        axis = event.axis
        value = round(event.value, 2)
        if abs(value) > 0.1:
            flag = True
            previous_state[axis] = value
            if axis == AXIS_MAP['y'] and value == -1.00:
                mc.jog_coord(2, 1, 50)
            elif axis == AXIS_MAP['y'] and value == 1.00:
                mc.jog_coord(2, 0, 50)
            if axis == AXIS_MAP['x'] and value == 1.00:
                mc.jog_coord(1, 0, 50)
            elif axis == AXIS_MAP['x'] and value == -1.00:
                mc.jog_coord(1, 1, 50)
            if axis == AXIS_MAP['z'] and value == 1.00:
                mc.jog_coord(3, 1, 50)
            elif axis == AXIS_MAP['z'] and value == -1.00:
                mc.jog_coord(3, 0, 50)
            if axis == AXIS_MAP['rz'] and value == 1.00:
                mc.jog_coord(6, 1, 50)
            elif axis == AXIS_MAP['rz'] and value == -1.00:
                mc.jog_coord(6, 0, 50)
            # Axis RELEASE_AXIS for servo release
            elif axis == RELEASE_AXIS and value == 1.00:
                mc.release_all_servos()
                time.sleep(0.03)
            # Axis POWER_AXIS to power on
            elif axis == POWER_AXIS and value == 1.00:
                mc.power_on()
                time.sleep(0.03)
        else:
            if previous_state[axis] != 0:
                # mc.stop()
                threading.Thread(target=safe_stop).start()
                previous_state[axis] = 0
    # Joystick button pressed
    elif event.type == pygame.JOYBUTTONDOWN:
        if joystick.get_button(BUTTON_MAP['gripper_open']):
            mc.set_gripper_state(0, 100)
        elif joystick.get_button(BUTTON_MAP['gripper_close']):
            mc.set_gripper_state(1, 100)
        elif joystick.get_button(BUTTON_MAP['pump_on']):
            pump_on()
        elif joystick.get_button(BUTTON_MAP['pump_off']):
            pump_off()
        elif joystick.get_button(BUTTON_MAP['to_init']):
            mc.send_angles(init_angles, 50)
            time.sleep(2)
        elif joystick.get_button(BUTTON_MAP['to_home']):
            mc.send_angles(go_home, 50)
            time.sleep(3)
    # D-Pad (HAT) movement
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
                # mc.stop()
                threading.Thread(target=safe_stop).start()
                hat_pressed = False


# Initialize joystick if detected
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    print("没有检测到手柄")
    pygame.quit()
    sys.exit()
# Main loop to process events
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        joy_handler()
pygame.quit()
