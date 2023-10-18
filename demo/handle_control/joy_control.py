import pygame
import time
from pymycobot import MyCobot
from threading import Thread
from enum import Enum
import typing as T

mc = MyCobot("/dev/ttyAMA0")
context = {"running": True}
arm_speed = 50
sampling_rate = 10


class JoyStickKey(Enum):
    StartKey = 7
    SelectKey = 6
    ModeKey = 8
    RLeftKey = 2
    RRightKey = 1
    RTopKey = 3
    RBottomKey = 0
    R1 = 5
    L1 = 4
    LJoyStickKey = 9
    RJoyStickKey = 10
    ArrowUp = (0, 1)
    ArrowDown = (0, -1)
    ArrowLeft = (-1, 0)
    ArrowRight = (1, 0)
    ArrowReleased = (0, 0)


class JoyStickContinous(Enum):
    LeftXAxis = 0
    LeftYAxis = 1
    L2 = 2
    RightXAxis = 3
    RightYAxis = 4
    R2 = 5


joystick_key_map = {
    0: JoyStickKey.RBottomKey,
    1: JoyStickKey.RRightKey,
    2: JoyStickKey.RLeftKey,
    3: JoyStickKey.RTopKey,
    4: JoyStickKey.L1,
    5: JoyStickKey.R1,
    6: JoyStickKey.SelectKey,
    7: JoyStickKey.StartKey,
    8: JoyStickKey.ModeKey,
    9: JoyStickKey.LJoyStickKey,
    10: JoyStickKey.RJoyStickKey,
    (0, 1): JoyStickKey.ArrowUp,
    (0, -1): JoyStickKey.ArrowDown,
    (1, 0): JoyStickKey.ArrowRight,
    (-1, 0): JoyStickKey.ArrowLeft,
    (0, 0): JoyStickKey.ArrowReleased,
}

joystick_continous_map = {
    0: JoyStickContinous.LeftXAxis,
    1: JoyStickContinous.LeftYAxis,
    2: JoyStickContinous.L2,
    3: JoyStickContinous.RightXAxis,
    4: JoyStickContinous.RightYAxis,
    5: JoyStickContinous.R2,
}


def get_init_key_hold_timestamp():
    return {
        JoyStickKey.L1: -1,
        JoyStickKey.R1: -1,
        JoyStickContinous.L2: -1,
        JoyStickContinous.R2: -1,
    }


key_hold_timestamp = get_init_key_hold_timestamp()


def get_joystick():
    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
    except:
        print("Please connect the handle first.")
        exit(1)
    joystick.init()
    return joystick


def dispatch_key_action(key: T.Union[JoyStickKey, JoyStickContinous], value: float):
    global mc, key_hold_timestamp
    print(f"key {key}")

    if key == JoyStickKey.StartKey:
        if mc.is_all_servo_enable() != 1:
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(255, 0, 0)
            time.sleep(0.5)
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(255, 0, 0)
            time.sleep(0.5)
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(255, 0, 0)
        else:
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(0, 255, 0)
            time.sleep(0.5)
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(0, 255, 0)
            time.sleep(0.5)
            mc.set_color(0, 0, 0)
            time.sleep(0.5)
            mc.set_color(0, 255, 0)

    check_valid_key_hold = (
        lambda key: key_hold_timestamp[key] > 0
        and time.time() - key_hold_timestamp[key] > 2
    )
    check_valid_continous_hold = (
        lambda key: key_hold_timestamp[key] > 0
        and value > 0.5
        and time.time() - key_hold_timestamp[key] > 2
    )

    if key == JoyStickKey.L1:
        if check_valid_key_hold(key):
            mc.send_angles([0, 0, 0, 0, 0, 0], arm_speed)
    elif key == JoyStickKey.R1:
        if check_valid_key_hold(key):
            # TODO : 移动到初始点位
            pass
    elif key == JoyStickContinous.L2:
        if check_valid_continous_hold(key):
            mc.release_all_servos()
    elif key == JoyStickContinous.R2:
        if check_valid_continous_hold(key):
            mc.power_on()
    else:
        key_hold_timestamp = get_init_key_hold_timestamp()


def dispatch_continous_key_action(key: JoyStickContinous, value: float):
    print(f"{key}:{value}")


def retreive_joystick_input(joystick, context):
    while context["running"]:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                n = joystick.get_numbuttons()
                for key_id in range(n):
                    button_status = joystick.get_button(key_id)
                    if not button_status:
                        continue
                    dispatch_key_action(joystick_key_map[key_id], 1.0)

            elif event.type == pygame.JOYAXISMOTION:
                axes = joystick.get_numaxes()
                for key_id in range(axes):
                    axis = joystick.get_axis(key_id)
                    dispatch_continous_key_action(joystick_continous_map[key_id], axis)

            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                dispatch_key_action(joystick_key_map[hat], 1.0)


def get_input(joystick, context):
    while context["running"]:
        n = joystick.get_numbuttons()
        for key_id in range(n):
            button_status = joystick.get_button(key_id)
            if not button_status:
                continue
            dispatch_key_action(joystick_key_map[key_id], 1.0)

        axes = joystick.get_numaxes()
        for key_id in range(axes):
            axis = joystick.get_axis(key_id)
            if axis > 0.01:
                dispatch_key_action(joystick_continous_map[key_id], axis)

        hat = joystick.get_hat(0)
        # print(hat)
        # dispatch_key_action(joystick_key_map[hat], 1.0)

        # time.sleep(1 / sampling_rate)


if __name__ == "__main__":
    joystick = get_joystick()
    Thread(target=retreive_joystick_input, args=(joystick, context)).start()
