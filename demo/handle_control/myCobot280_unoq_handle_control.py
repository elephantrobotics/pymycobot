# coding:utf-8
# Arduino UNO Q Bridge joystick control demo for myCobot280.
import sys
import threading
import time

import pygame
from pymycobot import MyCobot280


# UNO Q Debian/App Lab local control. The default UNO Q Bridge baudrate is 1000000.
mc = MyCobot280(unoq_bridge=True)
robot_lock = threading.RLock()

INIT_ANGLES = [0, 0, -90, 0, 0, 0]
GO_HOME = [0, 0, 0, 0, 0, 0]
JOG_SPEED = 50
DEAD_ZONE = 0.3
JOYSTICK_RETRY_SECONDS = 2.0

AXIS_MAP = {
    # Xbox 360 Controller on UNO Q / pygame:
    # left stick horizontal=axis 0, left stick vertical=axis 1,
    # left trigger=axis 2, right stick horizontal=axis 3,
    # right stick vertical=axis 4, right trigger=axis 5.
    "x": 1,
    "y": 0,
    "z": 4,
    "rz": 3,
}
RELEASE_AXIS = 2
POWER_AXIS = 5

BUTTON_MAP = {
    # A=0, B=1, X=2, Y=3, L1=4, R1=5.
    "gripper_open": 2,
    "gripper_close": 3,
    "pump_on": 0,
    "pump_off": 1,
    "to_init": 5,
    "to_home": 4,
}

COORD_AXIS_ACTIONS = {
    AXIS_MAP["y"]: (2, 1, 0),
    AXIS_MAP["x"]: (1, 1, 0),
    AXIS_MAP["z"]: (3, 0, 1),
    AXIS_MAP["rz"]: (6, 0, 1),
}

HAT_ACTIONS = {
    (0, -1): (4, 0),
    (0, 1): (4, 1),
    (-1, 0): (5, 1),
    (1, 0): (5, 0),
}

COORD_NAMES = {
    1: "X",
    2: "Y",
    3: "Z",
    4: "RX",
    5: "RY",
    6: "RZ",
}

previous_axis_state = {}
previous_hat = (0, 0)
stop_thread = None
joystick = None


def robot_call(name, *args):
    with robot_lock:
        return getattr(mc, name)(*args)


def log_action(label, api_name, *args):
    print("{} -> {}({})".format(label, api_name, ", ".join(str(arg) for arg in args)))


def pump_on():
    log_action("A 按钮：打开吸泵", "set_digital_output", 33, 0)
    robot_call("set_digital_output", 33, 0)
    time.sleep(0.05)


def pump_off():
    log_action("B 按钮：关闭吸泵", "set_digital_output", 33, 1)
    robot_call("set_digital_output", 33, 1)
    time.sleep(0.05)
    log_action("B 按钮：打开泄气阀", "set_digital_output", 23, 0)
    robot_call("set_digital_output", 23, 0)
    time.sleep(1)
    log_action("B 按钮：关闭泄气阀", "set_digital_output", 23, 1)
    robot_call("set_digital_output", 23, 1)
    time.sleep(0.05)


def safe_stop():
    try:
        log_action("摇杆/十字键回中：停止运动", "stop")
        robot_call("stop")
        time.sleep(0.02)
    except Exception as exc:
        print("stop 出错：", exc)


def request_stop():
    global stop_thread
    if stop_thread and stop_thread.is_alive():
        return
    stop_thread = threading.Thread(target=safe_stop, daemon=True)
    stop_thread.start()


def reset_joystick_state():
    global previous_hat
    previous_axis_state.clear()
    previous_hat = (0, 0)


def wait_for_joystick():
    global joystick
    while True:
        for wait_event in pygame.event.get():
            if wait_event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            reset_joystick_state()
            print("检测到手柄：{}".format(joystick.get_name()))
            pygame.event.clear()
            return

        print("未检测到手柄 USB 接收器，{} 秒后继续检测...".format(JOYSTICK_RETRY_SECONDS))
        time.sleep(JOYSTICK_RETRY_SECONDS)


def axis_direction(value):
    if value > DEAD_ZONE:
        return 1
    if value < -DEAD_ZONE:
        return -1
    return 0


def handle_axis_motion(event):
    axis = event.axis
    direction = axis_direction(event.value)

    if previous_axis_state.get(axis, 0) == direction:
        return
    previous_axis_state[axis] = direction

    if direction == 0:
        request_stop()
        return

    if axis in COORD_AXIS_ACTIONS:
        coord_id, negative_dir, positive_dir = COORD_AXIS_ACTIONS[axis]
        move_dir = positive_dir if direction > 0 else negative_dir
        label = "axis {} value {:.2f}：{}方向{}".format(
            axis,
            event.value,
            COORD_NAMES.get(coord_id, coord_id),
            "增加" if move_dir == 1 else "减小",
        )
        log_action(label, "jog_coord", coord_id, move_dir, JOG_SPEED)
        robot_call("jog_coord", coord_id, move_dir, JOG_SPEED)
    elif axis == RELEASE_AXIS and direction > 0:
        log_action("L2 扳机：释放所有关节", "release_all_servos")
        robot_call("release_all_servos")
        time.sleep(0.03)
    elif axis == POWER_AXIS and direction > 0:
        log_action("R2 扳机：上电锁定关节", "power_on")
        robot_call("power_on")
        time.sleep(0.03)


def handle_button_down():
    if joystick is None:
        return

    if joystick.get_button(BUTTON_MAP["gripper_open"]):
        log_action("X 按钮：夹爪张开", "set_gripper_state", 0, 100, 1)
        robot_call("set_gripper_state", 0, 100, 1)
    elif joystick.get_button(BUTTON_MAP["gripper_close"]):
        log_action("Y 按钮：夹爪关闭", "set_gripper_state", 1, 100, 1)
        robot_call("set_gripper_state", 1, 100, 1)
    elif joystick.get_button(BUTTON_MAP["pump_on"]):
        pump_on()
    elif joystick.get_button(BUTTON_MAP["pump_off"]):
        pump_off()
    elif joystick.get_button(BUTTON_MAP["to_init"]):
        log_action("R1 按钮：运动到初始点", "send_angles", INIT_ANGLES, JOG_SPEED)
        robot_call("send_angles", INIT_ANGLES, JOG_SPEED)
        time.sleep(2)
    elif joystick.get_button(BUTTON_MAP["to_home"]):
        log_action("L1 按钮：运动到零位", "send_angles", GO_HOME, JOG_SPEED)
        robot_call("send_angles", GO_HOME, JOG_SPEED)
        time.sleep(3)


def handle_hat_motion():
    global previous_hat
    if joystick is None:
        return

    hat_value = joystick.get_hat(0)
    if hat_value == previous_hat:
        return
    previous_hat = hat_value

    if hat_value == (0, 0):
        request_stop()
        return

    if hat_value in HAT_ACTIONS:
        coord_id, move_dir = HAT_ACTIONS[hat_value]
        label = "十字键 {}：{}方向{}".format(
            hat_value,
            COORD_NAMES.get(coord_id, coord_id),
            "增加" if move_dir == 1 else "减小",
        )
        log_action(label, "jog_coord", coord_id, move_dir, JOG_SPEED)
        robot_call("jog_coord", coord_id, move_dir, JOG_SPEED)


def joy_handler(event):
    device_added_event = getattr(pygame, "JOYDEVICEADDED", None)
    device_removed_event = getattr(pygame, "JOYDEVICEREMOVED", None)

    if event.type == device_added_event and joystick is None:
        wait_for_joystick()
        return
    if event.type == device_removed_event:
        handle_joystick_disconnect()
        return

    if event.type == pygame.JOYAXISMOTION:
        handle_axis_motion(event)
    elif event.type == pygame.JOYBUTTONDOWN:
        handle_button_down()
    elif event.type == pygame.JOYHATMOTION:
        handle_hat_motion()


def handle_joystick_disconnect():
    global joystick
    print("手柄已断开，停止机械臂并等待重新连接。")
    request_stop()
    joystick = None
    reset_joystick_state()


pygame.init()
pygame.joystick.init()
wait_for_joystick()
print("UNO Q 手柄控制已启动，按 Ctrl+C 退出。")

running = True
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            else:
                try:
                    joy_handler(event)
                except pygame.error as exc:
                    print("读取手柄失败：{}，等待重新连接。".format(exc))
                    handle_joystick_disconnect()

        if joystick is None and running:
            wait_for_joystick()
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n收到 Ctrl+C，停止机械臂并退出。")
finally:
    try:
        request_stop()
        if stop_thread:
            stop_thread.join(timeout=0.5)
    finally:
        pygame.quit()
