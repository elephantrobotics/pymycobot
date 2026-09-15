# coding:utf-8
import glob
import os
import platform
import stat
import sys
import time

import pygame


RETRY_SECONDS = 2.0
AXIS_PRINT_THRESHOLD = 0.2
DIAG_INTERVAL = 5


def format_mode(path):
    try:
        mode = os.stat(path).st_mode
    except OSError as exc:
        return "stat failed: {}".format(exc)
    return stat.filemode(mode)


def print_input_diagnostics():
    print("pygame joystick count: {}".format(pygame.joystick.get_count()))
    if platform.system() == "Windows":
        print_windows_diagnostics()
    else:
        print_linux_diagnostics()


def print_windows_diagnostics():
    print("当前系统：Windows")
    print("如果 pygame 检测不到手柄，请先确认 Windows 已识别手柄：")
    print("  1. 打开“设备管理器”，查看是否有 Xbox 360 Controller 或游戏控制器设备。")
    print("  2. 按 Win+R，运行 joy.cpl，确认手柄能在“游戏控制器”列表中出现。")
    print("  3. 如果 joy.cpl 能识别但 pygame 不能识别，尝试重新插拔 USB 接收器或重启 Python 进程。")
    print("  4. 如使用虚拟环境，请确认当前环境已安装 pygame。")


def print_linux_diagnostics():
    js_devices = sorted(glob.glob("/dev/input/js*"))
    event_devices = sorted(glob.glob("/dev/input/event*"))
    input_names = sorted(glob.glob("/dev/input/by-id/*")) + sorted(glob.glob("/dev/input/by-path/*"))

    print("当前系统：{}".format(platform.system()))
    print("/dev/input/js*: {}".format(js_devices if js_devices else "none"))
    for path in js_devices:
        print("  {} {}".format(path, format_mode(path)))
    print("/dev/input/event* count: {}".format(len(event_devices)))
    if input_names:
        print("/dev/input links:")
        for path in input_names:
            try:
                target = os.readlink(path)
            except OSError:
                target = ""
            print("  {} -> {}".format(path, target))
    print("如果 lsusb 能看到手柄但这里没有 /dev/input/js0，通常是 joydev/xpad 没有创建 joystick 设备。")
    print("可检查：lsmod | grep -E 'joydev|xpad'")
    print("可尝试：sudo modprobe joydev")
    print("如仍没有 js0，再尝试：sudo modprobe xpad")
    print_proc_input_devices()


def print_proc_input_devices():
    path = "/proc/bus/input/devices"
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as input_file:
            content = input_file.read().strip()
    except OSError as exc:
        print("{} 读取失败：{}".format(path, exc))
        return

    if not content:
        print("{} 为空。".format(path))
        return

    print("{}:".format(path))
    for block in content.split("\n\n"):
        lower_block = block.lower()
        if "xbox" in lower_block or "joystick" in lower_block or "gamepad" in lower_block:
            print(block)


def wait_for_joystick():
    retry_count = 0
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print("检测到手柄：{}".format(joystick.get_name()))
            print("轴数量：{}".format(joystick.get_numaxes()))
            print("按钮数量：{}".format(joystick.get_numbuttons()))
            print("十字键数量：{}".format(joystick.get_numhats()))
            print("请依次按 A/B/X/Y、L/R、摇杆、十字键，按 Ctrl+C 退出。")
            pygame.event.clear()
            return joystick

        print("未检测到手柄 USB 接收器，{} 秒后继续检测...".format(RETRY_SECONDS))
        retry_count += 1
        if retry_count % DIAG_INTERVAL == 0:
            print_input_diagnostics()
        time.sleep(RETRY_SECONDS)


def print_button_event(event):
    state = "down" if event.type == pygame.JOYBUTTONDOWN else "up"
    print("button {} {}".format(event.button, state))


def print_axis_event(event):
    value = round(event.value, 3)
    if abs(value) >= AXIS_PRINT_THRESHOLD:
        print("axis {} value {}".format(event.axis, value))
    elif value == 0:
        print("axis {} value 0".format(event.axis))


def print_hat_event(event):
    print("hat {} value {}".format(event.hat, event.value))


def main():
    print("pygame version: {}".format(pygame.version.ver))
    print("SDL version: {}".format(pygame.get_sdl_version()))
    pygame.init()
    pygame.joystick.init()
    joystick = None

    try:
        joystick = wait_for_joystick()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                    print_button_event(event)
                elif event.type == pygame.JOYAXISMOTION:
                    print_axis_event(event)
                elif event.type == pygame.JOYHATMOTION:
                    print_hat_event(event)
                elif event.type == getattr(pygame, "JOYDEVICEREMOVED", None):
                    print("手柄已断开，等待重新连接。")
                    joystick = None
                    joystick = wait_for_joystick()
                elif event.type == getattr(pygame, "JOYDEVICEADDED", None) and joystick is None:
                    joystick = wait_for_joystick()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n退出手柄映射检测。")
    finally:
        if joystick:
            joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
