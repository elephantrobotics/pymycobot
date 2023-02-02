# coding:utf-8
import pygame
import time
from pymycobot import MyCobot
import threading

mc = MyCobot("com7")

# The default initial point can be changed, if you want to change, you should change 'command' and 'zero' at the same time
command = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]
zero = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]

action = 0
speed = 80
is_enable = False
is_zero = False
gripper_value = 1


def control():
    global command, action, gripper_value, is_enable, is_zero
    while True:
        if is_enable and is_zero:
            if action == 1:
                command[0] += 1
                if command[0] > 280:
                    command[0] = 279
                # mc.send_coords(command, speed)
                mc.send_coord(1, command[0], speed)
            elif action == 2:
                command[0] -= 1
                if command[0] < -280:
                    command[0] = -279
                # mc.send_coords(command, speed)
                mc.send_coord(1, command[0], speed)
            elif action == 3:
                command[1] += 1
                if command[1] > 280:
                    command[1] = 279
                mc.send_coord(2, command[1], speed)
            elif action == 4:
                command[1] -= 1
                if command[1] < -280:
                    command[1] = -279
                mc.send_coord(2, command[1], speed)
            elif action == 5:
                command[2] += 1
                if command[2] > 280:
                    command[2] = 279
                mc.send_coord(3, command[2], speed)
            elif action == 6:
                command[2] -= 1
                if command[2] < -280:
                    command[2] = -279
                mc.send_coord(3, command[2], speed)
            elif action == 7:
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
                    is_enable = False
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
                    is_enable = True
                action = 0
            elif action == 8:
                gripper_value += 2
                if gripper_value > 100:
                    gripper_value = 100
                mc.set_gripper_value(gripper_value, speed)
            elif action == 9:
                gripper_value -= 2
                if gripper_value < 0:
                    gripper_value = 0
                mc.set_gripper_value(gripper_value, speed)
            elif action == 10:
                mc.set_basic_output(5, 0)
                mc.set_basic_output(2, 0)
                action = 0
            elif action == 11:
                mc.set_basic_output(5, 1)
                mc.set_basic_output(2, 1)
                action = 0
            elif action == 12:
                command[3] += 1
                if command[3] > 180:
                    command[3] = 180
                mc.send_coord(4, command[3], speed)
            elif action == 13:
                command[3] -= 1
                if command[3] < -180:
                    command[3] = -180
                mc.send_coord(4, command[3], speed)

            elif action == 14:
                command[4] += 1
                if command[4] > 180:
                    command[4] = 180
                mc.send_coord(5, command[4], speed)
            elif action == 15:
                command[4] -= 1
                if command[4] < -180:
                    command[4] = -180
                mc.send_coord(5, command[4], speed)
            elif action == 16:
                command[5] += 1
                if command[5] > 180:
                    command[5] = 180
                mc.send_coord(6, command[5], speed)
            elif action == 17:
                command[5] -= 1
                if command[5] < -180:
                    command[5] = -180
                mc.send_coord(6, command[5], speed)
            elif action == 18:
                # start_time = time.time()
                time.sleep(2)
                if action == 18:
                    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
                    action = 0
            elif action == 19:
                # start_time = time.time()
                time.sleep(2)
                if action == 19:
                    mc.release_all_servos()
                    action = 0
            elif action == 20:
                time.sleep(2)
                if action == 20:
                    mc.power_on()
                    action = 0
            elif action == 21:
                time.sleep(2)
                if action == 21:
                    mc.send_coords(zero, 20)
                    command = zero.copy()
                    action = 0
            else:
                pass
        else:
            # print(action)
            if action == 7:
                statue = 0
                for _ in range(3):
                    statue = mc.is_all_servo_enable()
                    if statue:
                        break
                    time.sleep(0.1)
                if statue in [0, -1]:
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
                    is_enable = False
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
                    mc.power_on()
                    is_enable = True
                action = 0
            elif action == 21:
                time.sleep(2)
                if action == 21:
                    mc.send_coords(zero, 20)
                    command = zero.copy()
                    action = 0
                    is_zero = True
        time.sleep(0.05)


def main():
    global action
    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
    except:
        print("Please connect the handle first.")
        return
    joystick.init()

    done = False

    start_time = 0
    while not done:
        for event_ in pygame.event.get():
            if event_.type == pygame.QUIT:
                done = True
            # 按键按下或弹起事件
            elif (
                event_.type == pygame.JOYBUTTONDOWN or event_.type == pygame.JOYBUTTONUP
            ):
                buttons = joystick.get_numbuttons()
                # 获取所有按键状态信息
                for i in range(buttons):
                    button = joystick.get_button(i)
                    if i == 7:
                        if button == 1:
                            action = 7
                            break
                        else:
                            action = 0
                    if i == 1:
                        if button == 1:
                            action = 11
                            break
                    if i == 0:
                        if button == 1:
                            action = 10
                            break
                    if i == 3:
                        if button == 1:
                            action = 9
                            break
                        if action == 9 and button == 0:
                            action = 0
                            break
                    if i == 2:
                        if button == 1:
                            action = 8
                            break
                        if action == 8 and button == 0:
                            action = 0
                            break
                    if i == 4:
                        if button == 1:
                            action = 18
                            start_time = time.time()
                            break
                        if start_time != 0 and button == 0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
                    if i == 5:
                        if button == 1:
                            action = 21
                            start_time = time.time()
                            break
                        if start_time != 0 and button == 0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
                    # print("button " + str(i) + ": " + str(button))
            # 轴转动事件
            elif event_.type == pygame.JOYAXISMOTION:
                axes = joystick.get_numaxes()
                # 获取所有轴状态信息
                # while True:
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    # res[i] = axis
                    if i == 1:
                        if axis < -3.0517578125e-05:
                            action = 1
                            break
                        elif axis > -3.0517578125e-05:
                            action = 2
                            break
                        else:
                            action = 0
                    if i == 0:
                        if axis < 0:
                            action = 3
                            break
                        elif axis > 0:
                            action = 4
                            break
                        else:
                            action = 0
                    if i == 2:
                        if axis < 0:
                            action = 17
                            break
                        elif axis > 0:
                            action = 16
                            break
                        else:
                            action = 0
                    if i == 3:
                        if axis < -3.0517578125e-05:
                            action = 5
                            break
                        elif axis > -3.0517578125e-05:
                            action = 6
                            break
                        else:
                            action = 0
                    if i == 4:
                        if axis > 0.9:
                            action = 19
                            start_time = time.time()
                            break
                        if start_time != 0 and axis == -1.0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
                    if i == 5:
                        if axis > 0.9:
                            action = 20
                            start_time = time.time()
                            break
                        if start_time != 0 and axis == -1.0:
                            if time.time() - start_time > 2:
                                start_time = 0
                                break
                            else:
                                start_time = 0
                                action = 0
                    # print("axis " + str(i) + ": " + str(axis))
            # 方向键改变事件
            elif event_.type == pygame.JOYHATMOTION:
                # hats = joystick.get_numhats()
                # 获取所有方向键状态信息
                # for i in range(hats):
                hat = joystick.get_hat(0)
                # print("hat " + str(i) +": " + str(hat))
                if hat == (0, 1):
                    action = 12
                elif hat == (0, -1):
                    action = 13
                elif hat == (-1, 0):
                    action = 14
                elif hat == (1, 0):
                    action = 15
                elif hat == (0, 0):
                    action = 0

    pygame.quit()

if __name__ == "__main__":
    # pass
    t = threading.Thread(target=control)
    t.daemon = True
    t.start()
    main()
