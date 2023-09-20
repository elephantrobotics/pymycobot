# coding:utf-8
import pygame
import time
from pymycobot import MyArm
import threading
import RPi.GPIO as GPIO

mc = MyArm("/dev/ttyAMA0", debug=True)

# The default initial point can be changed, if you want to change, you should change 'command' and 'zero' at the same time
# command = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]
# zero = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]
#command = [127.7, 0.8, 234.5, -179.9, 1.05, 0.52]
#zero = [127.7, 0.8, 234.5, -179.9, 1.05, 0.52]
command = [111.8, -63.8, 236.3, -179.82, -0.35, -29.18]
zero = [111.8, -63.8, 236.3, -179.82, -0.35, -29.18]

action = 0
speed = 80
is_enable = False
is_zero = False
gripper_value = 1
print('start handle_control')
# 初始化
#GPIO.setmode(GPIO.BCM)
# 引脚20/21分别控制电磁阀和泄气阀门
#GPIO.setup(20, GPIO.OUT)
#GPIO.setup(21, GPIO.OUT)


# 开启吸泵
def pump_on():
    # 打开电磁阀
    GPIO.output(20, 0)


# 停止吸泵
def pump_off():
    # 关闭电磁阀
    GPIO.output(20, 1)
    time.sleep(0.05)
    # 打开泄气阀门
    GPIO.output(21, 0)
    time.sleep(1)
    GPIO.output(21, 1)
    time.sleep(0.05)


def control():
    global command, action, gripper_value, is_enable, is_zero
    while True:
        if is_enable and is_zero:
            if action == 1:
                command[0] += 2
                if command[0] > 285:
                    command[0] = 280
                # mc.send_coords(command, speed)
                print('x + coords---->:', command[0])
                mc.send_coord(1, command[0], speed)
                #mc.send_angles([0, -30, 0, -90, 0, -60, 0], 30)
            elif action == 2:
                command[0] -= 2
                if command[0] < -285:
                    command[0] = -280
                print('x - coords---->:', command[0])
                # mc.send_coords(command, speed)
                mc.send_coord(1, command[0], speed)
                #mc.send_angles([0, 30, 0, -90, 0, -60, 0], 30)
            elif action == 3:
                command[1] += 2
                if command[1] > 285:
                    command[1] = 280
                print('y + coords---->:', command[1])
                mc.send_coord(2, command[1], speed)
                #mc.send_angles([60, -30, 0, -90, 0, -60, 0], 30)
            elif action == 4:
                command[1] -= 2
                if command[1] < -285:
                    command[1] = -280
                print('y - coords---->:', command[1])
                mc.send_coord(2, command[1], speed)
                #mc.send_angles([-60, -30, 0, -90, 0, -60, 0], 30)
            elif action == 5:
                command[2] += 2
                if command[2] > 450:
                    command[2] = 445
                print('z1111')
                mc.send_coord(3, command[2], speed)
            elif action == 6:
                command[2] -= 2
                if command[2] < -120:
                    command[2] = -118
                mc.send_coord(3, command[2], speed)
            elif action == 7:
                #is_enable = True
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
                    #is_enable = True
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
                #gripper_value += 5
                #if gripper_value > 100:
                  #  gripper_value = 100
                #mc.set_gripper_value(gripper_value, speed)
                command[2] += 2
                if command[2] > 450:
                    command[2] = 445
                print('z + coords---->:', command[2])
                mc.send_coord(3, command[2], speed)
            elif action == 9:
                #gripper_value -= 5
                #if gripper_value < 0:
                 #   gripper_value = 0
                #mc.set_gripper_value(gripper_value, speed)
                command[2] -= 2
                if command[2] < -120:
                    command[2] = -118
                print('z - coords---->:', command[2])
                mc.send_coord(3, command[2], speed)
            elif action == 10:
                pump_on()
                action = 0
            elif action == 11:
                pump_off()
                action = 0
            elif action == 12:
                command[3] += 10
                if command[3] > 285:
                    command[3] = 280
                mc.send_coord(4, command[3], speed)
            elif action == 13:
                command[3] -= 10
                if command[3] < -285:
                    command[3] = -280
                mc.send_coord(4, command[3], speed)

            elif action == 14:
                command[4] += 10
                if command[4] > 285:
                    command[4] = 280
                mc.send_coord(5, command[4], speed)
            elif action == 15:
                command[4] -= 10
                if command[4] < -285:
                    command[4] = -280
                mc.send_coord(5, command[4], speed)
            elif action == 16:
                command[5] += 10
                if command[5] > 450:
                    command[5] = 440
                print('z222')
                #mc.send_coord(6, command[5], speed)
            elif action == 17:
                command[5] -= 10
                if command[5] < -120:
                    command[5] = -118
                #mc.send_coord(6, command[5], speed)
            elif action == 18:
                # start_time = time.time()
                time.sleep(2)
                if action == 18:
                    mc.send_angles([0, 0, 0, 0, 0, 0, 0], 30)
                    time.sleep(2)
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
                    print('isisisisiisisisisisisisisiis:', is_enable)
                    mc.send_angles([-30, 0, 0, -90, 0, -90, 0], 30)
                    time.sleep(3)
                    # print('init_coords:', mc.get_coords())
                    # mc.send_coords(zero, 20)
                    command = zero.copy()
                    print('command', command)
                    action = 0
            else:
                pass
        else:
            # print(action)
            if action == 7:
                #is_enable = True
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
                # print('total:', buttons)
                # 获取所有按键状态信息
                for i in range(buttons):
                    button = joystick.get_button(i)
                    print('*' * 30)
                    print('i value:', i)
                    print('button---------->', button)

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
                    print("button " + str(i) + ": " + str(button))
                    print('action--->', action)
                    print('*' * 30)
                    print('\n')
            # 轴转动事件
            elif event_.type == pygame.JOYAXISMOTION:
                axes = joystick.get_numaxes()
                # 获取所有轴状态信息
                # while True:
                for i in range(axes):
                    axis = joystick.get_axis(i)
                    print('*' * 30)
                    print('i value:', i)
                    print('axis---------->', axis)
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
                    print("axis " + str(i) + ": " + str(axis))
                    print('action--->', action)
                    print('*' * 30)
                    print('\n')
            # 方向键改变事件
            elif event_.type == pygame.JOYHATMOTION:
                # hats = joystick.get_numhats()
                # 获取所有方向键状态信息
                # for i in range(hats):
                hat = joystick.get_hat(0)
                # print("hat " + str(i) +": " + str(hat))
                print('hat:', str(hat))
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
