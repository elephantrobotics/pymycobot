# coding:utf-8
import pygame
import time
from pymycobot import MyCobot
import threading

mc = MyCobot("com15")
command = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]
zero = [144.8, -66.9, 185.3, 178.47, 0.87, -115.07]
action = 0
speed = 5
is_enable = False
is_zero = False
is_move_jog = False
gripper_value = 1
lock = threading.Lock()
axis_list = []


def control():
    global command, action, gripper_value, is_enable, is_zero, is_move_jog, speed
    while True:
        if is_enable and is_zero:
            if action == 1 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(1, 1, speed)
                is_move_jog = True
                print("x++")
                lock.release()
            elif action == 2 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(1, 0, speed)
                is_move_jog = True
                print("x--")
                lock.release()
            elif action == 3 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(2, 1, speed)
                is_move_jog = True
                print("y++")
                
                lock.release()
            elif action == 4 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(2, 0, speed)
                is_move_jog = True
                print("y--")
                
                lock.release()
            elif action == 5 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(3, 1, speed)
                is_move_jog = True
                print("z++")
                lock.release()
            elif action == 6 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(3, 0, speed)
                is_move_jog = True
                print("z--")
                lock.release()
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
            elif action == 12 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(4, 1, speed)
                print("rx++")
                is_move_jog = True
                lock.release()
            elif action == 13 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(4, 0, speed)
                print("rx--")
                is_move_jog = True
                lock.release()

            elif action == 14 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(5, 1, speed)
                print("ry++")
                is_move_jog = True
                lock.release()
            elif action == 15 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(5, 0, speed)
                print("ry--")
                is_move_jog = True
                lock.release()
            elif action == 16 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(6, 1, speed)
                print("rz++")
                is_move_jog = True
                lock.release()
            elif action == 17 and is_move_jog == False:
                lock.acquire()
                mc.jog_coord(6, 0, speed)
                print("rz--")
                is_move_jog = True
                lock.release()
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
            elif action == 22:
                lock.acquire()
                # print("stop")
                mc.jog_stop()
                print("stop move")
                is_move_jog = False
                action = 0
                lock.release()
            else:
                pass
        else:
            # print(action)
            if action == 7:
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
                    mc.power_on()
                    is_enable = True
                action = 0
            elif action == 21:
                time.sleep(2)
                if action == 21:
                    mc.send_coords(zero, 20)
                    print("初始化")
                    command = zero.copy()
                    action = 0
                    is_zero = True
        time.sleep(0.03)
        
        
def jog_control(axis, button, centre):
    global action, axis_list, is_move_jog
    print(axis, centre, is_move_jog)
    
    if axis == centre and is_move_jog:
        lock.acquire()
        print(" 221 ")
        action = 22
        lock.release()
        return 1
    if axis == centre:
        return 0
    if axis < centre:
        print("button: ", button)
        if button == 1:
            lock.acquire()
            action = 1
            lock.release()
        elif button == 3:
            lock.acquire()
            action = 5
            lock.release()
        elif button == 0:
            lock.acquire()
            action = 3
            lock.release()
        elif button == 2:
            lock.acquire()
            action = 17
            lock.release()
        if len(axis_list) < 2:
            axis_list.append(axis)
        else:
            print(axis, axis_list)
            
            if axis <= axis_list[1] <= axis_list[0]:
                axis_list = []
                return 1
            else:
                lock.acquire()
                action = 22
                lock.release()
                axis_list = []
                return 1
        return 1
    if axis > centre:
        if button == 1:
            lock.acquire()
            action = 2
            lock.release()
        elif button == 3:
            lock.acquire()
            action = 6
            lock.release()
        elif button == 0:
            lock.acquire()
            action = 4
            lock.release()
        elif button == 2:
            lock.acquire()
            action = 16
            lock.release()
        if len(axis_list) < 2:
            axis_list.append(axis)
        else:
            print(axis, axis_list)
            if axis >= axis_list[1] >= axis_list[0]:
                
                axis_list = []
                return 1
            else:
                lock.acquire()
                print(" 222 ")
                action = 22
                lock.release()
                axis_list = []
                return 1
        return 1


def main():
    global action, is_move_jog
    # 模块初始化
    pygame.init()
    pygame.joystick.init()

    # 若只连接了一个手柄，此处带入的参数一般都是0
    try:
        joystick = pygame.joystick.Joystick(0)
    except:
        print("请先连接手柄")
        return
    # 手柄对象初始化
    joystick.init()

    done = False

    start_time = 0
    axis_list = []
    
    while not done:
        for event_ in pygame.event.get():
            # 退出事件
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
                    if i == 0:
                        if axis < 0 and is_move_jog == False:
                            lock.acquire()
                            action = 3
                            lock.release()
                            break
                        elif axis > 0 and is_move_jog == False:
                            lock.acquire()
                            action = 4
                            lock.release()
                            break
                        elif axis == 0 and action in [3, 4]:
                            lock.acquire()
                            action = 22
                            lock.release()
                    # res[i] = axis
                    if i == 1:
                        # print("axis " + str(1) + ": " + str(axis))
                        if axis < -3.0517578125e-05 and is_move_jog == False:
                            lock.acquire()
                            action = 1
                            lock.release()
                            break
                        elif axis > -3.0517578125e-05 and is_move_jog == False:
                            lock.acquire()
                            action = 2
                            lock.release()
                            break
                        elif axis == -3.0517578125e-05 and action in [1,2]:
                            lock.acquire()
                            # print("22")
                            action = 22
                            lock.release()

                    if i == 2:
                        if axis < 0 and is_move_jog == False:
                            lock.acquire()
                            action = 17
                            lock.release()
                            break
                        elif axis > 0 and is_move_jog == False:
                            lock.acquire()
                            action = 16
                            lock.release()
                            break
                        elif axis == 0 and action in [16, 17]:
                            lock.acquire()
                            action = 22
                            lock.release()
                        
                    if i == 3:
                        # print("axis " + str(1) + ": " + str(axis))
                        if axis < -3.0517578125e-05 and is_move_jog == False:
                            lock.acquire()
                            action = 5
                            lock.release()
                            break
                        elif axis > -3.0517578125e-05 and is_move_jog == False:
                            lock.acquire()
                            action = 6
                            lock.release()
                            break
                        elif axis == -3.0517578125e-05 and action in [5,6]:
                            lock.acquire()
                            action = 22
                            lock.release()
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
                               
                # print("axis " + str(1) + ": " + str(axis))
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
                    action = 22

        # joystick_count = pygame.joystick.get_count()

    pygame.quit()


"""
axis 1(-1): x++
axis 1(1): x--
axis 0(-1): y++
axis 0(1): y--
axis 3(-1): z++ 零位-3.0517578125e-05
axis 3(1): z--
axis 2(-1): rz-- 
axis 2(1): rz++


button 3: 1 z++
button 0: 1 z--
button 2: 1 夹爪关闭
button 1: 1 夹爪打开

方向↑：hat 0 按下（0,1），弹起（0,0）
方向↓：hat 0 按下（0,-1），弹起（0,0）
方向←：hat 0 按下（-1,0），弹起（0,0）
方向→：hat 0 按下（1,0），弹起（0,0）

Y: 3(1)按下
X: 2(1)按下
A: 0(1)按下
B: 1(1)按下
左1: 4(1)按下
左2：axis 4(1)按下， (-1)弹起
右1:5(1)按下
右2：axis 5(1)按下， (-1)弹起
"""
if __name__ == "__main__":
    t = threading.Thread(target=control)
    t.daemon = True
    t.start()
    main()
