# coding=utf8
import time
from pymycobot.mybuddy import MyBuddy

port = '/dev/ttyAMA0'
mb = MyBuddy(port)

pid = [21, 22, 23]


# 出厂
def set_pid_factory():
    pid_data = [25, 25, 1]
    for i in range(1, 3):  # 左右臂
        for j in range(1, 7):  # 舵机
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # 腰部pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


# 拖动示教
def set_pid_drag():
    pid_data = [10, 0, 1]
    for i in range(1, 3):  # 左右臂
        for j in range(1, 7):  # 舵机
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # 腰部pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


# 高精度
def set_pid_precision():
    pid_data = [32, 8, 0]
    for i in range(1, 3):  # 左右臂
        for j in range(1, 7):  # 舵机
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # 腰部pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


def get_pid():
    for i in range(1, 3):  # 左右臂
        for j in range(1, 7):  # 舵机
            for p in range(3):  # pid
                data = mb.get_servo_data(i, j, pid[p])
                arm = '右臂'
                if i == 1:
                    arm = '左臂'
                print(f'{arm},关节{j}，data_id：{pid[p]}   data: {data}')
                time.sleep(0.05)

    for p in range(3):  # 腰部pid
        data = mb.get_servo_data(3, 1, pid[p])
        print(f'腰部,data_id：{pid[p]}   data: {data}')
        time.sleep(0.05)


if __name__ == '__main__':
    key = 0
    status = True
    while status:
        print('请选择PID操作=======================\n'
              '1  设置出厂PID\n'
              '2  设置高精度PID\n'
              '3  设置拖动示教（高稳定）PID\n'
              '4  读取当前PID\n'
              '==================================================')
        print('请选择数字1-4，按下ENTER键确认：')
        key = input()
        key = int(key)
        if key <= 0 or key > 4:
            print('没有该选项，请重新选择！')
        else:
            status = False
    if key != 4:
        print('正在设置中！！')
    if key == 1:
        set_pid_factory()
    elif key == 2:
        set_pid_precision()
    elif key == 3:
        set_pid_drag()
    elif key == 4:
        get_pid()
    if key != 4:
        print('设置完成！！')
