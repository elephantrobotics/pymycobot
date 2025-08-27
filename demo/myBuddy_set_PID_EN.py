# coding=utf8
import time
from pymycobot.mybuddy import MyBuddy

port = '/dev/ttyAMA0'
mb = MyBuddy(port)

pid = [21, 22, 23]


# 出厂
def set_pid_factory():
    pid_data = [25, 25, 1]
    for i in range(1, 3):  # left and right arms
        for j in range(1, 7):  # Servo
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # waist pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


# 拖动示教
def set_pid_drag():
    pid_data = [10, 0, 1]
    for i in range(1, 3):  # left and right arms
        for j in range(1, 7):  # Servo
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # 腰部pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


# 高精度
def set_pid_precision():
    pid_data = [32, 8, 0]
    for i in range(1, 3):  # left and right arms
        for j in range(1, 7):  # Servo
            for p in range(3):  # pid
                mb.set_servo_data(i, j, pid[p], pid_data[p])
                time.sleep(0.02)

    for p in range(3):  # waist pid
        mb.set_servo_data(3, 1, pid[p], pid_data[p])
        time.sleep(0.02)


def get_pid():
    for i in range(1, 3):  # left and right arms
        for j in range(1, 7):  # Servo
            for p in range(3):  # pid
                data = mb.get_servo_data(i, j, pid[p])
                arm = 'right arm'
                if i == 1:
                    arm = 'left arm'
                print(f'{arm},Joint{j}，data_id：{pid[p]}   data: {data}')
                time.sleep(0.05)

    for p in range(3):  # waist pid
        data = mb.get_servo_data(3, 1, pid[p])
        print(f'# waist,data_id：{pid[p]}   data: {data}')
        time.sleep(0.05)


if __name__ == '__main__':
    key = 0
    status = True
    while status:
        print('Please select PID operation=======================\n'
              '1  Set factory PID\n'
              '2  Set high precision PID\n'
              '3  Set drag teach (high stability) PID\n'
              '4  read current PID\n'
              '==================================================')
        print('Please select numbers 1-4 and press ENTER to confirm：')
        key = input()
        key = int(key)
        if key <= 0 or key > 4:
            print('No such option, please choose again!')
        else:
            status = False
    if key != 4:
        print('Setup is in progress! !')
    if key == 1:
        set_pid_factory()
    elif key == 2:
        set_pid_precision()
    elif key == 3:
        set_pid_drag()
    elif key == 4:
        get_pid()
    if key != 4:
        print('Setup is complete! !')
