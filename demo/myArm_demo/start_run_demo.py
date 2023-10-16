import time
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0')

time.sleep(0.1)


def main():
    """
    展示MyArm各个关节的运动状态
    :return: None
    """
    angles_list = [
        [98, 7, 68, -92, 164, 5, -2],
        [0, 35, 0, -75, 164, 5, -2],
        [0, 0, 0, -90, 165, -90, 2],
        [0, 0, 0, 0, 0, 0, 0],
    ]
    mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
    time.sleep(3)
    mc.send_angles([0, 0, 0, -100, 0, -78, 0], 60)
    time.sleep(3)

    mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
    time.sleep(3)
    mc.send_angle(1, 90, 60)
    time.sleep(3)

    for i in range(1, 8):
        if i == 1:
            mc.send_angle(i,140 , 60)
            time.sleep(3)
            mc.send_angle(i,40 , 60)
            time.sleep(3)
            mc.send_angle(i, 90, 60)
            time.sleep(2)
        elif i == 6:
            mc.send_angle(i,50 , 60)
            time.sleep(2)
            mc.send_angle(i, -50, 60)
            time.sleep(2)
            mc.send_angle(i, 0, 60)
            time.sleep(2)
        else:
            mc.send_angle(i, 50, 60)
            time.sleep(2)
            mc.send_angle(i, -50, 60)
            time.sleep(2)
            mc.send_angle(i, 0, 60)
            time.sleep(2)

    mc.send_angles(angles_list[0], 60)
    time.sleep(3)
    mc.send_angles(angles_list[1], 60)
    time.sleep(3)
    mc.send_angles(angles_list[2], 60)
    time.sleep(3)
    mc.send_angles(angles_list[3], 60)
    time.sleep(3)


if __name__ == '__main__':
    main()
