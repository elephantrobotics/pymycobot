import time
from pymycobot.myarm import MyArm


mc = MyArm('/dev/ttyAMA0')


init_angles = [
    [-60, 0, 0, -90, 0, -90, 0],  # first init point
    [0, 0, 0, -90, 0, -90, 0],  # second init point
]

box_angles = [
    [-47.9, 17.31, 0.17, -89.91, -0.17, -56.07, 0.0],  # D
    [-27.59, 44.82, -1.75, -48.95, 0.0, -55.89, -0.0],  # C
    [52.99, 18.36, -1.4, -86.57, -0.17, -55.89, -0.0],  # A
    [87.97, 18.28, -1.4, -86.57, -0.35, -71.19, -0.0],  # B
]


# 开启
def gripper_on():
    mc.set_gripper_state(0, 100)
    time.sleep(0.5)


# 关闭
def gripper_off():
    mc.set_gripper_state(1, 100)
    time.sleep(0.5)


def move():
    """
    myarm使用夹爪模拟aikitV2套装抓取木块
    """
    mc.send_angles(init_angles[0], 50)
    time.sleep(3)

    mc.send_angles(init_angles[1], 50)
    time.sleep(3)
    gripper_on()
    mc.send_angles([0.0, 26.27, 0.17, -72.86, -0.17, -77.51, 0.0], 50)
    time.sleep(3)

    # mc.send_angles([0.0, -47.63, 0.17, -77.43, 0.08, -55.72, 0.0], 50)
    mc.send_angles([-2.02, 41.74, 0.43, -86.13, -0.17, -46.05, 0.0], 50)
    

    time.sleep(3)

    gripper_off()
    time.sleep(2)
    tmp = []
    while True:
        if not tmp:
            tmp = mc.get_angles()
        else:
            break
    time.sleep(0.5)
    tmp[6] = 0.0
    print(tmp)
    mc.send_angles([tmp[0], 0, 0, -90, -0.0, -90, tmp[6]], 50)
    # mc.send_angles([0, 0, 0, -90, 0, -90, 0], 50)
    time.sleep(3)
    mc.send_angles(box_angles[3], 50)
    time.sleep(4)
    gripper_on()
    time.sleep(2)
    mc.send_angles(init_angles[0], 50)
    time.sleep(4)


if __name__ == '__main__':
    gripper_off()
    move()
