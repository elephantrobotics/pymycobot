import time
from pymycobot.myarm import MyArm
import RPi.GPIO as GPIO

mc = MyArm('/dev/ttyAMA0')

# 初始化
GPIO.setmode(GPIO.BCM)
# 引脚20/21分别控制电磁阀和泄气阀门
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

init_angles = [
    [-60, 0, 0, -90, 0, -90, 0],  # first init point
    [0, 0, 0, -90, 0, -90, 0],  # second init point
]

box_angles = [
    [-47.9, 17.31, 0.17, -89.91, -0.17, -56.07, 0.0],  # D
    [-27.59, 44.82, -1.75, -48.95, 0.0, -55.89, -0.08],  # C
    [52.99, 18.36, -1.4, -86.57, -0.17, -55.89, -0.08],  # A
    [87.97, 18.28, -1.4, -86.57, -0.35, -71.19, -0.08],  # B
]


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


def move():
    """
    myarm使用吸泵模拟aikitV2套装抓取木块
    """
    #mc.send_angles(init_angles[0], 50)
    #time.sleep(3)

    mc.send_angles(init_angles[1], 50)
    time.sleep(3)

    #mc.send_angles([0, 22, 0, -70, 0, -82, 0], 30)
    mc.send_coords([176.3, -1.5, 201.3, -179.89, 3.6, 179.49], 30)
    time.sleep(3)

    #mc.send_angles([-0.35, 37.79, -0.7, -90.43, -0.35, -45.96, -0.26], 30)
    mc.send_coords([177.8, -3.5, 114, -179.26, 5.79, 179.64], 30)
    time.sleep(2.5)
    pump_on()
    time.sleep(3)
    tmp = []
    while True:
        if not tmp:
            tmp = mc.get_angles()
        else:
            break
    time.sleep(0.5)
    mc.send_angles([tmp[0], 0, 0, -90, -0.79, -90, tmp[6]], 50)
    # mc.send_angles([0, 0, 0, -90, 0, -90, 0], 50)
    time.sleep(4)
    mc.send_angles(box_angles[3], 50)
    time.sleep(4)
    pump_off()
    time.sleep(2)
    mc.send_angles(init_angles[0], 50)
    time.sleep(4)


if __name__ == '__main__':
    pump_off()
    move()
