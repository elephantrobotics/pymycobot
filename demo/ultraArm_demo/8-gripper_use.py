import time
import platform
import serial
import serial.tools.list_ports
from pymycobot.ultraArm import ultraArm

plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

# 自动选择系统并连接机械臂
if platform.system() == "Windows":
    ua = ultraArm(plist[0], 115200)
    ua.go_zero()
elif platform.system() == "Linux":
    ua = ultraArm('/dev/ttyUSB0', 115200)
    ua.go_zero()

# 机械臂运动的位置
angles = [
    [-81.71, 0.0, 0.0],
    [-90.53, 21.77, 47.56],
    [-90.53, 0.0, 0.0],
    [-59.01, 21.77, 45.84],
    [-59.01, 0.0, 0.0]
]

ua.set_angles(angles[0], 50)
time.sleep(3)

i = 5
# 循环5次
while i > 0:
    # 张开夹爪
    ua.set_gripper_state(0)
    time.sleep(2)
    # 闭合夹爪
    ua.set_gripper_state(1)
    time.sleep(2)
    i -= 1