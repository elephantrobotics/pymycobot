from pymycobot.ultraArm import ultraArm
import time
import serial
import serial.tools.list_ports

#以上需写在代码开头，意为导入项目包

# ultraArm 类初始化需要两个参数：串口和波特率
#   第一个是串口字符串， 如：
#       linux： "/dev/ttyUSB0"
#       windows: "COM3"
#   第二个是波特率：115200
#   以下为如:
#           linux:
#              ua = ultraArm("/dev/USB0", 115200)
#           windows:
#              ua = ultraArm("COM3", 115200)

# 获取串口列表
plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

# 初始化一个ultraArm对象
# 下面为 windows版本创建对象代码

ua = ultraArm(plist[0], 115200)
# ultraArm进行坐标运动/角度运动之前必须进行回零，否则无法获取到正确的角度/坐标
ua.go_zero()
time.sleep(0.5)

# 通过传递角度参数，让机械臂每个关节移动到对应[0, 0, 0]的位置
ua.set_angles([0, 0, 0], 50)

# 设置等待时间，确保机械臂已经到达指定位置
time.sleep(2.5)

# 让关节1移动到90这个位置
ua.set_angle(1, 90, 50)
# 设置等待时间，确保机械臂已经到达指定位置
time.sleep(2)

# 以下代码可以让机械臂左右摇摆
# 设置循环次数
num = 7

while num > 0:
    # 让关节2移动到45这个位置
    ua.send_angle(2, 45, 50)

    # 设置等待时间，确保机械臂已经到达指定位置
    time.sleep(3)

    # 让关节2移动到-15这个位置
    ua.set_angle(2, -15, 50)

    # 设置等待时间，确保机械臂已经到达指定位置
    time.sleep(3)

    num -= 1

# 让机械臂缩起来。你可以手动摆动机械臂，然后使用get_angles()函数获得坐标数列，
# 通过该函数让机械臂到达你所想的位置。
ua.set_angles([88.68, 60, 30], 50)

# 设置等待时间，确保机械臂已经到达指定位置
time.sleep(2.5)