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
#
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

# 获取当前头部的坐标以及姿态
coords = ua.get_coords_info()
time.sleep(2)
print(coords)

# # 让机械臂到达[57.0,-10,30]这个坐标，速度为80mm/s
ua.set_coords([57.0,-10,30], 80)

# 设置等待时间2秒
time.sleep(2)

# 让机械臂到达[-13.7, 40, 20]这个坐标，速度为80mm/s
ua.set_coords([-13.7, 40, 20], 80)

# 设置等待时间2秒
time.sleep(2)

# 仅改变头部的x坐标，设置头部的x坐标为-40，速度为70mm/s
ua.set_coord(1, -40, 70)