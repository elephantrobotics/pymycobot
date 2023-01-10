from pymycobot.ultraArm import ultraArm
import time
import serial
import serial.tools.list_ports
#输入以上代码导入工程所需要的包

# ultraArm 类初始化需要两个参数：
#   第一个是串口字符串， 如：
#       linux： "/dev/ttyUSB0"
#       windows: "COM3"
#   第二个是波特率: 115200
#
#   如:
#         linux:
#              ua = ultraArm("/dev/USB0", 115200)
#           windows:
#              ua = ultraArm("COM3", 115200)
#

plist = [
        str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
    ]

# 初始化一个ultraArm对象
# 下面为 windows版本创建对象代码
ua = ultraArm(plist[0], 115200)

# ultraArm进行坐标运动/角度运动之前必须进行回零，否则无法获取到正确的角度/坐标
ua.go_zero()
time.sleep(0.5)

# 开启吸泵    
ua.set_gpio_state(0)

#等待2 秒
time.sleep(3)


# 关闭吸泵
ua.set_gpio_state(1)

time.sleep(2)