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

plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

# 初始化一个ultraArm对象
# 下面为 windows版本创建对象代码
ua = ultraArm(plist[0], 115200)
ua.go_zero()

time.sleep(2)

ua.set_pwm(128)