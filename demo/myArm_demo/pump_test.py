from pymycobot.myarm import MyArm
import time
import RPi.GPIO as GPIO

# 初始化一个MyCobot对象
mc = MyArm('/dev/ttyAMA0')

# 初始化
GPIO.setmode(GPIO.BCM)
# 引脚20/21分别控制电磁阀和泄气阀门
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)


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


pump_on()
time.sleep(6)
pump_off()
time.sleep(3)
