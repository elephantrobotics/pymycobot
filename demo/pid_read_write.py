from pymycobot import MyCobot
# from pymycobot import PI_PORT
import time
import serial

# pi
# 280
# mc = MyCobot(PI_PORT, 1000000)
# mc = MyCobot('/dev/ttyAMA0', 1000000)
# 320
# mc = MyCobot(PI_PORT, 115200)
# mc = MyCobot('/dev/ttyAMA0', 115200)

# M5
# 280/320
mc = MyCobot('COM66', 115200)
# mc = MyCobot('/dev/ttyUSB0',115200)

# 参数对应地址
data_id = [7, 21, 22, 23, 24, 26, 27] 
# 修改后的参数
data    = [0, 32, 8, 0, 0, 3, 3]

def read():
    for i in range(1, 7):
        for j in range(7):
            print("Servo motor " + str(i) + "  data_id  " +str(data_id[j]) + " : " + str(mc.get_servo_data(i, data_id[j])) )
            time.sleep(0.2)

def write():
    for i in range(1,7):
        for j in range(7):
            mc.set_servo_data(i, data_id[j], data[j])
            time.sleep(0.5)

if __name__ == "__main__":
    mc.power_on()
    time.sleep(2)
    # 写入参数（第一次使用先读取再进行修改）
    write()
    # 读取参数
    read()
