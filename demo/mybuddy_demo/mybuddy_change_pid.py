#import library
import time
import os
import sys
import serial
import serial.tools.list_ports
import platform

sys.path.append(os.getcwd())
from pymycobot.mybuddy import MyBuddy

#define type
port: str
mc: MyBuddy
DEBUG = False

# Connect Robot

data_id = [21, 22, 23, 24, 26, 27] 
data    = [25, 25, 1, 0, 3, 3]
mb : MyBuddy
ports = []

def setup():                                                    #机械臂检测函数，选择正确的串口
    global mb
    # auto select system port
    if platform.system() == 'Windows':
        port = 'COM21'
        baud = 115200
    elif platform.system() == 'Linux':
        port = '/dev/ttyACM0'
        baud = 115200
    mb = MyBuddy(port, baud, debug=DEBUG)
    time.sleep(1)

def change():
    global mb, data, data_id
    _mybuddy = mb
    print(_mybuddy)
    for m in range(1,3):
        for i in range(1,7):
            for j in range(len(data_id)):
                _mybuddy.set_servo_data(m, i, data_id[j], data[j])
                time.sleep(0.1)
                _data = _mybuddy.get_servo_data(m, i, data_id[j])
                time.sleep(0.1)
                if _data == data[j]:
                    print("Servo motor :" + str(i) + "  data_id : " + str(data_id[j]) + "   data: " + str(_data) + "  modify successfully ")
                else:
                    print("Servo motor :"  + str(i) + "  data_id : " + str(data_id[j]) + "   data: " + str(_data) + "  modify error ")

if __name__ == "__main__":                                      #主函数
    setup()
    try:
        change()                            
    except Exception as e:
        print(e)

    



