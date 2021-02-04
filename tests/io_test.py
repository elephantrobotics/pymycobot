import time, random, subprocess
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

def io_test(mc):
    print('Start check IO part of api\n')
    # print()

    mc.set_pin_mode(19, 1)
    time.sleep(1)
    mc.set_digital_output(19, 1)
    time.sleep(5)
    mc.set_digital_output(19, 0)


if __name__ == '__main__':
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
    mycobot = MyCobot(port)
    io_test(mycobot)
