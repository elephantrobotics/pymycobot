import os
import time
from pymycobot.mycobot import MyCobot


def io_test(mc):
    print("Start check IO part of api\n")
    # print()

    mc.set_pin_mode(19, 1)
    time.sleep(1)
    mc.set_digital_output(19, 1)
    time.sleep(5)
    mc.set_digital_output(19, 0)


if __name__ == "__main__":
    with open(os.path.dirname(__file__) + "/port.txt") as f:
        port = f.read().strip().replace("\n", "")
        print(port)
    mycobot = MyCobot(port)
    io_test(mycobot)
