import time
import os
from pymycobot.mycobot import MyCobot


if __name__ == "__main__":
    with open(os.path.dirname(__file__) + "/port.txt") as f:
        port = f.read().strip().replace("\n", "")
        print(port)
    cobot = MyCobot(port)
    cobot.send_angles([0, 0, 0, 0, 0, 0], 100)
    time.sleep(10)
    print("start")
    for count in range(50):
        time.sleep(0.05)
        cobot.send_angle(1, (-30), 100)
        time.sleep(0.05)
        cobot.send_angle(2, 70, 100)
        time.sleep(0.05)
        cobot.send_angle(3, (-110), 100)
        time.sleep(0.05)
        cobot.send_angle(4, (-100), 100)
        time.sleep(0.05)
        cobot.send_angle(5, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(6, 0, 100)
        time.sleep(1)
        cobot.send_angle(2, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(6, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(5, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(4, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(3, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(1, 0, 100)
        time.sleep(1)
        cobot.send_angle(1, 60, 100)
        time.sleep(0.05)
        cobot.send_angle(2, 70, 100)
        time.sleep(0.05)
        cobot.send_angle(3, (-100), 100)
        time.sleep(0.05)
        cobot.send_angle(4, (-100), 100)
        time.sleep(0.05)
        cobot.send_angle(5, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(6, 0, 100)
        time.sleep(1)
        cobot.send_angle(2, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(6, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(5, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(4, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(3, 0, 100)
        time.sleep(0.05)
        cobot.send_angle(1, 0, 100)
        time.sleep(1)
        print("time")
