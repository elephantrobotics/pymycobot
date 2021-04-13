import time
import os
from pymycobot.mycobot import MyCobot


def test(mc):
    special_angles = [
        [0, 90, 0, 0, 90, 0],
        [0, 0, -90, 0, -90, 0],
        [0, 90, 0, -90, -90, 0],
        [0, -90, 90, -90, -90, 0],
    ]

    for angles in special_angles:
        mc.send_angles(angles, 80)
        time.sleep(4)
        coords = mc.get_coords()
        time.sleep(.1)
        print(coords)


if __name__ == '__main__':
    time.sleep(1)
    with open(os.path.dirname(__file__) + '/port.txt') as f:
        port = f.read().strip().replace('\n', '')
        print(port)
    mc = MyCobot(port)
    test(mc)
