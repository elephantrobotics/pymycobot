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
        time.sleep(0.1)
        print(coords)


"""
[-278.1, -16.8, 104.2, 79.82, -81.6, -169.53]
[-279.2, -15.2, 117.5, 34.72, -85.83, -124.83]
[171.6, -113.1, 229.8, 114.15, -84.21, -24.27]
[-211.5, -113.8, 188.5, -89.79, 6.14, -177.87]
[190.8, -113.1, 210.7, 104.26, -82.19, -14.39]
"""

if __name__ == "__main__":
    time.sleep(1)
    with open(os.path.dirname(__file__) + "/port.txt") as f:
        port = f.read().strip().replace("\n", "")
        print(port)
    mc = MyCobot(port, boudrate="1000000")
    # test(mc)
    # coords = [-279.2, -15.2, 117.5, 34.72, -85.83, -124.83]
    # mc.send_coords(coords, 80, 1)
    # exit(0)
    while True:
        print(mc.get_coords())
        time.sleep(1)
