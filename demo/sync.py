import time
import os
import sys
from pymycobot.mycobot import MyCobot

sys.path.append(os.path.dirname(__file__))
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]


def test(mycobot):
    print("\nStart check basic options\n")

    """
    mycobot.sync_send_angles([0,0,0,0,0,0], 100)
    mycobot.sync_send_angles([100,50,0,0,70,160], 100)
    mycobot.sync_send_angles([-100,-50,0,0,-70,-160], 100)
    """
    mycobot.sync_send_angles([0, 0, 0, 0, 0, 0], 100).sync_send_angles(
        [100, 50, 0, 0, 70, 160], 100
    ).sync_send_angles([-100, -50, 0, 0, -70, -160], 100).sync_send_angles(
        reset, 100
    ).set_free_mode()

    print("=== check end ===\n")


if __name__ == "__main__":
    print(
        """
--------------------------------------------
| This file will test basic option method: |
|     sync_send_angles()                   |
--------------------------------------------
          """
    )
    time.sleep(3)
    mycobot = setup()
    test(mycobot)
    # print(mycobot.get_angles())
