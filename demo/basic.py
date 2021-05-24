import time
import os
import sys
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

sys.path.append(os.path.dirname(__file__))
from port_setup import setup

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]


def test(mycobot):
    print("\nStart check basic options\n")

    mycobot.set_color(255, 255, 0)
    print("::set_color() ==> color {}\n".format("255 255 0"))
    time.sleep(3)

    angles = [0, 0, 0, 0, 0, 0]
    mycobot.send_angles(angles, 100)
    print("::send_angles() ==> angles {}, speed 100\n".format(angles))
    time.sleep(3)

    print("::get_angles() ==> degrees: {}\n".format(mycobot.get_angles()))
    time.sleep(1)

    mycobot.send_angle(Angle.J1.value, 90, 50)
    print("::send_angle() ==> angle: joint1, degree: 90, speed: 50\n")
    time.sleep(4)

    radians = [1, 1, 1, 1, 1, 1]
    mycobot.send_radians(radians, 100)
    print("::send_radians() ==> set radians {}, speed 100\n".format(radians))
    time.sleep(3)

    print("::get_radians() ==> radians: {}\n".format(mycobot.get_radians()))
    time.sleep(1)

    coords = [160, 160, 160, 0, 0, 0]
    mycobot.send_coords(coords, 70, 0)
    print("::send_coords() ==> send coords {}, speed 70, mode 0\n".format(coords))
    time.sleep(3)

    print("::get_coords() ==> coords {}\n".format(mycobot.get_coords()))
    time.sleep(0.5)

    mycobot.send_coord(Coord.X.value, -40, 70)
    print("::send_coord() ==> send coord id: X, coord value: -40, speed: 70\n")
    time.sleep(2)

    print("::set_free_mode()\n")
    mycobot.send_angles(reset, 100)
    time.sleep(5)
    mycobot.release_all_servos()

    print("=== check end ===\n")


if __name__ == "__main__":
    print(
        """
--------------------------------------------
| This file will test basic option method: |
|     set_led_color()                      |
|     send_angles()                        |
|     get_angles()                         |
|     send_angle()                         |
|     send_radians()                       |
|     get_radians()                        |
|     send_coords()                        |
|     get_coords()                         |
|     send_coord()                         |
--------------------------------------------
          """
    )
    time.sleep(3)
    # port = subprocess.check_output(['echo -n /dev/ttyUSB*'],
    # shell=True).decode()
    # with open(os.path.dirname(__file__) + "/port.txt") as f:
        # port = f.read().strip().replace("\n", "")
        # print(port)
    mycobot = setup()
    test(mycobot)
