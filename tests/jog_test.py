import time
import os
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]
zero = [0, 0, 0, 0, 0, 0]
sp = 100


def test(mc):
    print('Start check jog api\n')

    mc.send_angles(zero, sp)
    time.sleep(5)

    print('jog_angle() -> control joint1')
    mc.jog_angle(Angle.J1.value, 1, 10)
    time.sleep(3)
    print('pause 10 s')
    mc.pause()
    time.sleep(3)
    print('speed get', mc.get_speed())
    mc.set_speed(20)
    print('speed set', mc.get_speed())
    time.sleep(7)
    print('resume')
    mc.resume()
    time.sleep(10)

    coords = [160, 140, 160, 0, 0, 0]
    mc.send_coords(coords, 100, 0)
    time.sleep(4)

    mc.jog_coord(Coord.Z.value, 1, 10)
    time.sleep(3)
    print('stop')

    print('=== check end ===\n')


if __name__ == '__main__':
    print('''
--------------------------------------------
| This file will test jog method:          |
|     jog_angle()                          |
|     jog_coord()                          |
|     pause()                              |
|     resume()                             |
|     stop()                               |
|     get_speed()                          |
|     set_speed()                          |
--------------------------------------------
          ''')
    time.sleep(3)
    with open(os.path.dirname(__file__) + '/port.txt') as f:
        port = f.read().strip().replace('\n', '')
        print(port)
    mc = MyCobot(port)

    test(mc)
