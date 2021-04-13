import time
import os
from pymycobot.mycobot import MyCobot


reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]
zero = [0, 0, 0, 0, 0, 0]
sp = 100


def test(mc):
    print('Start check power state api\n')

    mc.power_on()
    time.sleep(.5)
    f = mc.is_power_on()
    print('is power on: {}'.format(f))
    assert f == 1, 'should be 1'
    mc.send_angles(zero, sp)
    time.sleep(3)

    for _ in range(3):
        mc.power_off()
        time.sleep(.5)
        f = mc.is_power_on()
        print('is power on: {}'.format(f))
        assert f == 0, 'should be 0'

        mc.send_angles(reset, sp)
        time.sleep(1)

        mc.power_on()
        time.sleep(.5)
        f = mc.is_power_on()
        print('is power on: {}'.format(f))
        assert f == 1, 'should be 1'

    mc.send_angles(reset, 100)
    time.sleep(5)
    mc.set_free_mode()

    print('=== check end ===\n')


if __name__ == '__main__':
    print('''
--------------------------------------------
| This file will test power state method:  |
|     power_off()                          |
|     power_on()                           |
|     is_power_on()                        |
--------------------------------------------
          ''')
    time.sleep(3)
    with open(os.path.dirname(__file__) + '/port.txt') as f:
        port = f.read().strip().replace('\n', '')
        print(port)
    mc = MyCobot(port)
    test(mc)
