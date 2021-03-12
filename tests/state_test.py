import time
from pymycobot.mycobot import MyCobot


reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]
zero = [0,0,0,0,0,0]
coords = [160, 160, 160, 0, 0, 0]
sp = 100

def test(mc):
    print('Start check state api\n')

    print('goto zero position')
    mc.send_angles(zero, sp)
    time.sleep(4)
    print('is in position(angle): 1 - true, 0 - false, -1 - error data')
    for _ in range(10):
        print(mc.is_in_position(zero, 0), ' ', end='')
        time.sleep(.5)

    print('\nis in position(coords): 1 - true, 0 - false, -1 - error data')
    for _ in range(10):
        print(mc.is_in_position(coords, 1), ' ', end='')
        time.sleep(.5)

    mc.send_coords(coords, 100, 0)
    time.sleep(4)
    print('\nis in position(coords): 1 - true, 0 - false, -1 - error data')
    for _ in range(10):
        print(mc.is_in_position(coords, 1), ' ', end='')
        time.sleep(.5)


    mc.send_angles(zero, 100)
    time.sleep(4)
    print('is moving: 1 - true, 0 - false')
    print(mc.is_moving())
    time.sleep(3)
    print('is moving: 1 - true, 0 - false')
    print(mc.is_moving())
    time.sleep(1)
    mc.jog_angle(1, 1, 35)

    t = time.time()
    print('is moving: 1 - true, 0 - false')
    while time.time() - t < 20:
        print(mc.is_moving())
        time.sleep(.5)



if __name__ == '__main__':
    print('''
--------------------------------------------
| This file will test state method:        |
|     is_in_position()                     |
|     is_moving()                          |
--------------------------------------------
          ''')
    time.sleep(3)
    with open('./port.txt') as f:
        port = f.read().strip().replace('\n', '')
        print(port)
    mc = MyCobot(port)
    test(mc)

