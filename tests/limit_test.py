import time
from pymycobot.mycobot import MyCobot


reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]
zero = [0,0,0,0,0,0]
sp = 100

def test(mc):
    print('Start check limit api\n')

    print('get joint min angle value:')
    print(mc.get_joint_min_angle(1))
    print(mc.get_joint_min_angle(2))
    print(mc.get_joint_min_angle(3))
    print(mc.get_joint_min_angle(4))
    print(mc.get_joint_min_angle(5))
    print(mc.get_joint_min_angle(6))


    print('get joint max angle value:')
    print(mc.get_joint_max_angle(1))
    print(mc.get_joint_max_angle(2))
    print(mc.get_joint_max_angle(3))
    print(mc.get_joint_max_angle(4))
    print(mc.get_joint_max_angle(5))
    print(mc.get_joint_max_angle(6))


    mc.send_angles(reset, 100)
    time.sleep(5)
    mc.set_free_mode()

    print('=== check end ===\n')


if __name__ == '__main__':
    print('''
--------------------------------------------
| This file will test limit method:        |
|     get_joint_min_angle()                |
|     get_joint_max_angle()                |
--------------------------------------------
          ''')
    time.sleep(3)
    with open('./port.txt') as f:
        port = f.read().strip().replace('\n', '')
        print(port)
    mc = MyCobot(port)

    test(mc)
