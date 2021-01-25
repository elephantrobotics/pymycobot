import time, random, subprocess
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord

def test(mycobot):
    print('Start check api\n')
    # print()

    print('all servo status: {}'.format(mycobot.is_all_servo_enable()))
    time.sleep(1)
    servo = 2
    print('servo {} status: {}'.format(servo, mycobot.is_servo_enable(servo)))
    time.sleep(1)
    print('power status: {}'.format(mycobot.is_power_on()))
    time.sleep(1)

    color_name = ['red', 'green', 'blue']
    color_code = ['ff0000', '00ff00', '0000ff']
    print('::ser_color()')
    i = random.randint(0, len(color_code) - 1)
    mycobot.set_led_color(color_code[i])
    print('==>set color {}\n'.format(color_name[i]))
    time.sleep(3)

    print('::send_angles()')
    angles = [165,-20,-40,0,0,0]
    mycobot.send_angles(angles, 80)
    print('==> set angles {}, speed 80\n'.format(angles))
    time.sleep(5)
    print('is in position: {}'.format(mycobot.is_in_position(angles, 0)))

    print('::get_angles()')
    print('==> degrees: {}\n'.format(mycobot.get_angles()))
    time.sleep(5)

    print('::send_angle()')
    mycobot.send_angle(Angle.J2.value, 60, 50)
    print('==> angle: joint2, degree: 10, speed: 50\n')
    time.sleep(2)

    coords = mycobot.get_radians()
    time.sleep(1)
    print(coords)
    mycobot.send_radians(coords, 50)
    time.sleep(1)


    print(':: send power off')
    mycobot.power_off()
    print('power status: {}'.format(mycobot.is_power_on()))

    print('::send_radians (should not move)')
    radians = [1,1,1,1,1,1]
    mycobot.send_radians(radians, 70)
    print('==> set radians {}, speed 70\n'.format(radians))
    time.sleep(5)

    print(':: send power on')
    mycobot.power_on()

    print('::send_radians')
    radians = [1,1,1,1,1,1]
    mycobot.send_radians(radians, 70)
    print('movement status: {}'.format(mycobot.is_moving()))
    print('==> set radians {}, speed 70\n'.format(radians))
    time.sleep(5)
    print('movement status: {}'.format(mycobot.is_moving()))

    print('::get_radians()')
    print('==> radians: {}\n'.format(mycobot.get_radians()))
    time.sleep(5)

    print('::get_coords()')
    print('==> coords {}\n'.format(mycobot.get_coords()))
    time.sleep(0.5)

    print('::send_coords()')
    coords = [160, 160, 160, 0, 0, 0]
    mycobot.send_coords(coords, 70, 0)
    print('==> send coords {}, speed 70, mode 0\n'.format(coords))
    time.sleep(5)
    print('is in position: {}'.format(mycobot.is_in_position(coords, 1)))

    print('::send_coord()')
    mycobot.send_coord(Coord.X.value, -40, 70)
    print('==> send coord id: X, coord value: -40, speed: 70\n')
    time.sleep(2)

    print(':: get speed value: {}\n'.format(mycobot.get_speed()))
    time.sleep(1)

    print(':: start jog control test ... ')
    mycobot.jog_angle(1, 1, 50)
    time.sleep(3)
    mycobot.jog_stop()

    mycobot.jog_coord(3, 0, 50)
    time.sleep(3)
    mycobot.jog_stop()
    print('test jog finish')

    print('::set_free_mode()')
    mycobot.set_free_mode()
    print('==> into free moving mode.')
    print('=== check end <==\n')

    print(mycobot.get_joint_min_angle(1))
    print(mycobot.get_joint_max_angle(1))


if __name__ == '__main__':
    port = subprocess.check_output(['echo -n /dev/ttyUSB*'], 
                                    shell=True).decode()
    mycobot = MyCobot(port)
    test(mycobot)

    # while True:
        # print("get_coords():",mycobot.get_coords())
        # print("get_angles():",mycobot.get_angles())
        # print("get_angles_of_radian():",mycobot.get_angles_of_radian())
        # time.sleep(1)

