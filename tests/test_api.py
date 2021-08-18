import os
import sys
import time
import pytest

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot import MyCobot, Angle, Coord
from pymycobot import utils

# from pymycobot.genre import Angle, Coord

port: str
mc: MyCobot
sp: int = 80


@pytest.fixture(scope="module")
def setup():
    print("")
    global port, mc

    detected = utils.detect_port_of_basic()
    if not detected:
        plist = utils.get_port_list()
        idx = 1
        for port in plist:
            print("{} : {}".format(idx, port))
            idx += 1

        _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
        port = plist[int(_in) - 1]
    else:
        port = detected
    print(port)
    print("")

    baud = input("\nPlease input baudrate (default: 115200):")
    if not baud:
        baud = 115200
    print(baud)
    print("")

    DEBUG = False
    f = input("Wether DEBUG mode[Y/n] (default: no):")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    mc = MyCobot(port, baud, debug=DEBUG)
    print("")


def test_basic_api(setup):
    print("==========================================================")
    print("start basic test...")
    mc.set_color(0, 255, 0)
    time.sleep(0.1)

    zero = [0, 0, 0, 0, 0, 0]
    mc.send_angles(zero, sp)
    time.sleep(2)
    angles = mc.get_angles()
    print(angles)
    print("check angle ", end="")
    for angle in angles:
        assert -2 < angle < 2, "The error is large."

    for _ in range(10):
        print(mc.is_in_position(zero, 0), " ", end="")
        time.sleep(0.5)
    print("")

    mc.send_angle(Angle.J1.value, 90, 50)
    time.sleep(4)

    radians = [1, 1, 1, 1, 1, 1]
    mc.send_radians(radians, sp)
    time.sleep(3)

    radians = mc.get_radians()
    print(radians)
    time.sleep(0.1)
    print("check radian")
    for radian in radians:
        assert 0.9 <= radian <= 1.1, "The error is large."

    coords = [160, 160, 160, 0, 0, 0]
    print("check coords before sended ", end="")
    for _ in range(3):
        print(mc.is_in_position(coords, 1), " ", end="")
        time.sleep(0.5)
    print("")

    mc.send_coords(coords, sp, 2)
    time.sleep(3)

    get_coords = mc.get_coords()
    print(get_coords)
    time.sleep(0.5)
    print("check coords ", end="")
    for (old, new) in zip(coords, get_coords):
        assert old - 5 <= new <= old + 5

    for _ in range(10):
        print(mc.is_in_position(coords, 1), " ", end="")
        time.sleep(0.5)
    print("")

    mc.send_coord(Coord.X.value, -40, 70)
    time.sleep(2)


def test_jog(setup):
    print("\n==========================================================")
    print("start jog test...")
    zero = [0, 0, 0, 0, 0, 0]
    mc.send_angles(zero, sp)
    time.sleep(5)

    print("jog_angle() -> control joint1")
    mc.jog_angle(Angle.J1.value, 1, 10)
    time.sleep(1)
    print("wait 1 s, check is paused: %s" % mc.is_paused())
    time.sleep(2)
    print("pause 10 s")
    mc.pause()

    time.sleep(3)
    print("wait 3 s, check is paused: %s" % mc.is_paused())
    print("speed get", mc.get_speed())
    mc.set_speed(20)
    print("speed set", mc.get_speed())
    assert mc.get_speed() == 20
    time.sleep(6)

    print("resume")
    mc.resume()  # FIXME:
    time.sleep(10)

    coords = [160, 140, 160, 0, 0, 0]
    mc.send_coords(coords, 100, 0)
    time.sleep(4)

    mc.jog_coord(Coord.Z.value, 1, 10)
    time.sleep(3)
    print("stop")
    mc.stop()
    time.sleep(1)


def test_state_control(setup):
    # reset = [153.19, 137.81, -153.54, 156.79, 87.27, 13.62]
    zero = [0, 0, 0, 0, 0, 0]
    print("==========================================================")
    print("Start state test...")

    print("goto zero position")
    mc.send_angles(zero, sp)
    time.sleep(4)

    print("is moving: 1 - true, 0 - false")
    print(mc.is_moving())  # FIXME:
    time.sleep(1)
    mc.jog_angle(1, 1, 10)
    time.sleep(1)
    print("is moving: 1 - true, 0 - false")
    print(mc.is_moving())
    time.sleep(3)
    mc.stop()


def test_power_state(setup):
    print("==========================================================")
    print("start power test...")
    current = mc.is_power_on()
    if current == 1:
        mc.power_off()
        time.sleep(0.1)
        f = mc.is_power_on()
        print(f)
        assert f == 0

        mc.power_on()

    elif current == 0:
        mc.power_on()
        time.sleep(0.1)
        f = mc.is_power_on()
        print(f)
        assert f == 1


def test_angle_limit(setup):
    print("==========================================================")
    print("get joint min angle value:")
    print(mc.get_joint_min_angle(1))
    print(mc.get_joint_min_angle(2))
    print(mc.get_joint_min_angle(3))
    print(mc.get_joint_min_angle(4))
    print(mc.get_joint_min_angle(5))
    print(mc.get_joint_min_angle(6))

    print("get joint max angle value:")
    print(mc.get_joint_max_angle(1))
    print(mc.get_joint_max_angle(2))
    print(mc.get_joint_max_angle(3))
    print(mc.get_joint_max_angle(4))
    print(mc.get_joint_max_angle(5))
    print(mc.get_joint_max_angle(6))


def test_servo(setup):
    print("==========================================================")
    print("Start servo test...")
    time.sleep(2)

    for _ in range(3):
        print("Is all servos enable: %s" % mc.is_all_servo_enable())
        time.sleep(0.1)
    for i in range(1, 7):
        print("Servo %s is enable %s" % (i, mc.is_servo_enable(i)))
        time.sleep(0.1)

    mc.release_all_servos()
    print("Release all servos and wait 10s.")
    time.sleep(10)

    for i in range(1, 7):
        print("Focused servo %s" % i)
        mc.focus_servo(i)
        time.sleep(1)


def test_gripper(setup):
    print("==========================================================")
    print("start gripper test...")
    # Set the current position to (2048).
    # Use it when you are sure you need it.
    # Gripper has been initialized for a long time. Generally, there
    # is no need to change the method.
    # mc.set_gripper_ini()

    time.sleep(2)
    flag = mc.is_gripper_moving()
    assert flag == 0

    mc.set_gripper_value(0, 50)
    time.sleep(2)

    mc.set_gripper_value(100, 50)
    time.sleep(2)

    mc.set_gripper_state(1, 70)
    time.sleep(2)

    mc.set_gripper_state(0, 70)
    time.sleep(2)

    print(mc.get_gripper_value())


def test_io(setup):
    print("==========================================================")
    print("Start IO test...")

    mc.set_pin_mode(19, 1)
    time.sleep(1)
    mc.set_digital_output(19, 1)
    time.sleep(5)
    mc.set_digital_output(19, 0)


def test_pump(setup):
    print("==========================================================")
    print("Start Pump test...")

    def pump_on():
        mc.set_basic_output(2, 0)
        mc.set_basic_output(5, 0)

    def pump_off():
        mc.set_basic_output(2, 1)
        mc.set_basic_output(5, 1)

    pump_off()
    time.sleep(3)
    pump_on()
    time.sleep(3)
    pump_off()
    time.sleep(3)


def test_encoder(setup):
    print(mc.get_encoders())
    time.sleep(1)
    for i in range(1, 7):
        print(i, mc.get_encoder(i))


def test_error_data(setup):
    print("==========================================================")
    print("Start data check test...\n")
    try:
        mc.send_angle(7, 0, 0)
    except Exception as e:
        print(e)

    try:
        mc.send_angle(1, 191, 0)
    except Exception as e:
        print(e)

    try:
        mc.send_angle(1, 180, -10)
    except Exception as e:
        print(e)

    try:
        mc.send_angles([], 1)
    except Exception as e:
        print(e)

    try:
        mc.send_angles([1, 2, 3, 4, 5, 6, 7], 1)
    except Exception as e:
        print(e)

    try:
        mc.send_angles([1, 2, 3, 4, 5, 6], 101)
    except Exception as e:
        print(e)

    try:
        mc.send_angles([-191, 2, 3, 4, 5, 6], -10)
    except Exception as e:
        print(e)

    try:
        mc.set_color(10, -1, 0)
    except Exception as e:
        print(e)


if __name__ == "__main__":
    setup()
    test_basic_api()
