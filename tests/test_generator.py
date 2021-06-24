from ensurepip import version
import os
import sys
import pytest

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot import MycobotCommandGenerater

mg: MycobotCommandGenerater


@pytest.fixture(scope="module")
def setup():
    global mg
    print("")
    DEBUG = False
    f = input("Wether DEBUG mode[Y/n] (default: no):")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    mg = MycobotCommandGenerater(debug=DEBUG)
    print("")


def test_generator(setup):
    # print(mg.__dir__())
    print(mg.send_coords([-160, 160, 160, 0, 0, 0], 7, 2))
    print(mg.version())
    print(mg.power_on())
    print(mg.power_off())
    print(mg.release_all_servos())
    print(mg.is_controller_connected())
    print(mg.get_angles())
    print(mg.send_angle(1, 0, 10))
    print(mg.send_angles([0, 0, 0, 0, 0, 0], 5))
    print(mg.get_coords())
    print(mg.send_coord(1, 110.5, 8))
    print(mg.pause())
    print(mg.is_paused())
    print(mg.resume())
    print(mg.stop())
    print(mg.is_in_position([0, 0, 0, 0, 0, 0], 0))
    print(mg.is_moving())
    print(mg.jog_angle(1, 0, 1))
    print(mg.jog_coord(2, 1, 3))
    print(mg.jog_stop())
    print(mg.set_encoder(1, 1024))
    print(mg.get_encoder(2))
    print(mg.get_speed())
    print(mg.set_speed(100))
    print(mg.get_joint_min_angle(1))
    print(mg.get_joint_max_angle(2))
    print(mg.is_servo_enable(6))
    print(mg.is_all_servo_enable())
    print(mg.set_servo_data(0, 1, 2))
    print(mg.get_servo_data(1, 2))
    print(mg.set_servo_calibration(1))
    print(mg.release_servo(1))
    print(mg.focus_servo(1))
    print(mg.set_color(255, 255, 0))
    print(mg.set_pin_mode(1, 0))
    print(mg.set_digital_output(0, 1))
    print(mg)
