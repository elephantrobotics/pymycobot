from ensurepip import version
import os
import sys
import pytest
from pprint import pprint

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot import MycobotCommandGenerater

mg: MycobotCommandGenerater


@pytest.fixture(scope="module")
def setup():
    global mg
    print("")
    DEBUG = False
    mg = MycobotCommandGenerater(debug=DEBUG)
    print("")


def test_generator(setup):
    pprint(mg.send_coords([-160, 160, 160, 0, 0, 0], 7, 2))
    pprint(mg.version())
    pprint(mg.power_on())
    pprint(mg.power_off())
    pprint(mg.release_all_servos())
    pprint(mg.is_controller_connected())
    pprint(mg.get_angles())
    pprint(mg.send_angle(1, 0, 10))
    pprint(mg.send_angles([0, 0, 0, 0, 0, 0], 5))
    pprint(mg.get_coords())
    pprint(mg.send_coord(1, 110.5, 8))
    pprint(mg.pause())
    pprint(mg.is_paused())
    pprint(mg.resume())
    pprint(mg.stop())
    pprint(mg.is_in_position([0, 0, 0, 0, 0, 0], 0))
    pprint(mg.is_moving())
    pprint(mg.jog_angle(1, 0, 1))
    pprint(mg.jog_coord(2, 1, 3))
    pprint(mg.jog_stop())
    pprint(mg.set_encoder(1, 1024))
    pprint(mg.get_encoder(2))
    pprint(mg.get_speed())
    pprint(mg.set_speed(100))
    pprint(mg.get_joint_min_angle(1))
    pprint(mg.get_joint_max_angle(2))
    pprint(mg.is_servo_enable(6))
    pprint(mg.is_all_servo_enable())
    pprint(mg.set_servo_data(0, 1, 2))
    pprint(mg.get_servo_data(1, 2))
    pprint(mg.set_servo_calibration(1))
    pprint(mg.release_servo(1))
    pprint(mg.focus_servo(1))
    pprint(mg.set_color(255, 255, 0))
    pprint(mg.set_pin_mode(1, 0))
    pprint(mg.set_digital_output(0, 1))
    pprint(mg)
