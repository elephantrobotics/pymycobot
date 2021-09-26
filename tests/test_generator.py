import os
import sys
import pytest
from pprint import pprint

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot import MyCobotCommandGenerator


@pytest.fixture(scope="module")
def mcg():
    print("")
    DEBUG = False
    mg = MyCobotCommandGenerator(debug=DEBUG)
    print("")
    return mg

def test_flatten1(mcg):
    assert mcg._flatten([1,2,3,4,5,6]) == [1,2,3,4,5,6]

def test_flatten2(mcg):
    assert mcg._flatten([1,2,3,4,[],5]) == [1,2,3,4,5]

def test_flatten3(mcg):
    assert mcg._flatten([1,2,3,4,[5,6],7]) == [1,2,3,4,5,6,7]

def test_process_data_command(mcg):
    assert mcg._process_data_command(()) == []

def test_version(mcg):
    assert mcg.version() == ([0xFE, 0xFE, 2, 0x00, 0xFA], True)

def test_power_on(mcg):
    assert mcg.power_on() == ([0xFE, 0xFE, 2, 0x10, 0xFA], False)

def test_power_off(mcg):
    assert mcg.power_off() == ([0xFE, 0xFE, 2, 0x11, 0xFA], False)

def test_release_all_servos(mcg):
    assert mcg.release_all_servos() == ([0xFE, 0xFE, 2, 0x13, 0xFA], False)

def test_is_controller_connected(mcg):
    assert mcg.is_controller_connected() == ([0xFE, 0xFE, 2, 0x14, 0xFA], True)

def test_get_angles(mcg):
    assert mcg.get_angles() == ([0xFE, 0xFE, 2, 0x20, 0xFA], True)

def test_get_coords(mcg):
    assert mcg.get_coords() == ([0xFE, 0xFE, 2, 0x23, 0xFA], True)

def test_pause(mcg):
    assert mcg.pause() == ([0xFE, 0xFE, 2, 0x26, 0xFA], False)

def test_resume(mcg):
    assert mcg.resume() == ([0xFE, 0xFE, 2, 0x28, 0xFA], False)

def test_stop(mcg):
    assert mcg.stop() == ([0xFE, 0xFE, 2, 0x29, 0xFA], False)

def test_generator(mcg):
    print("")
    pprint(mcg.send_coords([-160, 160, 160, 0, 0, 0], 7, 2))
    print("")
    pprint(mcg.send_angle(1, 0, 10))
    print("")
    pprint(mcg.send_angles([0, 0, 0, 0, 0, 0], 5))
    print("")
    pprint(mcg.send_coord(1, 110.5, 8))

    print("is_in_position")
    pprint(mcg.is_in_position([0, 0, 0, 0, 0, 0], 0))

    print("")
    pprint(mcg.is_moving())
    print("")
    pprint(mcg.jog_angle(1, 0, 1))
    print("")
    pprint(mcg.jog_coord(2, 1, 3))
    print("")
    pprint(mcg.jog_stop())
    print("set_encoder")
    pprint(mcg.set_encoder(1, 1024))

    print("get_encoder")
    pprint(mcg.get_encoder(2))
    print("set_speed")
    pprint(mcg.set_speed(100))
    print("get_speed")
    pprint(mcg.get_speed())
    print("get_joint_min_angle")
    pprint(mcg.get_joint_min_angle(1))
    print("get_joint_max_angle")
    pprint(mcg.get_joint_max_angle(2))
    print("is_servo_enable")
    pprint(mcg.is_servo_enable(6))
    print("is_all_servo_enable")
    pprint(mcg.is_all_servo_enable())
    print("set_servo_data")
    pprint(mcg.set_servo_data(0, 1, 2))
    print("get_servo_data")
    pprint(mcg.get_servo_data(1, 2))
    print("set_servo_calibration")
    pprint(mcg.set_servo_calibration(1))
    print("release_servo")
    pprint(mcg.release_servo(1))
    print("focus_servo")
    pprint(mcg.focus_servo(1))
    print("set_color")
    pprint(mcg.set_color(255, 255, 0))
    print("set_pin_mode")
    pprint(mcg.set_pin_mode(1, 0))
    print("set_digital_output")
    pprint(mcg.set_digital_output(0, 1))
    pprint(mcg)
