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

def test_encode_int16(mcg):
    assert mcg._encode_int16([0, 1]) == sum([mcg._encode_int16(0), mcg._encode_int16(1)], [])

def test_process_data_command1(mcg):
    assert mcg._process_data_command(()) == []

def test_process_data_command2(mcg):
    assert mcg._process_data_command((1, 2, 3)) == [1, 2, 3]

def test_process_data_command3(mcg):
    assert mcg._process_data_command((1, 2, [1, 2])) == mcg._flatten([1, 2, mcg._encode_int16([1, 2])])

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

def test_is_in_position(mcg):
    assert mcg.is_in_position([0, 0, 0, 0, 0, 0], 0) == (mcg._flatten([0xFE, 0xFE, 15, 0x2A, mcg._flatten([mcg._encode_int16([0, 0, 0, 0, 0, 0]), 0]), 0xFA]), True)

def test_pause(mcg):
    assert mcg.pause() == ([0xFE, 0xFE, 2, 0x26, 0xFA], False)

def test_resume(mcg):
    assert mcg.resume() == ([0xFE, 0xFE, 2, 0x28, 0xFA], False)

def test_stop(mcg):
    assert mcg.stop() == ([0xFE, 0xFE, 2, 0x29, 0xFA], False)

def test_set_pin_mode(mcg):
    assert mcg.set_pin_mode(1, 0) == ([0xFE, 0xFE, 4, 0x60, 1, 0, 0xFA], False)

def test_set_digital_output(mcg):
    assert mcg.set_digital_output(0, 1) == ([0xFE, 0xFE, 4, 0x61, 0, 1, 0xFA], False)
