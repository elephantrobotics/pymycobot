import os
import sys
import pytest

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot.error import ultraArmP1DataException
from pymycobot.ultraarm_p1_base import UltraArmP1Base
from pymycobot import utils

port = utils.get_port_list()
print(port)

detect_result = utils.detect_port_of_basic()
print(detect_result)


class UltraArmP1(UltraArmP1Base):
    def __init__(self):
        super().__init__(debug=False)
        self.sent_commands = []
        self.recv_buffer = bytearray()

    def _send_command(self, command, clear_input=True):
        self.sent_commands.append(command)
        if command in ("M6", "M7"):
            self.recv_buffer.extend(f"{command} OK\n".encode())
        elif command == "M8":
            self.recv_buffer.extend(
                b"M8 acc:2000.000,400.000,400.000,400.000\n"
            )
        elif command == "M110 J1":
            self.recv_buffer.extend(b"M110 J1 Data:0.00\n")
        elif command == "M110 J4":
            self.recv_buffer.extend(b"M110 Data:0.00,0.00\n")
        elif command.startswith("M9"):
            self.recv_buffer.extend(b"M9 OK\n")

    def _read_available_bytes(self):
        data = bytes(self.recv_buffer)
        self.recv_buffer.clear()
        return data

    def _clear_input_buffer(self):
        pass

    def _clear_serial_buffer(self):
        pass

    def _send_raw_command(self, command):
        self.sent_commands.append(command)


def test_ultraarm_p1_move_pause_sends_m6_and_returns_ok():
    arm = UltraArmP1()

    assert arm.move_pause() == "ok"
    assert arm.sent_commands == ["M6"]


def test_ultraarm_p1_move_resume_sends_m7_and_returns_ok():
    arm = UltraArmP1()

    assert arm.move_resume() == "ok"
    assert arm.sent_commands == ["M7"]


def test_ultraarm_p1_get_joint_acc_parses_all_joint_accelerations():
    arm = UltraArmP1()

    assert arm.get_joint_acc() == [2000.0, 400.0, 400.0, 400.0]
    assert arm.sent_commands == ["M8"]


def test_ultraarm_p1_set_joint_acc_sends_m9_with_joint_and_acc():
    arm = UltraArmP1()

    assert arm.set_joint_acc(1, 100) == "ok"
    assert arm.sent_commands == ["M9 J1 F100"]


def test_ultraarm_p1_get_default_sensor_initialize_parses_float_data():
    arm = UltraArmP1()

    assert arm.get_default_sensor_initialize(1) == 0.0
    assert arm.sent_commands == ["M110 J1"]


def test_ultraarm_p1_get_default_sensor_initialize_parses_float_list():
    arm = UltraArmP1()

    assert arm.get_default_sensor_initialize(4) == [0.0, 0.0]
    assert arm.sent_commands == ["M110 J4"]


@pytest.mark.parametrize("joint_id", [0, 5])
def test_ultraarm_p1_set_joint_acc_rejects_invalid_joint_id(joint_id):
    arm = UltraArmP1()

    with pytest.raises(ultraArmP1DataException):
        arm.set_joint_acc(joint_id, 100)


@pytest.mark.parametrize("acc", [0, 601, "100"])
def test_ultraarm_p1_set_joint_acc_rejects_invalid_acc(acc):
    arm = UltraArmP1()

    with pytest.raises(ultraArmP1DataException):
        arm.set_joint_acc(1, acc)
