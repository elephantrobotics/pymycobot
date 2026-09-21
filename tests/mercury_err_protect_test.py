from unittest.mock import Mock

import pytest

from pymycobot.common import DataProcessor, ProtocolCode
from pymycobot.close_loop import CloseLoop
from pymycobot.error import MercuryDataException, calibration_parameters
from pymycobot.mercury import Mercury
from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.mercurysocket import MercurySocket


@pytest.mark.parametrize("robot_class", [Mercury, MercurySocket])
@pytest.mark.parametrize("status", [0, 1])
def test_set_err_protect_status_sends_status_and_returns_reply(
    robot_class, status
):
    robot = object.__new__(robot_class)
    robot.calibration_parameters = calibration_parameters
    robot._mesg = Mock(return_value=status)

    assert robot.set_err_protect_status(status) == status
    robot._mesg.assert_called_once_with(
        ProtocolCode.SET_ERROR_DETECT_MODE, status
    )


@pytest.mark.parametrize("robot_class", [Mercury, MercurySocket])
def test_get_err_protect_status_returns_reply(robot_class):
    robot = object.__new__(robot_class)
    robot._mesg = Mock(return_value=1)

    assert robot.get_err_protect_status() == 1
    robot._mesg.assert_called_once_with(
        ProtocolCode.GET_ERROR_DETECT_MODE, has_reply=True
    )


@pytest.mark.parametrize("status", [-1, 2])
def test_set_err_protect_status_rejects_invalid_status(status):
    robot = object.__new__(Mercury)
    robot.calibration_parameters = calibration_parameters
    robot._mesg = Mock()

    with pytest.raises(MercuryDataException):
        robot.set_err_protect_status(status)

    robot._mesg.assert_not_called()


def test_error_protect_commands_use_mercury_crc_frames():
    robot = object.__new__(Mercury)

    get_command, has_reply, _ = DataProcessor._mesg(
        robot, ProtocolCode.GET_ERROR_DETECT_MODE, has_reply=True
    )
    set_command, _, _ = DataProcessor._mesg(
        robot, ProtocolCode.SET_ERROR_DETECT_MODE, 1
    )

    assert get_command[:4] == [0xFE, 0xFE, 0x03, 0xE9]
    assert has_reply is True
    assert get_command[-2:] == robot.crc_check(get_command[:-2])
    assert set_command[:5] == [0xFE, 0xFE, 0x04, 0xE8, 0x01]
    assert set_command[-2:] == robot.crc_check(set_command[:-2])


@pytest.mark.parametrize("status", [0, 1])
def test_get_err_protect_status_parses_single_byte_reply(monkeypatch, status):
    robot = object.__new__(Mercury)

    def receive_reply(self, genre, *args, **kwargs):
        assert genre == ProtocolCode.GET_ERROR_DETECT_MODE
        return bytearray([status]), 1

    monkeypatch.setattr(CloseLoop, "_mesg", receive_reply)

    assert MercuryCommandGenerator._mesg(
        robot, ProtocolCode.GET_ERROR_DETECT_MODE, has_reply=True
    ) == status
