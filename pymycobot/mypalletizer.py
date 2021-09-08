# coding=utf-8

import logging

from .log import setup_logging
from .generate import MycobotCommandGenerater
from .common import ProtocolCode, read, write


def calibration_parameters(**kwargs):
    pass


class MyPalletizer(MycobotCommandGenerater):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyPalletizer, self).__init__(debug)
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)

        self.calibration_parameters = calibration_parameters

        import serial

        self._serial_port = serial.Serial(port, baudrate, timeout=timeout)

    _write = write
    _read = read

    def _mesg(self, genre, *args, **kwargs):
        """

        Args:
            genre: command type (Command)
            *args: other data.
                   It is converted to octal by default.
                   If the data needs to be encapsulated into hexadecimal,
                   the array is used to include them. (Data cannot be nested)
            **kwargs: support `has_reply`
                has_reply: Whether there is a return value to accept.
        """
        real_command, has_reply = super(MyPalletizer, self)._mesg(
            genre, *args, **kwargs
        )
        self._write(self._flatten(real_command))

        if has_reply:
            data = self._read()
            res = self._process_received(data, genre)
            if genre in [
                ProtocolCode.IS_POWER_ON,
                ProtocolCode.IS_CONTROLLER_CONNECTED,
                ProtocolCode.IS_PAUSED,
                ProtocolCode.IS_IN_POSITION,
                ProtocolCode.IS_MOVING,
                ProtocolCode.IS_SERVO_ENABLE,
                ProtocolCode.IS_ALL_SERVO_ENABLE,
                ProtocolCode.GET_SERVO_DATA,
                ProtocolCode.GET_DIGITAL_INPUT,
                ProtocolCode.GET_GRIPPER_VALUE,
                ProtocolCode.IS_GRIPPER_MOVING,
                ProtocolCode.GET_SPEED,
                ProtocolCode.GET_ENCODER,
                ProtocolCode.GET_BASIC_INPUT,
            ]:
                return self._process_single(res)
            elif genre in [ProtocolCode.GET_ANGLES]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_COORDS]:
                if res:
                    r = []
                    for idx in range(3):
                        r.append(self._int2coord(res[idx]))
                    r.append(self._int2angle(res[3]))
                    return r
                else:
                    return res
            elif genre in [
                ProtocolCode.GET_JOINT_MIN_ANGLE,
                ProtocolCode.GET_JOINT_MAX_ANGLE,
            ]:
                return self._int2angle(res[0]) if res else 0
            else:
                return res
        return None
