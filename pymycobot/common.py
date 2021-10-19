# coding=utf-8

from __future__ import division
import time
import struct


class ProtocolCode(object):
    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # System status
    VERSION = 0x00

    # Overall status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    IS_CONTROLLER_CONNECTED = 0x14
    READ_NEXT_ERROR = 0x15
    SET_FREE_MODE = 0x1A
    IS_FREE_MODE = 0x1B

    # MDI MODE AND OPERATION
    GET_ANGLES = 0x20
    SEND_ANGLE = 0x21
    SEND_ANGLES = 0x22
    GET_COORDS = 0x23
    SEND_COORD = 0x24
    SEND_COORDS = 0x25
    PAUSE = 0x26
    IS_PAUSED = 0x27
    RESUME = 0x28
    STOP = 0x29
    IS_IN_POSITION = 0x2A
    IS_MOVING = 0x2B

    # JOG MODE AND OPERATION
    JOG_ANGLE = 0x30
    JOG_COORD = 0x32
    JOG_STOP = 0x34
    SET_ENCODER = 0x3A
    GET_ENCODER = 0x3B
    SET_ENCODERS = 0x3C
    GET_ENCODERS = 0x3D

    # RUNNING STATUS AND SETTINGS
    GET_SPEED = 0x40
    SET_SPEED = 0x41
    GET_FEED_OVERRIDE = 0x42
    GET_ACCELERATION = 0x44
    GET_JOINT_MIN_ANGLE = 0x4A
    GET_JOINT_MAX_ANGLE = 0x4B

    # SERVO CONTROL
    IS_SERVO_ENABLE = 0x50
    IS_ALL_SERVO_ENABLE = 0x51
    SET_SERVO_DATA = 0x52
    GET_SERVO_DATA = 0x53
    SET_SERVO_CALIBRATION = 0x54
    RELEASE_SERVO = 0x56
    FOCUS_SERVO = 0x57

    # ATOM IO
    SET_PIN_MODE = 0x60
    SET_DIGITAL_OUTPUT = 0x61
    GET_DIGITAL_INPUT = 0x62
    SET_PWM_MODE = 0x63
    SET_PWM_OUTPUT = 0x64
    GET_GRIPPER_VALUE = 0x65
    SET_GRIPPER_STATE = 0x66
    SET_GRIPPER_VALUE = 0x67
    SET_GRIPPER_INI = 0x68
    IS_GRIPPER_MOVING = 0x69
    SET_COLOR = 0x6A

    # Basic
    SET_BASIC_OUTPUT = 0xA0
    GET_BASIC_INPUT = 0xA1


class DataProcessor(object):
    # Functional approach
    def _encode_int8(self, data):
        return struct.pack("b", data)

    def _encode_int16(self, data):
        if isinstance(data, int):
            return list(struct.pack(">h", data))
        else:
            return sum([list(struct.pack('>h', elem)) for elem in data], [])

    def _decode_int8(self, data):
        return struct.unpack("b", data)[0]

    def _decode_int16(self, data):
        return struct.unpack(">h", data)[0]

    def _angle2int(self, angle):
        return int(angle * 100)

    def _coord2int(self, coord):
        return int(coord * 10)

    def _int2angle(self, _int):
        return round(_int / 100.0, 3)

    def _int2coord(self, _int):
        return round(_int / 10.0, 2)

    def _flatten(self, _list):
        return sum(
            ([x] if not isinstance(x, list) else self._flatten(x)
             for x in _list), []
        )

    def _process_data_command(self, args):
        if not args:
            return []

        return self._flatten(
            [
                [self._encode_int16(int(i))
                 for i in x] if isinstance(x, list) else x
                for x in args
            ]
        )

    def _is_frame_header(self, data, pos):
        return data[pos] == ProtocolCode.HEADER and data[pos + 1] == ProtocolCode.HEADER

    def _process_received(self, data, genre):
        if not data:
            return []

        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        for idx in range(data_len - 1):
            if self._is_frame_header(data, idx):
                data_len = data[idx + 2] - 2
                if data_len > 0:
                    break
        else:
            return []

        # compare send header and received header
        cmd_id = data[idx + 3]
        if cmd_id != genre:
            return []
        data_pos = idx + 4
        valid_data = data[data_pos: data_pos + data_len]

        # process valid data
        res = []
        if data_len == 12 or data_len == 8:
            for idx in range(0, len(valid_data), 2):
                one = valid_data[idx: idx + 2]
                res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            res.append(self._decode_int16(valid_data))
        else:
            res.append(self._decode_int8(valid_data))
        return res

    def _process_single(self, data):
        return data[0] if data else -1


def write(self, command):
    self.log.debug("_write: {}".format(command))

    self._serial_port.write(command)
    self._serial_port.flush()
    time.sleep(0.05)


def read(self):
    if self._serial_port.inWaiting() > 0:
        data = self._serial_port.read(self._serial_port.inWaiting())
        self.log.debug("_read: {}".format(data))
    else:
        self.log.debug("_read: no data can be read")
        data = None
    return data
