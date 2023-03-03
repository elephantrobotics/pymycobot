# coding=utf-8

from __future__ import division
import time
import struct


class ProtocolCode(object):
    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # System status
    ROBOT_VERSION = 0x01
    SOFTWARE_VERSION = 0x02
    GET_ROBOT_ID = 0x03
    SET_ROBOT_ID = 0x04

    # Overall status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    IS_CONTROLLER_CONNECTED = 0x14
    READ_NEXT_ERROR = 0x15
    SET_FRESH_MODE = 0x16
    GET_FRESH_MODE = 0x17
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
    GET_ANGLE = 0x2C
    GET_COORD = 0x2D
    SEND_ANGLES_AUTO = 0x2E
    GET_SOLUTION_ANGLES = 0x2E
    SET_SOLUTION_ANGLES = 0x2F

    # JOG MODE AND OPERATION
    JOG_ANGLE = 0x30
    JOG_ABSOLUTE = 0x31
    JOG_COORD = 0x32
    JOG_INCREMENT = 0x33
    JOG_STOP = 0x34
    SET_ENCODER = 0x3A
    GET_ENCODER = 0x3B
    SET_ENCODERS = 0x3C
    GET_ENCODERS = 0x3D
    SET_ENCODERS_DRAG = 0x3E

    # RUNNING STATUS AND SETTINGS
    GET_SPEED = 0x40
    SET_SPEED = 0x41
    GET_FEED_OVERRIDE = 0x42
    GET_ACCELERATION = 0x44
    SET_ACCELERATION = 0x45
    GET_JOINT_MIN_ANGLE = 0x4A
    GET_JOINT_MAX_ANGLE = 0x4B
    SET_JOINT_MIN = 0x4C
    SET_JOINT_MAX = 0x4D

    # SERVO CONTROL
    IS_SERVO_ENABLE = 0x50
    IS_ALL_SERVO_ENABLE = 0x51
    SET_SERVO_DATA = 0x52
    GET_SERVO_DATA = 0x53
    SET_SERVO_CALIBRATION = 0x54
    JOINT_BRAKE = 0x55
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
    SET_GRIPPER_CALIBRATION = 0x68
    IS_GRIPPER_MOVING = 0x69
    SET_COLOR = 0x6A
    SET_ELETRIC_GRIPPER = 0x6B
    INIT_ELETRIC_GRIPPER = 0x6C
    SET_GRIPPER_MODE = 0x6D
    GET_GRIPPER_MODE = 0x6E

    GET_ACCEI_DATA = 0x73

    # Basic
    SET_BASIC_OUTPUT = 0xA0
    GET_BASIC_INPUT = 0xA1
    GET_BASE_INPUT = 0xA2
    SET_BASE_PWM = 0xA5

    # Linux GPIO, mode: GPIO.BCM
    SET_GPIO_MODE = 0xAA
    SET_GPIO_UP = 0xAB
    SET_GPIO_OUTPUT = 0xAC
    GET_GPIO_IN = 0xAD

    # set WIFI
    SET_SSID_PWD = 0xB0
    GET_SSID_PWD = 0xB1
    SET_SERVER_PORT = 0xB2

    # Get the measured distance
    GET_TOF_DISTANCE = 0xC0

    # Coordinate transformation
    SET_TOOL_REFERENCE = 0x81
    GET_TOOL_REFERENCE = 0x82
    SET_WORLD_REFERENCE = 0x83
    GET_WORLD_REFERENCE = 0x84
    SET_REFERENCE_FRAME = 0x85
    GET_REFERENCE_FRAME = 0x86
    SET_MOVEMENT_TYPE = 0x87
    GET_MOVEMENT_TYPE = 0x88
    SET_END_TYPE = 0x89
    GET_END_TYPE = 0x8A

    # Impact checking
    SET_JOINT_CURRENT = 0x90
    GET_JOINT_CURRENT = 0x91
    SET_CURRENT_STATE = 0x92

    # planning speed
    GET_PLAN_SPEED = 0xD0
    GET_PLAN_ACCELERATION = 0xD1
    SET_PLAN_SPEED = 0xD2
    SET_PLAN_ACCELERATION = 0xD3

    # Motor status read
    GET_SERVO_SPEED = 0xE1
    GET_SERVO_CURRENTS = 0xE2
    GET_SERVO_VOLTAGES = 0xE3
    GET_SERVO_STATUS = 0xE4
    GET_SERVO_TEMPS = 0xE5
    GET_SERVO_LASTPDI = 0xE6

    GET_BASE_COORDS = 0xF0
    BASE_TO_SINGLE_COORDS = 0xF1
    COLLISION = 0xF2
    GET_BASE_COORD = 0xF3
    GET_ALL_BASE_COORDS = 0xF4
    WRITE_BASE_COORD = 0xF5
    WRITE_BASE_COORDS = 0xF6
    JOG_INC_COORD = 0xF7
    COLLISION_SWITCH = 0xF8
    IS_COLLISION_ON = 0xF9

    # IIC
    # SET_IIC_STATE = 0xA4
    # GET_IIS_BYTE = 0xA5
    # SET_IIC_BYTE = 0xA6

    # ultraArm
    END = "\r"
    COORDS_SET = "G0"
    SLEEP_TIME = "G4"
    BACK_ZERO = "G28"
    ABS_CARTESIAN = "G90"
    REL_CARTESIAN = "G91"
    SET_JOINT = "G92"
    SET_PWM = "M21"
    GPIO_CLOSE = "M23"
    GPIO_ON = "M22"
    GRIPPER_ZERO = "M24"
    GIRPPER_OPEN = "M25"
    GIRPPER_RELEASE = "M26"
    FAN_ON = "M27"
    FAN_CLOSE = "M28"
    SET_ANGLE = "M10"
    SET_ANGLES = "M11"
    GET_CURRENT_ANGLE_INFO = "M12"
    SET_JOG_ANGLE = "M13"
    JOG_COORD_ = "M14"
    SET_JOG_STOP = "M15"
    RELEASE_SERVOS = "M17"
    LOCK_SERVOS = "M18"
    GET_CURRENT_COORD_INFO = "M114"
    GET_BACK_ZERO_STATUS = "M119"


class DataProcessor(object):
    # Functional approach
    def _encode_int8(self, data):
        return struct.pack("b", data)

    def _encode_int16(self, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            return sum([list(struct.pack(">h", elem)) for elem in data], [])

    def _decode_int8(self, data):
        return struct.unpack("B", data)[0]

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
            ([x] if not isinstance(x, list) else self._flatten(x) for x in _list), []
        )

    def _process_data_command(self, args):
        if not args:
            return []

        return self._flatten(
            [
                [self._encode_int16(int(i)) for i in x] if isinstance(x, list) else x
                for x in args
            ]
        )

    def _is_frame_header(self, data, pos1, pos2):
        return data[pos1] == ProtocolCode.HEADER and data[pos2] == ProtocolCode.HEADER

    def _process_received(self, data, genre, arm=6):
        if not data:
            return []
        if genre == 177:
            data = str(data)[2:-1].split(": ")
            return data[1][0:-9], data[-1]
        if not data:
            return []
        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        header_i, header_j = 0, 1
        while header_j < data_len - 4:
            if self._is_frame_header(data, header_i, header_j):
                if arm == 6:
                    cmd_id = data[header_i + 3]
                elif arm == 12:
                    cmd_id = data[header_i + 4]
                # compare send header and received header
                if cmd_id == genre:
                    break
            header_i += 1
            header_j += 1
        else:
            return []
        if arm == 6:
            data_len = data[header_i + 2] - 2
        elif arm == 12:
            data_len = data[header_i + 3] - 2
        unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]

        if cmd_id in unique_data:
            data_pos = header_i + 5
            data_len -= 1
        else:
            if arm == 6:
                data_pos = header_i + 4
            elif arm == 12:
                data_pos = header_i + 5

        valid_data = data[data_pos : data_pos + data_len]

        # process valid data
        res = []
        if genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_TEMPS]:
            for i in valid_data:
                res.append(i)
            return res
        if data_len in [6, 8, 12, 14, 24, 60]:
            for header_i in range(0, len(valid_data), 2):
                one = valid_data[header_i : header_i + 2]
                res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [
                ProtocolCode.GET_PLAN_SPEED,
                ProtocolCode.GET_PLAN_ACCELERATION,
            ]:
                return [
                    self._decode_int8(valid_data[0:1]),
                    self._decode_int8(valid_data[1:]),
                ]
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            for i in range(1,4):
                res.append(valid_data[i])
        elif data_len == 7:
            error_list = [i for i in valid_data]
            for i in error_list:
                if i in range(16,23):
                    res.append(1)
                elif i in range(23,29):
                    res.append(2)
                elif i in range(32,112):
                    res.append(3)
                else:
                    res.append(i)
        else:
            if genre in [
                ProtocolCode.GET_SERVO_VOLTAGES,
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.GET_SERVO_TEMPS,
            ]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i : i + 1])
                    res.append(0xFF & data1 if data1 < 0 else data1)
                return res
            res.append(self._decode_int8(valid_data))
        if genre == ProtocolCode.GET_ACCEI_DATA:
            for i in range(len(res)):
                res[i] /= 1000
        return res

    def _process_single(self, data):
        return data[0] if data else -1


def write(self, command, method=None):
    if len(command) > 3 and command[3] == 176 and len(command) > 5:
        command = "'" + command[4] + "'" + "(" + command[5] + ")"
        command = command.encode()
    if method == "socket":
        data = b""
        if self.rasp:
            self.sock.send(str(command).encode())
        else:
            self.sock.sendall(bytes(command))

        if len(command) > 3 and command[3] == 177:
            while True:
                data = self.sock.recv(1024)
                if b"password" in data:
                    break
        elif len(command) > 3 and command[3] == 192:
            while True:
                data += self.sock.recv(1024)
                if len(data) == 6:
                    break
        else:
            try:
                self.sock.settimeout(1)
                data = self.sock.recv(1024)
            except:
                data = b""
        return data
    else:
        self.log.debug("_write: {}".format([hex(i) for i in command]))
        self._serial_port.write(command)
        self._serial_port.flush()


def read(self, genre):
    datas = b""
    data_len = -1
    k = 0
    pre = 0
    t = time.time()
    wait_time = 0.1
    if genre == ProtocolCode.GET_SSID_PWD:
        time.sleep(0.1)
        if self._serial_port.inWaiting() > 0:
            datas = self._serial_port.read(self._serial_port.inWaiting())
        return datas
    elif genre == ProtocolCode.GET_ACCEI_DATA:
        wait_time = 1
    while True and time.time() - t < wait_time:
        data = self._serial_port.read()
        k += 1
        if data_len == 1 and data == b"\xfa":
            datas += data
            break
        elif len(datas) == 2:
            data_len = struct.unpack("b", data)[0]
            datas += data
        elif len(datas) > 2 and data_len > 0:
            datas += data
            data_len -= 1
        elif data == b"\xfe":
            if datas == b"":
                datas += data
                pre = k
            else:
                if k - 1 == pre:
                    datas += data
                else:
                    datas = b"\xfe"
                    pre = k
    else:
        datas = None
    self.log.debug("_read: {}".format(datas))
    return datas
