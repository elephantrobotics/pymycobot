# coding=utf-8

from __future__ import division
import time
import struct
import sys
import platform
import locale
from pymycobot.robot_info import Robot320Info


class ProGripper(object):
    SET_GRIPPER_ANGLE = 11
    GET_GRIPPER_ANGLE = 12
    SET_GRIPPER_CALIBRATION = 13
    GET_GRIPPER_STATUS = 14
    SET_GRIPPER_TORQUE = 27
    GET_GRIPPER_TORQUE = 28
    SET_GRIPPER_SPEED = 32
    GET_GRIPPER_DEFAULT_SPEED = 33
    SET_GRIPPER_ABS_ANGLE = 36
    SET_GRIPPER_PAUSE = 37
    SET_GRIPPER_RESUME = 38
    SET_GRIPPER_STOP = 39


class ProtocolCode(object):
    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # System status
    ROBOT_VERSION = 0x01
    SOFTWARE_VERSION = 0x02
    GET_ROBOT_ID = 0x03
    OVER_LIMIT_RETURN_ZERO = 0x04
    SET_ROBOT_ID = 0x04
    
    GET_ERROR_INFO = 0x07
    CLEAR_ERROR_INFO = 0x08
    GET_ATOM_VERSION = 0x09

    SET_POS_SWITCH = 0x0B
    GET_POS_SWITCH = 0x0C

    SetHTSGripperTorque = 0x35
    GetHTSGripperTorque = 0x36
    GetGripperProtectCurrent = 0x37
    InitGripper = 0x38
    SetGripperProtectCurrent = 0x39

    # Overall status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    IS_CONTROLLER_CONNECTED = 0x14
    READ_NEXT_ERROR = 0x15
    SET_FRESH_MODE = 0x16
    GET_FRESH_MODE = 0x17
    GET_ROBOT_STATUS = 0x19
    SET_FREE_MODE = 0x1A
    IS_FREE_MODE = 0x1B
    COBOTX_GET_ANGLE = 0x1C
    POWER_ON_ONLY = 0x1D
    SET_VISION_MODE = 0x1D
    SET_CONTROL_MODE = 0x1E
    GET_CONTROL_MODE = 0x1F
    FOCUS_ALL_SERVOS = 0x18
    GO_ZERO = 0x19
    SET_BREAK = 0x19

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
    JOG_INCREMENT_COORD = 0x34

    COBOTX_GET_SOLUTION_ANGLES = 0x35
    COBOTX_SET_SOLUTION_ANGLES = 0x36

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
    SET_GRIPPER_ENABLED = 0x58
    GET_ZERO_POS = 0x59
    IS_INIT_CALIBRATION = 0x5A

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
    IS_TOOL_BTN_CLICKED = 0x6b
    SET_GRIPPER_TORQUE = 0x6F
    IS_BTN_CLICKED = 0x6F
    SET_COLOR_MYARM = 0x70
    SET_ELETRIC_GRIPPER = 0x6B
    INIT_ELETRIC_GRIPPER = 0x6C
    SET_GRIPPER_MODE = 0x6D
    GET_GRIPPER_MODE = 0x6E

    GET_ACCEI_DATA = 0x73
    SET_COLLISION_MODE = 0x74
    SET_COLLISION_THRESHOLD = 0x75
    GET_COLLISION_THRESHOLD = 0x76
    SET_TORQUE_COMP = 0x77
    GET_TORQUE_COMP = 0x78
    GET_VR_MODE = 0x79
    SET_VR_MODE = 0x7A
    GET_MODEL_DIRECTION = 0x7C
    SET_MODEL_DIRECTION = 0x7D
    GET_FILTER_LEN = 0x7E
    SET_FILTER_LEN = 0x7F

    # Basic
    SET_BASIC_OUTPUT = 0xA0
    GET_BASIC_INPUT = 0xA1
    GET_BASE_INPUT = 0xA2
    MERCURY_ROBOT_STATUS = 0xA2
    MERCURY_ERROR_COUNTS = 0xA3
    MERCURY_SET_POS_OVER_SHOOT = 0xA4
    MERCURY_GET_POS_OVER_SHOOT = 0xA5
    SET_BASE_PWM = 0xA5

    # Linux GPIO, mode: GPIO.BCM
    SET_GPIO_MODE = 0xAA
    SET_GPIO_UP = 0xAB
    SET_GPIO_OUTPUT = 0xAC
    GET_GPIO_IN = 0xAD

    # set WIFI
    SET_SSID_PWD = 0xB0
    GET_SSID_PWD = 0xB1
    TOOL_SERIAL_RESTORE = 0xB1
    TOOL_SERIAL_READY = 0xB2
    TOOL_SERIAL_AVAILABLE = 0xB3
    TOOL_SERIAL_READ_DATA = 0xB4
    TOOL_SERIAL_WRITE_DATA = 0xB5
    TOOL_SERIAL_FLUSH = 0xB6
    TOOL_SERIAL_PEEK = 0xB7
    TOOL_SERIAL_SET_BAUD = 0xB8
    TOOL_SERIAL_SET_TIME_OUT = 0xB9
    SET_SERVER_PORT = 0xB2

    # Get the measured distance
    GET_TOF_DISTANCE = 0xC0
    GET_BASIC_VERSION = 0xC1
    SET_COMMUNICATE_MODE = 0xC2
    GET_COMMUNICATE_MODE = 0xC3

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
    WRITE_MOVE_C = 0x8C
    SOLVE_INV_KINEMATICS = 0x8D

    # Impact checking
    SET_JOINT_CURRENT = 0x90
    GET_JOINT_CURRENT = 0x91
    SET_CURRENT_STATE = 0x92
    GET_POS_OVER = 0x94
    CLEAR_ENCODERS_ERROR = 0x95
    GET_DOWN_ENCODERS = 0x96

    # planning speed
    GET_PLAN_SPEED = 0xD0
    GET_PLAN_ACCELERATION = 0xD1
    SET_PLAN_SPEED = 0xD2
    SET_PLAN_ACCELERATION = 0xD3
    move_round = 0xD4
    GET_ANGLES_COORDS = 0xD5
    GET_QUICK_INFO = 0xD6
    SET_FOUR_PIECES_ZERO = 0xD7

    # Motor status read
    GET_SERVO_SPEED = 0xE1
    GET_SERVO_CURRENTS = 0xE2
    GET_SERVO_VOLTAGES = 0xE3
    GET_SERVO_STATUS = 0xE4
    GET_SERVO_TEMPS = 0xE5
    GET_SERVO_LASTPDI = 0xE6
    SERVO_RESTORE = 0xE7
    SET_VOID_COMPENSATE = 0xE7
    SET_TOQUE_GRIPPER = 0xE8
    GET_TOQUE_GRIPPER = 0xE9
    SET_ERROR_DETECT_MODE = 0xE8
    GET_ERROR_DETECT_MODE = 0xE9

    MERCURY_GET_BASE_COORDS = 0xF0
    MERCURY_SET_BASE_COORD = 0xF1
    MERCURY_SET_BASE_COORDS = 0xF2
    MERCURY_JOG_BASE_COORD = 0xF3
    
    MERCURY_DRAG_TECH_SAVE = 0x70
    MERCURY_DRAG_TECH_EXECUTE = 0x71
    MERCURY_DRAG_TECH_PAUSE = 0x72
    MERCURY_DRAG_TEACH_CLEAN = 0x73

    GET_ROBOT_MODIFIED_VERSION = 1
    GET_ROBOT_FIRMWARE_VERSION = 2
    GET_ROBOT_AUXILIARY_FIRMWARE_VERSION = 3
    GET_ROBOT_ATOM_MODIFIED_VERSION = 4
    GET_ROBOT_TOOL_FIRMWARE_VERSION = 9
    GET_ROBOT_SERIAL_NUMBER = 5
    SET_ROBOT_ERROR_CHECK_STATE = 6
    GET_ROBOT_ERROR_CHECK_STATE = 7
    GET_ROBOT_ERROR_STATUS = 0x15
    GET_ATOM_PRESS_STATUS = 0x6b
    GET_JOINTS_COORD = 0x23
    GET_ROBOT_TOOL_MODIFY_VERSION = 0x04
    GET_ATOM_LED_COLOR = 0x6a
    SET_ATOM_PIN_STATUS = 0x61
    GET_ATOM_PIN_STATUS = 0x62
    SET_MASTER_PIN_STATUS = 0x65
    GET_MASTER_PIN_STATUS = 0x66
    SET_AUXILIARY_PIN_STATUS = 0xa0
    GET_AUXILIARY_PIN_STATUS = 0xa1
    SET_SERVO_MOTOR_CLOCKWISE = 0x73
    GET_SERVO_MOTOR_CLOCKWISE = 0Xea
    SET_SERVO_MOTOR_COUNTER_CLOCKWISE = 0x74
    GET_SERVO_MOTOR_COUNTER_CLOCKWISE = 0xeb
    SET_SERVO_MOTOR_CONFIG = 0x52
    GET_SERVO_MOTOR_CONFIG = 0x53
    CLEAR_RECV_QUEUE = 0x19
    GET_RECV_QUEUE_LENGTH = 0x08
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
    CLEAR_ROBOT_ERROR = 0x16
    GET_RECV_QUEUE_SIZE = 0x17
    SET_RECV_QUEUE_SIZE = 0x18
    GET_SERVOS_ENCODER_DRAG = 0xEF
    RESTORE_SERVO_SYSTEM_PARAM = 0x0a
    GET_SERVO_D = 0xE8
    SET_SERVO_P = 0x70
    GET_SERVO_P = 0xE7
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
    IS_MOVING_END = "M9"
    GET_GRIPPER_ANGLE = "M50"
    SET_SYSTEM_VALUE = "M51"
    GET_SYSTEM_VALUE = "M52"


class DataProcessor(object):
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
        command_data = self._process_data_command(genre, self.__class__.__name__, args)
        if genre == 178:
            # 修改wifi端口
            command_data = self._encode_int16(command_data)

        elif genre in [76, 77]:
            command_data = [command_data[0]] + self._encode_int16(command_data[1] * 10)
        elif genre == 115 and self.__class__.__name__ not in ["MyArmC", "MyArmM", "Mercury", "MercurySocket", "Pro630",
                                                              "Pro630Client", "Pro400Client", "Pro400"]:
            command_data = [command_data[1], command_data[3]]
        LEN = len(command_data) + 2
        
        command = [
            ProtocolCode.HEADER,
            ProtocolCode.HEADER,
            LEN,
            genre,
        ]
        if command_data:
            command.extend(command_data)
        if self.__class__.__name__ in ["Mercury", "MercurySocket", "Pro630", "Pro630Client", "Pro400Client", "Pro400"]:
            command[2] += 1
            command.extend(self.crc_check(command))
        else:
            command.append(ProtocolCode.FOOTER)

        real_command = self._flatten(command)
        has_reply = kwargs.get("has_reply", False)
        return real_command, has_reply

    # Functional approach
    def _encode_int8(self, data):
        return struct.pack("b", data)

    @classmethod
    def _encode_int16(cls, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            res = []
            for v in data:
                t = cls._encode_int16(v)
                res.extend(t)
        return res

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
    
    @classmethod
    def crc_check(cls, command):
        crc = 0xffff
        for index in range(len(command)):
            crc ^= command[index]
            for _ in range(8):
                if crc & 1 == 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        if crc > 0x7FFF:
            crc_res = list(struct.pack(">H", crc))
        else:
            crc_res = cls._encode_int16(crc)
        for i in range(2):
            if isinstance(crc_res[i], str):
                crc_res[i] = ord(crc_res[i])
        return crc_res

    # def encode_int16(self, data):
    #     encoded_data = []

    #     if isinstance(data, int):
    #         encoded_data.extend(struct.pack(">h", data))
    #     else:
    #         for elem in data:
    #             encoded_data.extend(struct.pack(">h", elem))

    #     return [byte for byte in encoded_data]

    def _flatten(self, lst):
        flat_list = []
        for item in lst:
            if not isinstance(item, list):
                flat_list.append(item)
            else:
                flat_list.extend(self._flatten(item))
        return flat_list

    def _process_data_command(self, genre, _class, args):
        if not args:
            return []

        processed_args = []
        for index in range(len(args)):
            if isinstance(args[index], list):
                if genre in [ProtocolCode.SET_ENCODERS_DRAG] and index in [0, 1] and _class in ["Mercury",
                                                                                                "MercurySocket",
                                                                                                "Pro630",
                                                                                                "Pro630Client",
                                                                                                "Pro400Client",
                                                                                                "Pro400"]:
                    for data in args[index]:
                        byte_value = data.to_bytes(4, byteorder='big', signed=True)
                        res = []
                        for i in range(len(byte_value)):
                            res.append(byte_value[i])
                        processed_args.extend(res)
                else:
                    processed_args.extend(self._encode_int16(args[index]))
            else:
                if isinstance(args[index], str):
                    processed_args.append(args[index])
                else:
                    if genre == ProtocolCode.SET_SERVO_DATA and _class in ["Mercury", "MercurySocket", "Pro630",
                                                                           "Pro630Client", "Pro400Client",
                                                                           "Pro400"] and index == 2:
                        byte_value = args[index].to_bytes(2, byteorder='big', signed=True)
                        res = []
                        for i in range(len(byte_value)):
                            res.append(byte_value[i])
                        processed_args.extend(res)
                    else:
                        processed_args.append(args[index])

        return processed_args

    def _is_frame_header(self, data, pos1, pos2):
        return data[pos1] == ProtocolCode.HEADER and data[pos2] == ProtocolCode.HEADER

    def _process_received(self, data, genre, arm=6):
        if genre == 177:
            data = str(data)[2:-1].split(": ")
            return data[1][0:-9], data[-1]
        elif not data:
            return None
        elif data == b'\xfe\xfe\x04[\x01\r\x87':
            # 水星到位反馈
            return 1
        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        header_i, header_j = 0, 1
        while header_j < data_len - 4:
            if self._is_frame_header(data, header_i, header_j):
                if arm in [6, 7, 14, 8]:
                    cmd_id = data[header_i + 3]
                elif arm == 12:
                    cmd_id = data[header_i + 4]
                # compare send header and received header
                if cmd_id == genre:
                    break
            header_i += 1
            header_j += 1
        else:
            return None
        if arm in [6, 7, 8]:
            data_len = data[header_i + 2] - 2
        elif arm == 12:
            data_len = data[header_i + 3] - 2
        elif arm == 14:
            data_len = data[header_i + 2] - 3
            
        unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]
        if cmd_id == ProtocolCode.GET_DIGITAL_INPUT and arm == 14:
            data_pos = header_i + 4
        elif cmd_id in unique_data:
            if arm == 12:
                data_pos = header_i + 6
            else:
                data_pos = header_i + 5
            data_len -= 1
        else:
            if arm in [6, 7, 14, 8]:
                data_pos = header_i + 4
            elif arm == 12:
                data_pos = header_i + 5
        valid_data = data[data_pos : data_pos + data_len]

        # process valid data
        res = []
        if genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_TEMPS,
                     ProtocolCode.GO_ZERO]:
            for i in valid_data:
                res.append(i)
            return res
        if data_len in [6, 8, 12, 14, 16, 24, 26, 60]:
            ignor_t = (
                ProtocolCode.GET_SERVO_CURRENTS,
                ProtocolCode.GET_SERVO_TEMPS,
                ProtocolCode.GET_SERVO_VOLTAGES,
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.IS_ALL_SERVO_ENABLE
            )
            if data_len == 8 and (
                    (arm == 14 and cmd_id == ProtocolCode.IS_INIT_CALIBRATION) or
                    (arm == 8 and cmd_id in ignor_t)
            ) or data_len == 6 and cmd_id in ignor_t:
                for v in valid_data:
                    res.append(v)
                return res
            elif data_len == 8 and arm == 14 and cmd_id == ProtocolCode.GET_DOWN_ENCODERS:
                i = 0
                while i < data_len:
                    byte_value = int.from_bytes(valid_data[i:i + 4], byteorder='big', signed=True)
                    i += 4
                    res.append(byte_value)
                return res
            for header_i in range(0, len(valid_data), 2):
                one = valid_data[header_i: header_i + 2]
                res.append(self._decode_int16(one))

            if genre in [ProtocolCode.GET_SERVO_STATUS]:
                res.clear()
                for i in valid_data:
                    res.append(i)
            error_key = None
            if genre == ProtocolCode.GET_ROBOT_STATUS:
                error_key = "robot_error"
            if genre == ProtocolCode.GET_SERVO_STATUS:
                error_key = "servo_error"
            if error_key is not None:
                robot_320_info = Robot320Info.error_info
                locale_lang = locale.getdefaultlocale()[0]
                if locale_lang not in ["zh_CN", "en_US"]:
                    locale_lang = "en_US"
                for i in range(len(res)):
                    if res[i] != 0:
                        data_bin = bin(res[i])[2:]
                        error_list = []
                        data_bin_len = len(data_bin)
                        for j in range(data_bin_len):
                            if data_bin[data_bin_len - 1 - j] != 0:
                                error_list.append(j)
                                if locale_lang == "zh_CN":
                                    print("错误: 关节{} - {}".format(i + 1,
                                                                 robot_320_info[locale_lang][error_key].get(j, 255)))
                                else:
                                    print("Error: Joint{} - {}".format(i + 1,
                                                                       robot_320_info[locale_lang][error_key].get(j,
                                                                                                                  255)))
                        res[i] = error_list
        elif data_len == 2:
            if genre in [
                ProtocolCode.GET_PLAN_SPEED,
                ProtocolCode.GET_PLAN_ACCELERATION,
            ]:
                return [
                    self._decode_int8(valid_data[0:1]),
                    self._decode_int8(valid_data[1:]),
                ]
            elif genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            elif genre in [ProtocolCode.GET_ERROR_INFO, ProtocolCode.GET_MASTER_PIN_STATUS]:
                return [self._decode_int8(valid_data[1:])]
            res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            if genre == ProtocolCode.GET_ATOM_PRESS_STATUS:
                return [data for data in valid_data]
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre == ProtocolCode.COBOTX_GET_ANGLE and arm == 14:
                byte_value = int.from_bytes(valid_data, byteorder='big', signed=True)
                res.append(byte_value)
                return res
            for i in range(1, 4):
                res.append(valid_data[i])
        elif data_len == 7:
            error_list = [i for i in valid_data]
            for i in error_list:
                if i in range(16, 23):
                    res.append(1)
                elif i in range(23, 29):
                    res.append(2)
                elif i in range(32, 112):
                    res.append(3)
                else:
                    res.append(i)
        elif data_len == 28:
            for i in range(0, data_len, 4):
                byte_value = int.from_bytes(valid_data[i:i + 4], byteorder='big', signed=True)
                res.append(byte_value)
        elif data_len == 40:
            i = 0
            while i < data_len:
                if i < 28:
                    byte_value = int.from_bytes(valid_data[i:i + 4], byteorder='big', signed=True)
                    res.append(byte_value)
                    i += 4
                else:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        elif data_len == 30:
            i = 0
            res = []
            while i < 30:
                if i < 9 or i >= 23:
                    res.append(valid_data[i])
                    i += 1
                elif i < 23:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
            return res
        elif data_len == 32:
            def byte2int(bvs):
                return list(map(lambda _i: self._decode_int16(bvs[_i:_i + 2]), range(0, 16, 2)))
            return [byte2int(valid_data[0:16]), byte2int(valid_data[16:32])]

        elif data_len == 38:
            i = 0
            res = []
            while i < data_len:
                if i < 10 or i >= 30:
                    res.append(valid_data[i])
                    i += 1
                elif i < 38:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
            return res
        else:
            if genre in [
                ProtocolCode.GET_SERVO_VOLTAGES,
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.GET_SERVO_TEMPS,
                ProtocolCode.GET_TOQUE_GRIPPER,
            ]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i: i + 1])
                    res.append(0xFF & data1 if data1 < 0 else data1)
                return res
            res.append(self._decode_int8(valid_data))
        if genre == ProtocolCode.GET_ACCEI_DATA:
            for i in range(len(res)):
                res[i] /= 1000
        return res

    def _process_single(self, data):
        return data[0] if data else -1

    def _process_high_low_bytes(self, data):
        if not data:
            return -1

        if len(data) < 2:
            return -1

        # Get the last two bits, low and high
        low_byte = data[-1]
        high_byte = data[-2]
        combined_value = (high_byte << 8) | low_byte
        return combined_value

    @staticmethod
    def check_python_version():
        if sys.version_info.major == 2:
            return 2
        elif sys.version_info.major == 3:
            return 3
        else:
            return -1


def write(self, command, method=None):
    if len(command) > 3 and command[3] == 176 and len(command) > 5:
        command = "'" + command[4] + "'" + "(" + command[5] + ")"
        command = command.encode()
    if method == "socket":
        log_command = ""
        for i in command:
            if isinstance(i, str):
                log_command += i
            else:
                log_command += hex(i)[2:] + " "
        self.log.debug("_write: {}".format(log_command))

        py_version = DataProcessor.check_python_version()
        if py_version == 2:
            self.sock.sendall("".join([chr(b) for b in command]))
        else:
            if isinstance(command, str):
                command = command.encode()
            else:
                command = bytes(command)
            self.sock.sendall(command)
    else:
        self._serial_port.reset_input_buffer()
        command_log = ""
        for i in command:
            if isinstance(i, str):
                command_log += i[2:] + " "
            else:
                command_log += hex(i)[2:] + " "
        self.log.debug("_write: {}".format(command_log))
        self._serial_port.write(command)
        self._serial_port.flush()


def read(self, genre, method=None, command=None, _class=None, timeout=None):
    datas = b""
    data_len = -1
    k = 0
    pre = 0
    t = time.time()
    if platform.system() == "Windows":
        wait_time = 0.15
    else:
        wait_time = 0.5
    if method is not None:
        wait_time = 0.3
    if timeout is not None:
        wait_time = timeout
    if _class in ["Mercury", "MercurySocket", "Pro630", "Pro630Client", "Pro400Client", "Pro400"]:
        if genre == ProtocolCode.POWER_ON:
            wait_time = 8
        elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.RELEASE_ALL_SERVOS, ProtocolCode.FOCUS_ALL_SERVOS,
                       ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP]:
            wait_time = 3
        elif genre in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD,
                       ProtocolCode.SEND_COORDS, ProtocolCode.JOG_ANGLE, ProtocolCode.JOG_COORD,
                       ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD,
                       ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
            wait_time = 300
    elif _class in ["MyCobot", "MyCobotSocket", "MyCobot320", "MyCobot320Socket"]:
        if genre == ProtocolCode.GET_ROBOT_STATUS:
            wait_time = 75
    data = b""

    if method is not None:
        if genre == 177:
            while True:
                data = self.sock.recv(1024)
                if b"password" in data:
                    break
        elif genre == 192:
            while True:
                data += self.sock.recv(1024)
                if len(data) == 6:
                    break
        else:
            try:
                self.sock.settimeout(wait_time)
                data = self.sock.recv(1024)
                if isinstance(data, str):
                    datas = bytearray()
                    for i in data:
                        datas += hex(ord(i))
            except:
                data = b""
        if DataProcessor.check_python_version() == 2:
            command_log = ""
            for d in data:
                command_log += hex(ord(d))[2:] + " "
            self.log.debug("_read : {}".format(command_log))
            # self.log.debug("_read: {}".format([hex(ord(d)) for d in data]))
        else:
            command_log = ""
            for d in data:
                command_log += hex(d)[2:] + " "
            self.log.debug("_read : {}".format(command_log))
        return data
    else:
        if genre == ProtocolCode.GET_SSID_PWD:
            time.sleep(0.1)
            if self._serial_port.inWaiting() > 0:
                datas = self._serial_port.read(self._serial_port.inWaiting())
            return datas
        elif genre == ProtocolCode.GET_ACCEI_DATA:
            wait_time = 1
        while True and time.time() - t < wait_time:
            data = self._serial_port.read()
            # self.log.debug("data: {}".format(data))
            k += 1
            if _class in ["Mercury", "MercurySocket", "Pro630", "Pro630Client", "Pro400Client", "Pro400"]:
                if data_len == 3:
                    datas += data
                    crc = self._serial_port.read(2)
                    if DataProcessor.crc_check(datas) == [v for v in crc]:
                        datas += crc
                        break
            if data_len == 1 and data == b"\xfa":
                datas += data
                if [i for i in datas] == command:
                    if genre in (
                        ProtocolCode.GET_ATOM_PRESS_STATUS,
                        ProtocolCode.GET_SERVO_MOTOR_COUNTER_CLOCKWISE,
                        ProtocolCode.GET_SERVO_MOTOR_CLOCKWISE,
                        ProtocolCode.GET_SERVO_P,
                    ) and _class in ["MyArmM", "MyArmC", "MyArmAPI"]:
                        break
                    datas = b''
                    data_len = -1
                    k = 0
                    pre = 0
                    continue
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
            datas = b''
        if DataProcessor.check_python_version() == 2:
            command_log = ""
            for d in datas:
                command_log += hex(ord(d))[2:] + " "
            self.log.debug("_read : {}".format(command_log))
        else:
            command_log = ""
            for d in datas:
                command_log += hex(d)[2:] + " "
            self.log.debug("_read : {}".format(command_log))

        return datas
