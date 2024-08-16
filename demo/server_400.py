#!/usr/bin/env python3
# coding:utf-8
import socket
import serial
import time
import logging
import logging.handlers
import re
import fcntl
import struct
import traceback
import threading

"""
Instructions for use:

Please update pymycobot to the latest version before use.

`pip install pymycobot --upgrade`

Please change the parameters passed in the last line of the Server.py file, MercuryServer, based on your model.


"""
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
    
    SET_CW = 0x0B
    GET_CW = 0x0C
    CLEAR_WAIST_QUEUE = 0x0D
    
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
    SET_FREE_MODE = 0x1A
    IS_FREE_MODE = 0x1B
    COBOTX_GET_ANGLE = 0x1C
    POWER_ON_ONLY = 0x1D
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
    SET_GRIPPER_TORQUE = 0x6F
    IS_BTN_CLICKED = 0x6F
    SET_COLOR_MYARM = 0x70
    SET_ELECTRIC_GRIPPER = 0x6B
    INIT_ELECTRIC_GRIPPER = 0x6C
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
has_return = [0x02, 0x03, 0x04, 0x09, 0x10, 0x11, 0x12, 0x13, 0x1c, 0x18, 0x19, 0x20, 0x23, 0x27, 0x29, 0x2A, 0x2B, 0x35, 0x4A, 0x4B,0x4C, 0x4D,
              0x50, 0x51, 0x56,0x57, 0x59,0x5A,0x62, 0x82, 0x84, 0x86, 0x88, 0x8A, 0xA1, 0xA2, 0xB2, 0xB3, 0xB4, 0xB5, 0xB7, 0xD6, 0xe1, 0xe2, 0xe4]


def get_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    # DATE_FORMAT = "%m/%d/%Y %H:%M:%S %p"

    formatter = logging.Formatter(LOG_FORMAT)
    console = logging.StreamHandler()
    console.setFormatter(formatter)

    save = logging.handlers.RotatingFileHandler(
        "server.log", maxBytes=10485760, backupCount=1)
    save.setFormatter(formatter)

    logger.addHandler(save)
    logger.addHandler(console)
    return logger


class MercuryServer(object):

    def __init__(self, host, port, serial_num="/dev/ttyAMA1", baud=115200):
        """Server class

        Args:
            host: server ip address.
            port: server port.
            serial_num: serial number of the robot.The default is /dev/ttyAMA1.
            baud: baud rate of the serial port.The default is 115200.

        """
        self.logger = get_logger("AS")
        self.mc = None
        self.serial_num = serial_num
        self.baud = baud
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        print("Binding succeeded!")
        self.s.listen(1)
        self.conn = None
        self.stop = False
        self.connected = False
        self.mc = serial.Serial(self.serial_num, self.baud, timeout=0.1)
        self.connect()

    def connect(self):
        while True:
            try:
                print("waiting connect!------------------")
                self.conn, addr = self.s.accept()
                # self.connected = True
                while True:
                    try:
                        print("waiting data--------")
                        data = self.conn.recv(1024)
                        self.connected = True
                        command = []
                        for v in data:
                            command.append(v)
                        if command == []:
                            print("close disconnect!")
                            break
                        if self.mc.isOpen() == False:
                            self.mc.open()
                        else:
                            self.logger.info("get command: {}".format(
                                [hex(v) for v in command]))
                            # command = self.re_data_2(command)

                            self.write(command)
                            if command[3] == 0x29:
                                self.connected = False
                            if command[3] in has_return:
                                # res = self.read(command)
                                self.read_thread = threading.Thread(target=self.read, args=(command,), daemon=True)
                                self.read_thread.start()
                    except ConnectionResetError:
                        self.connected = False
                        pass
                    except Exception as e:
                        self.logger.error(traceback.format_exc())
                        break
            except Exception as e:
                self.logger.error(traceback.format_exc())
                self.conn.close()
                self.mc.close()
                
    def _encode_int16(self, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            res = []
            for v in data:
                t = self._encode_int16(v)
                res.extend(t)
        return res
              
    @classmethod  
    def crc_check(cls, command):
        crc = 0xffff
        for index in range(len(command)):
            crc ^= command[index]
            for _ in range(8):
                if crc & 1 == 1:
                    crc >>=  1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        if crc > 0x7FFF:
            return list(struct.pack(">H", crc))
        return cls._encode_int16(_, crc)

    def write(self, command):
        self.mc.write(command)
        self.mc.flush()

    def read(self, command):
        datas = b""
        data_len = -1
        k = 0
        pre = 0
        t = time.time()
        wait_time = 0.1
        if command[3] == 0x10:
            wait_time = 8
        elif command[3] in [0x11, 0x13, 0x18, 0x56, 0x57, 0x29]:
            wait_time = 3
        # elif command[3] in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
        #     wait_time = 300
        while True and time.time() - t < wait_time and self.connected:
            if self.mc.inWaiting() > 0:
                data = self.mc.read()
                # print("read data: ", data)
                # if data != b"":
                #     print(data, datas)
                k += 1
                if data_len == 3:
                    datas += data
                    crc = self.mc.read(2)
                    if self.crc_check(datas) == [v for v in crc]:
                        datas+=crc
                        break
                if data_len == 1 and data == b"\xfa":
                    datas += data
                    # if [i for i in datas] == command:
                    #     datas = b''
                    #     data_len = -1
                    #     k = 0
                    #     pre = 0
                    #     continue
                    break
                elif len(datas) == 2:
                    data_len = struct.unpack("b", data)[0]
                    datas += data
                elif len(datas) > 2 and data_len > 0:
                    datas += data
                    data_len -= 1
                    # if len(datas) == 4:
                    #     if datas[-1] != command[3]:
                    #         datas = b''
                    #         data_len = -1
                    #         k = 0
                    #         pre = 0
                    #         continue
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
        if self.conn is not None:
            self.logger.info("return datas: {}".format([hex(v) for v in datas]))
            
            self.conn.sendall(datas)
            datas = b''
        return datas

    def re_data_2(self, command):
        r2 = re.compile(r'[[](.*?)[]]')
        data_str = re.findall(r2, command)[0]
        data_list = data_str.split(",")
        data_list = [int(i) for i in data_list]
        return data_list


if __name__ == "__main__":
    ifname = "wlan0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack(
        '256s', bytes(ifname, encoding="utf8")))[20:24])
    PORT = 9000
    print("ip: {} port: {}".format(HOST, PORT))
    MercuryServer(HOST, PORT, "/dev/ttyAMA1", 115200)
