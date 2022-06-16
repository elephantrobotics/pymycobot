# coding=utf-8

from __future__ import division
import time
import math
import logging
import struct

from pymycobot.log import setup_logging
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyBuddy(MyBuddyCommandGenerator):
    """MyCobot Python API Serial communication class.

    Supported methods:

        # Overall status
            Look at parent class: `MyCobotCommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            Other look at parent class: `MyCobotCommandGenerator`.

        # JOG mode and operation
            Look at parent class: `MyCobotCommandGenerator`.

        # Running status and Settings
            Look at parent class: `MyCobotCommandGenerator`.

        # Servo control
            Look at parent class: `MyCobotCommandGenerator`.

        # Atom IO
            Look at parent class: `MyCobotCommandGenerator`.

        # Basic
            Look at parent class: `MyCobotCommandGenerator`.

        # Other
            wait() *
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyBuddy, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()

    _write = write
    
    def _read(self):
        datas = b''
        data_len = -1
        real_data = b''
        check_digit = 0
        k = 0
        pre = 0
        t = time.time()
        while True and time.time() - t < 0.1:
            try:
                data = self._serial_port.read()
                k+=1
                if data_len == 1:
                    datas += data
                    if struct.pack("B", check_digit & 0xff) == data:
                        break
                elif len(datas) == 3:
                    data_len = struct.unpack("b",data)[0]
                    datas += data
                elif len(datas)>=2:
                    datas += data
                    if len(datas) == 5: 
                        check_digit += self._decode_int8(data)
                    elif len(datas)>5:
                        real_data += data
                    if real_data != b'':
                        check_digit += self._decode_int8(real_data)
                        real_data = b''
                    data_len -= 1
                elif data == b'\xfe':
                    if datas == b'':
                        datas += data
                        pre = k
                    else:
                        if k-1 == pre:
                            datas += data
                        else:
                            datas = b'\xfe'
                            pre = k  
            except:
                datas = None
                break
        else:
            datas = None
        return datas
    
    def _process_received(self, data, genre):
        if genre == 177:
            data = str(data)[2:-1].split(": ")
            return data[1][0:-9], data[-1]
        if not data:
            return []
        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        header_i, header_j = 0, 1
        while header_j < data_len-4:
            if self._is_frame_header(data, header_i, header_j):
                cmd_id = data[header_i + 4]
                # compare send header and received header
                if cmd_id == genre:
                    break
            header_i += 1
            header_j += 1
        else:
            return []
        data_len = data[header_i + 3] - 2
        unique_data = [ProtocolCode.GET_BASIC_INPUT,
                       ProtocolCode.GET_DIGITAL_INPUT]

        if cmd_id in unique_data:
            data_pos = header_i + 5
            data_len -= 1
        else:
            data_pos = header_i + 5
        valid_data = data[data_pos: data_pos + data_len]

        # process valid data
        res = []
        if data_len == 12 or data_len == 8 or data_len == 24:
            for header_i in range(0, len(valid_data), 2):
                one = valid_data[header_i: header_i + 2]
                res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.GET_PLAN_SPEED, ProtocolCode.GET_PLAN_ACCELERATION]:
                return [self._decode_int8(valid_data[0:1]), self._decode_int8(valid_data[1:])]
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        else:
            if genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_TEMPS]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i:i+1])
                    res.append(256+data1 if data1 <0 else data1)
                return res
            res.append(self._decode_int8(valid_data))
        return res

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
        real_command, has_reply = super(
            MyBuddy, self)._mesg(genre, *args, **kwargs)
        self._write(self._flatten(real_command))

        if has_reply:
            data = self._read()
            res = self._process_received(data, genre)
            if genre in [
                ProtocolCode.ROBOT_VERSION,
                ProtocolCode.IS_POWER_ON,
                ProtocolCode.IS_CONTROLLER_CONNECTED,
                ProtocolCode.IS_PAUSED,  # TODO have bug: return b''
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
                ProtocolCode.GET_TOF_DISTANCE,
                ProtocolCode.GET_END_TYPE,
                ProtocolCode.GET_MOVEMENT_TYPE,
                ProtocolCode.GET_REFERENCE_FRAME,
                ProtocolCode.GET_JOINT_MIN_ANGLE,
                ProtocolCode.GET_JOINT_MAX_ANGLE
            ]:
                return self._process_single(res)
            elif genre in [ProtocolCode.GET_ANGLES]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE]:
                if res:
                    r = []
                    for idx in range(3):
                        r.append(self._int2coord(res[idx]))
                    for idx in range(3, 6):
                        r.append(self._int2angle(res[idx]))
                    return r
                else:
                    return res
            elif genre in [ProtocolCode.GET_SERVO_VOLTAGES]:
                return [self._int2coord(angle) for angle in res]
            else:
                return res
        return None

    def get_radians(self, id):
        """Get the radians of all joints

        Args: 
            id: 1/2/3 (L/R/W)
            
        Return:
            list: A list of float radians [radian1, ...]
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, id, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, id, radians, speed):
        """Send the radians of all joints to robot arm

        Args:
            id: 1/2/3 (L/R/W).
            radians: a list of radian values( List[float]), length 6
            speed: (int )0 ~ 100
        """
        calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, id, degrees, speed)

    def sync_send_angles(self, id, degrees, speed, timeout=7):
        """Send the angle in synchronous state and return when the target point is reached
            
        Args:
            id: 1/2/3 (L/R/W).
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(id, degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, id, coords, speed, mode, timeout=7):
        """Send the coord in synchronous state and return when the target point is reached
            
        Args:
            id: 1/2/3 (L/R/W).
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(id, coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1):
                break
            time.sleep(0.1)
        return self

    # Basic for raspberry pi.
    def gpio_init(self):
        """Init GPIO module, and set BCM mode."""
        import RPi.GPIO as GPIO  # type: ignore

        GPIO.setmode(GPIO.BCM)
        self.gpio = GPIO

    def gpio_output(self, pin, v):
        """Set GPIO output value.

        Args:
            pin: (int)pin number.
            v: (int) 0 / 1
        """
        self.gpio.setup(pin, self.gpio.OUT)
        self.gpio.output(pin, v)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self
