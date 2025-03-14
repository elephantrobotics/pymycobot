# coding=utf-8

from __future__ import division
import time
import math
import logging
import threading

from pymycobot.log import setup_logging
from pymycobot.generate import CommandGenerator
from pymycobot.public import PublicCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters
from pymycobot.sms import sms_sts


class MyCobot(CommandGenerator, PublicCommandGenerator, sms_sts):
    """MyCobot Python API Serial communication class.

    Supported methods:

        # Overall status
            Look at parent class: `CommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            Other look at parent class: `CommandGenerator`.

        # JOG mode and operation
            Look at parent class: `CommandGenerator`.

        # Running status and Settings
            Look at parent class: `CommandGenerator`.

        # Servo control
            Look at parent class: `CommandGenerator`.

        # Atom IO
            Look at parent class: `CommandGenerator`.

        # Basic
            Look at parent class: `CommandGenerator`.

        # Other
            wait() *
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False, thread_lock=True):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyCobot, self).__init__(debug)
        print("Note: This class is no longer maintained since v3.6.0, please refer to the project documentation: https://github.com/elephantrobotics/pymycobot/blob/main/README.md")
        self.calibration_parameters = calibration_parameters
        import serial
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        time.sleep(1.5)
        self.lock = threading.Lock()
        super(sms_sts, self).__init__(self._serial_port, 0)

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
        real_command, has_reply, _async = super(
            MyCobot, self)._mesg(genre, *args, **kwargs)
        if self.thread_lock:
            with self.lock:
                return self._res(real_command, has_reply, genre)
        else:
            return self._res(real_command, has_reply, genre)
            
    def _res(self, real_command, has_reply, genre):
        try_count = 0
        while try_count < 3:
            self._write(self._flatten(real_command))
            data = self._read(genre)
            if data is not None and data != b'':
                break
            try_count += 1
        else:
            return -1
        if genre == ProtocolCode.SET_SSID_PWD:
            return None
        res = self._process_received(data, genre)
        if res is None:
            return None
        if res is not None and isinstance(res, list) and len(res) == 1:
            return res[0]
        if genre in [
            ProtocolCode.ROBOT_VERSION,
            ProtocolCode.GET_ROBOT_ID,
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
            ProtocolCode.GET_FRESH_MODE,
            ProtocolCode.GET_GRIPPER_MODE,
            ProtocolCode.GET_ERROR_INFO,
            ProtocolCode.GET_COMMUNICATE_MODE,
            ProtocolCode.SET_COMMUNICATE_MODE,
            ProtocolCode.SetHTSGripperTorque,
            ProtocolCode.GetHTSGripperTorque,
            ProtocolCode.GetGripperProtectCurrent,
            ProtocolCode.InitGripper,
            ProtocolCode.SET_FOUR_PIECES_ZERO
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
        elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
            return self._int2coord(res[0])
        elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
            return self._int2coord(self._process_single(res))
        elif genre == ProtocolCode.GET_ANGLES_COORDS:
            r = []
            for index in range(len(res)):
                if index < 6:
                    r.append(self._int2angle(res[index]))
                elif index < 9:
                    r.append(self._int2coord(res[index]))
                else:
                    r.append(self._int2angle(res[index]))
            return r
        else:
            return res

    def get_radians(self):
        """Get the radians of all joints

        Return:
            list: A list of float radians [radian1, ...]
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, radians, speed):
        """Send the radians of all joints to robot arm

        Args:
            radians: a list of radian values( List[float]), length 6
            speed: (int )0 ~ 100
        """
        calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def sync_send_angles(self, degrees, speed, timeout=30):
        """Send the angle in synchronous state and return when the target point is reached
            
        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f == 1:
                return 1
            time.sleep(0.1)
        return 0

    def sync_send_coords(self, coords, speed, mode=0, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached
            
        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular（default）, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1) == 1:
                return 1
            time.sleep(0.1)
        return 0

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
    
    def close(self):
        self._serial_port.close()
        
    def open(self):
        self._serial_port.open()
        
    def get_acceleration(self):
        """get acceleration"""
        return self._process_single(
            self._mesg(ProtocolCode.GET_ACCELERATION, has_reply=True)
        )

    def set_acceleration(self, acc):
        """Set speed for all moves
        
        Args:
            acc: int
        """
        return self._mesg(ProtocolCode.SET_ACCELERATION, acc)
    
    def go_home(self):
        return self.send_angles([0,0,0,0,0,0], 10)
    
    def angles_to_coords(self, angles):
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.GET_COORDS, angles, has_reply=True)
    
    def solve_inv_kinematics(self, target_coords, current_angles):
        angles = [self._angle2int(angle) for angle in current_angles]
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(target_coords[idx]))
        for angle in target_coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)

