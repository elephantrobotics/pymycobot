# coding=utf-8

from __future__ import division
import time
import math
import socket
import logging

from pymycobot.log import setup_logging
from pymycobot.generate import MyCobotCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyCobotSocket(MyCobotCommandGenerator):
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
    _write = write
    _read = read

    def __init__(self, ip, netport=9000):
        """
        Args:
            ip: Server ip
            netport: Server port
        """
        super(MyCobotSocket, self).__init__()
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.rasp = False
        self.sock = self.connect_socket()

    def connect(self, serialport="/dev/ttyAMA0", baudrate="1000000", timeout='0.1'):
        """Connect the robot arm through the serial port and baud rate
        Args:
            serialport: (str) default /dev/ttyAMA0
            baudrate: default 1000000
            timeout: default 0.1
        
        """
        self.rasp = True
        self._write(serialport, "socket")
        self._write(baudrate, "socket")
        self._write(timeout, "socket")

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

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
            MyCobotSocket, self)._mesg(genre, *args, **kwargs)
        # [254,...,255]
        data = self._write(self._flatten(real_command), "socket")
        if data:
            res = self._process_received(data, genre)
            if genre in [
                ProtocolCode.VERSION,
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
                ProtocolCode.GET_TOF_DISTANCE
            ]:
                return self._process_single(res)
            elif genre in [ProtocolCode.GET_ANGLES]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_COORDS]:
                if res:
                    r = []
                    for idx in range(3):
                        r.append(self._int2coord(res[idx]))
                    for idx in range(3, 6):
                        r.append(self._int2angle(res[idx]))
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

    def get_radians(self):
        """Get all angle return a list

        Return:
            data_list (list[radian...]):
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, radians, speed):
        """Send all angles

        Args:
            radians (list): example [0, 0, 0, 0, 0, 0]
            speed (int): 0 ~ 100
        """
        calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def sync_send_angles(self, degrees, speed, timeout=7):
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, coords, speed, mode, timeout=7):
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1):
                break
            time.sleep(0.1)
        return self

    def set_gpio_mode(self, mode):
        """Set pin coding method
        Args:
            mode: (str) BCM or BOARD 
        """
        self.calibration_parameters(gpiomode=mode)
        if mode == "BCM":
            return self._mesg(ProtocolCode.SET_GPIO_MODE, 0)
        else:
            return self._mesg(ProtocolCode.SET_GPIO_MODE, 1)

    def set_gpio_out(self, pin_no, mode):
        """Set the pin as input or output
        Args:
            pin_no: (int) pin id
            mode: (str) "in" or "out"
        """
        if mode == "in":
            return self._mesg(ProtocolCode.SET_GPIO_UP, pin_no, 0)
        else:
            return self._mesg(ProtocolCode.SET_GPIO_UP, pin_no, 1)

    def set_gpio_output(self, pin_no, state):
        """Set the pin to high or low level
        Args:
            pin_no: (int) pin id.
            state: (int) 0 or 1
        """
        return self._mesg(ProtocolCode.SET_GPIO_OUTPUT, pin_no, state)

    def get_gpio_in(self, pin_no):
        """Get pin level status.
        Args:
            pin_no: (int) pin id.
        """
        return self._mesg(ProtocolCode.GET_GPIO_IN, pin_no)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self
