# coding=utf-8

from __future__ import division
import time
import math
import socket
import logging

from pymycobot.log import setup_logging
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyBuddySocket(MyBuddyCommandGenerator):
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
        super(MyBuddySocket, self).__init__()
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
            MyBuddySocket, self)._mesg(genre, *args, **kwargs)
        data = self._write(self._flatten(real_command), "socket")
        if data:
            res = self._process_received(data, genre, arm=12)
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
            id: 1/2 (L/R)
            
        Return:
            list: A list of float radians [radian1, ...]
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, id, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, id, radians, speed):
        """Send the radians of all joints to robot arm

        Args:
            id: 1/2 (L/R).
            radians: a list of radian values( List[float]), length 6
            speed: (int )0 ~ 100
        """
        # calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, id, degrees, speed)

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
    
    def close(self):
        self.sock.close()
