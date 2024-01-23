# coding=utf-8

from __future__ import division
import time
import math
import socket
import threading

from pymycobot.log import setup_logging
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyBuddySocket(MyBuddyCommandGenerator):
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
        self.lock = threading.Lock()

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
        with self.lock:
            self._write(self._flatten(real_command), "socket")
            if has_reply:
                data = self._read(genre, 'socket')
                res = self._process_received(data, genre, arm=12)
                if res == []:
                    return None
                if genre in [
                    ProtocolCode.ROBOT_VERSION,
                    ProtocolCode.SOFTWARE_VERSION,
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
                    ProtocolCode.SetHTSGripperTorque,
                    ProtocolCode.GetHTSGripperTorque,
                    ProtocolCode.GetGripperProtectCurrent,
                    ProtocolCode.InitGripper,
                    ProtocolCode.SET_FOUR_PIECES_ZERO
                    # ProtocolCode.GET_SERVO_CURRENTS
                ]:
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [ProtocolCode.GET_ANGLE]:
                    return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_COORD]:
                    if real_command[5] < 4:
                        if real_command[2] == 3:
                            return self._int2angle(res[0]) if res else None
                        return self._int2coord(res[0]) if res else None
                    else:
                        return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_ALL_BASE_COORDS, ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE, ProtocolCode.GET_BASE_COORDS, ProtocolCode.GET_BASE_COORD, ProtocolCode.BASE_TO_SINGLE_COORDS]:
                    if res:
                        r = [] 
                        for idx in range(3):
                            r.append(self._int2coord(res[idx]))
                        for idx in range(3, 6):
                            r.append(self._int2angle(res[idx]))
                        if len(res) == 12:
                            r1 = []
                            for idx in range(6, 9):
                                r1.append(self._int2coord(res[idx]))
                            for idx in range(9, 12):
                                r1.append(self._int2angle(res[idx]))
                            return [r, r1]
                        return r
                    else:
                        return res
                elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
                    return self._int2coord(res[0])
                elif genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.COLLISION]:
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
        
    def sync_send_angles(self, id, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached
            
        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(id, degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(id, degrees, 0)
            if f == 1:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, id, coords, speed, mode=0, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached
            
        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular（default）, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(id, coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(id, coords, 1) == 1:
                break
            time.sleep(0.1)
        return self
