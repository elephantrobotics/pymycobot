# coding=utf-8

import threading
import math
import time

from .log import setup_logging
from .generate import CommandGenerator
from .common import ProtocolCode, read, write


class MyPalletizedataException(Exception):
    pass


MIN_ID = 0
MAX_ID = 5

# In fact, most joints cannot reach plus or minus 180 degrees.
# There may be a value greater than 180 when reading the angle,
# and the maximum and minimum values are expanded for compatibility.
MIN_ANGLE = -170.0
MAX_ANGLE = 170.0


def calibration_parameters(**kwargs):
    if kwargs.get("id", None) is not None and not MIN_ID <= kwargs["id"] <= MAX_ID:
        raise MyPalletizedataException(
            "The id not right, should be {0} ~ {1}, but received {2}.".format(
                MIN_ID, MAX_ID, kwargs["id"]
            )
        )

    if (
        kwargs.get("degree", None) is not None
        and not MIN_ANGLE <= kwargs["degree"] <= MAX_ANGLE
    ):
        raise MyPalletizedataException(
            "degree value not right, should be {0} ~ {1}, but received {2}".format(
                MIN_ANGLE, MAX_ANGLE, kwargs["degree"]
            )
        )

    if kwargs.get("degrees", None) is not None:
        degrees = kwargs["degrees"]
        if not isinstance(degrees, list):
            raise MyPalletizedataException("`degrees` must be a list.")
        if len(degrees) not in [3,4]:
            raise MyPalletizedataException(
                "The length of `degrees` must be 3 /  4.")
        for idx, angle in enumerate(degrees):
            if not MIN_ANGLE <= angle <= MAX_ANGLE:
                raise MyPalletizedataException(
                    "Has invalid degree value, error on index {0}. Degree should be {1} ~ {2}.".format(
                        idx, MIN_ANGLE, MAX_ANGLE
                    )
                )

    if kwargs.get("coords", None) is not None:
        coords = kwargs["coords"]
        if not isinstance(coords, list):
            raise MyPalletizedataException("`coords` must be a list.")
        if len(coords) != 4:
            raise MyPalletizedataException(
                "The length of `coords` must be 4.")

    if kwargs.get("speed", None) is not None and not 0 <= kwargs["speed"] <= 100:
        raise MyPalletizedataException(
            "speed value not right, should be 0 ~ 100, the error speed is %s"
            % kwargs["speed"]
        )

    if kwargs.get("rgb", None) is not None:
        rgb_str = ["r", "g", "b"]
        for i, v in enumerate(kwargs["rgb"]):
            if not (0 <= v <= 255):
                raise MyPalletizedataException(
                    "The RGB value needs be 0 ~ 255, but the %s is %s" % (
                        rgb_str[i], v)
                )


class MyPalletizer(CommandGenerator):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether to show debug info
        """
        super(MyPalletizer, self).__init__(debug)
        self.calibration_parameters = calibration_parameters

        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()

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
        with self.lock:
            self._write(self._flatten(real_command))

            if has_reply:
                data = self._read(genre)
                # print(data)
                res = self._process_received(data, genre)
                if res == []:
                    return None
                if genre in [
                    ProtocolCode.ROBOT_VERSION,
                    ProtocolCode.SOFTWARE_VERSION,
                    ProtocolCode.GET_ROBOT_ID,
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
                    ProtocolCode.GET_TOF_DISTANCE,
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
                elif genre in [ProtocolCode.GET_COORDS]:
                    if res:
                        r = []
                        for idx in range(3):
                            r.append(self._int2coord(res[idx]))
                        if len(res)>3:
                            r.append(self._int2angle(res[3]))
                        return r
                    else:
                        return res
                elif genre in [
                    ProtocolCode.GET_JOINT_MIN_ANGLE,
                    ProtocolCode.GET_JOINT_MAX_ANGLE,
                ]:
                    return self._int2angle(res[0]) if res else 0
                elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
                    return self._int2coord(self._process_single(res))
                elif genre == ProtocolCode.GET_ANGLES_COORDS:
                    r = []
                    for index in range(len(res)):
                        if index < 4:
                            r.append(self._int2angle(res[index]))
                        elif index < 7:
                            r.append(self._int2coord(res[index]))
                        else:
                            r.append(self._int2angle(res[index]))
                    return r
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
        # calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def sync_send_angles(self, degrees, speed, timeout=15):
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_moving()
            if not f:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, coords, speed, mode, timeout=15):
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if not self.is_moving():
                break
            time.sleep(0.1)
        return self

    # Basic for raspberry pi.
    def gpio_init(self):
        """Init GPIO module.
        Raspberry Pi version need this.
        """
        import RPi.GPIO as GPIO  # type: ignore

        GPIO.setmode(GPIO.BCM)
        self.gpio = GPIO

    def gpio_output(self, pin, v):
        """Set GPIO output value.
        Args:
            pin: port number(int).
            v: Output value(int), 1 - GPIO.HEIGH, 0 - GPIO.LOW
        """
        self.gpio.setup(pin, self.gpio.OUT)
        self.gpio.setup(pin, v)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self
    
    def get_accie_data(self, value):
        """Get gyroscope data
        
        Args:
            value: 
                0 - Get data from a 3-axis gyroscope.\n
                1 - Get data from a 2-axis gyroscope.
            
        """
        data_list = [[25, 21], [26, 32]]
        return self._mesg(ProtocolCode.GET_ACCEI_DATA, data_list[1] if value else data_list[0], has_reply=True)
    
    def close(self):
        self._serial_port.close()
        
    def open(self):
        self._serial_port.open()
