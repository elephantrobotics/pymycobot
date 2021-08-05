import time
import math
import logging

from pymycobot.log import setup_logging
from pymycobot.generate import MycobotCommandGenerater
from pymycobot.common import Command
from pymycobot.error import check_datas


class MyCobot(MycobotCommandGenerater):
    """MyCobot Python API Serial communication class.

    Supported methods:

        # Overall status
            Look at parent class: `MycobotCommandGenerater`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            Other look at parent class: `MycobotCommandGenerater`.

        # JOG mode and operation
            Look at parent class: `MycobotCommandGenerater`.

        # Running status and Settings
            Look at parent class: `MycobotCommandGenerater`.

        # Servo control
            Look at parent class: `MycobotCommandGenerater`.

        # Atom IO
            Look at parent class: `MycobotCommandGenerater`.

        # Basic
            Look at parent class: `MycobotCommandGenerater`.

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
        super(MyCobot, self).__init__(debug)
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        import serial

        self._serial_port = serial.Serial(port, baudrate, timeout=timeout)

    def _write(self, command):
        self.log.debug("_write: {}".format(command))

        self._serial_port.write(command)
        self._serial_port.flush()
        time.sleep(0.05)

    def _read(self):
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
            self.log.debug("_read: {}".format(data))
        else:
            self.log.debug("_read: no data can be read")
            data = None
        return data

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
        real_command, has_reply = super(MyCobot, self)._mesg(genre, *args, **kwargs)
        self._write(self._flatten(real_command))

        if has_reply:
            data = self._read()
            res = self._process_received(data, genre)
            if genre in [
                Command.IS_POWER_ON,
                Command.IS_CONTROLLER_CONNECTED,
                Command.IS_PAUSED,
                Command.IS_IN_POSITION,
                Command.IS_MOVING,
                Command.IS_SERVO_ENABLE,
                Command.IS_ALL_SERVO_ENABLE,
                Command.GET_SERVO_DATA,
                Command.GET_DIGITAL_INPUT,
                Command.GET_GRIPPER_VALUE,
                Command.IS_GRIPPER_MOVING,
                Command.GET_SPEED,
                Command.GET_ENCODER,
                Command.GET_BASIC_INPUT,
            ]:
                return self._process_single(res)
            elif genre in [Command.GET_ANGLES]:
                return [self._int_to_angle(angle) for angle in res]
            elif genre in [Command.GET_COORDS]:
                if res:
                    r = []
                    for idx in range(3):
                        r.append(self._int_to_coord(res[idx]))
                    for idx in range(3, 6):
                        r.append(self._int_to_angle(res[idx]))
                    return r
                else:
                    return res
            elif genre in [Command.GET_JOINT_MIN_ANGLE, Command.GET_JOINT_MAX_ANGLE]:
                return self._int_to_angle(res[0]) if res else 0
            else:
                return res
        return None

    def get_radians(self):
        """Get all angle return a list

        Return:
            data_list (list[radian...]):
        """
        angles = self._mesg(Command.GET_ANGLES, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, radians, speed):
        """Send all angles

        Args:
            radians (list): example [0, 0, 0, 0, 0, 0]
            speed (int): 0 ~ 100
        """
        check_datas(len6=radians, speed=speed)
        degrees = [self._angle_to_int(radian * (180 / math.pi)) for radian in radians]
        return self._mesg(Command.SEND_ANGLES, degrees, speed)

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

    # Basic for raspberry pi.
    def gpio_init(self):
        """Init GPIO module.
        Raspberry Pi version need this.
        """
        import RPi.GPIO as GPIO

        GPIO.setmode(GPIO.BCM)
        self.gpio = GPIO

    def gpio_output(self, pin, v):
        """Set GPIO output value.
        Args:
            pin: port number(int).
            v: Output value(int), 1 - GPIO.HEIGH, 2 - GPIO.LOW
        """
        self.gpio.setup(pin, self.gpio.OUT)
        self.gpio.setup(pin, v)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self
