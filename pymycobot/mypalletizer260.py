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
        if len(degrees) not in [3, 4]:
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


class MyPalletizer260(CommandGenerator):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether to show debug info
        """
        super(MyPalletizer260, self).__init__(debug)
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
        real_command, has_reply = super(MyPalletizer260, self)._mesg(
            genre, *args, **kwargs
        )
        with self.lock:
            self._write(self._flatten(real_command))

            if has_reply:
                data = self._read(genre)
                res = self._process_received(data, genre)
                if res == []:
                    return None
                if genre in [
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
                        if len(res) > 3:
                            r.append(self._int2angle(res[3]))
                        return r
                    else:
                        return res
                elif genre in [
                    ProtocolCode.GET_JOINT_MIN_ANGLE,
                    ProtocolCode.GET_JOINT_MAX_ANGLE,
                ]:
                    return self._int2coord(res[0]) if res else 0
                elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION,
                               ProtocolCode.GET_ATOM_VERSION]:
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

    # Overall Status
    def set_free_mode(self, flag):
        """set to free mode

        Args:
            flag: 0/1
        """
        self.calibration_parameters(class_name=self.__class__.__name__, flag=flag)
        return self._mesg(ProtocolCode.SET_FREE_MODE, flag)

    def is_free_mode(self):
        """Check if it is free mode

        Return:
            0/1
        """
        return self._mesg(ProtocolCode.IS_FREE_MODE, has_reply=True)

    # MDI mode and operation
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

    def send_angle(self, id, degree, speed):
        """Send one angle of joint to robot arm.

        Args:
            id : Joint id(genre.Angle) int 1-4.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed, has_reply=True)

    def send_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 4.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True)

    def get_coords(self):
        """Get the coords from robot arm, coordinate system based on base.

        Return:
            list : A float list of coord . [x, y, z, θ].
        """
        return self._mesg(ProtocolCode.GET_COORDS, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one coord to robot arm.

        Args:
            id(int) : coord id(genre.Coord) int 1-4.
            coord(float) : coord value, mm
            speed(int) : 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, coord=coord, speed=speed)
        value = self._coord2int(coord) if id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, id, [value], speed, has_reply=True)

    def send_coords(self, coords, speed):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]). [x, y, z, θ]
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True)

    def is_in_position(self, data, id=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords. len 4
            id: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if id == 1:
            self.calibration_parameters(class_name=self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 4):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            self.calibration_parameters(class_name=self.__class__.__name__, angles=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, id, has_reply=True)

    def sync_send_angles(self, degrees, speed, timeout=15):
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_moving()
            if not f:
                break
            time.sleep(0.1)
        return 1

    def sync_send_coords(self, coords, speed, timeout=15):
        t = time.time()
        self.send_coords(coords, speed)
        while time.time() - t < timeout:
            if not self.is_moving():
                break
            time.sleep(0.1)
        return 1

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Jog control angle.
        Args:
            joint_id: int 1-4.
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int 1-4.
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coord_id=coord_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_absolute(self, joint_id, angle, speed):
        """Jog absolute angle

        Args:
            joint_id:  int 1-4.
            angle: -180 ~ 180
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, angle=angle, speed=speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, joint_id, [self._angle2int(angle)], speed)

    def jog_increment(self, joint_id, increment, speed):
        """step mode

        Args:
            joint_id: int 1-4.
            increment:
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def jog_stop(self):
        """Stop jog moving"""
        return self._mesg(ProtocolCode.JOG_STOP)

    def set_encoder(self, joint_id, encoder, speed):
        """Set a single joint rotation to the specified potential value.

        Args:
            joint_id: int 1 - 4
            encoder: The value of the set encoder.
            speed : 1 - 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id, encoder=encoder,
                                    speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder], speed)

    def get_encoder(self, joint_id):
        """Obtain the specified joint potential value.

        Args:
            joint_id: (int) 1 - 4
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id)
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id, has_reply=True)

    def set_encoders(self, encoders, sp):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            encoders: A encoder list. len 4
            sp: speed 1 ~ 100
        """
        return self._mesg(ProtocolCode.SET_ENCODERS, encoders, sp)

    # Running Status and Settings
    def get_speed(self):
        """
        Get speed
        """
        return self._mesg(ProtocolCode.GET_SPEED, has_reply=True)

    def set_speed(self, speed):
        """
        Set speed
        :param speed: 0-100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
        return self._mesg(ProtocolCode.SET_SPEED, speed)

    def get_feed_override(self):
        return self._process_single(
            self._mesg(ProtocolCode.GET_FEED_OVERRIDE, has_reply=True)
        )

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

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            joint_id: 0 - 3
        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: int 0 - 3
        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    def set_joint_min(self, id, angle):
        """Set the joint minimum angle

        Args:
            id: int. 0 - 3
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

    def set_joint_max(self, id, angle):
        """Set the joint maximum angle

        Args:
            id: int. 0 - 3
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    # Servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id: Joint id 1 - 4
        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id, has_reply=True)

    def set_servo_data(self, servo_id, data_id, value, mode=None):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Serial number of articulated steering gear. Joint id 1 - 4
            data_id: Data address.
            value: 0 - 4096
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
        """
        if mode is None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, value=value)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, value)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, value=value,
                                        mode=mode)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, [value], mode)

    def get_servo_data(self, servo_id, data_id, mode=None):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Serial number of articulated steering gear. Joint id 1 - 4
            data_id: Data address.
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode is not None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, mode=mode)
            return self._mesg(
                ProtocolCode.GET_SERVO_DATA, servo_id, data_id, mode, has_reply=True
            )
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id)
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, servo_id, data_id, has_reply=True
        )

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point,
            and the corresponding potential value is 2048.

        Args:
            servo_id: Serial number of articulated steering gear. Joint id 1 - 4
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def release_servo(self, servo_id, mode=None):
        """Power off designated servo

        Args:
            servo_id: int Joint id 1 - 4
            mode: Default damping, set to 1, cancel damping
        """
        if mode is None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id, mode)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int Joint id 1 - 4
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

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

    # Atom IO
    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int): pin number.
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_mode=pin_mode)
        return self._mesg(ProtocolCode.SET_PIN_MODE, pin_no, pin_mode)

    def get_gripper_value(self, gripper_type=None):
        """Get the value of gripper.

        Args:
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper
                4: Flexible gripper

        Return:
            gripper value (int)
        """
        if gripper_type is None:
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, has_reply=True)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_type=gripper_type)
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, gripper_type, has_reply=True)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 - not moving
            1 - is moving
            -1- error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()
