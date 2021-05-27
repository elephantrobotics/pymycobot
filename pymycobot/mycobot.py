import time
import serial
import math
import logging
import sys

from pymycobot.log import setup_logging
from pymycobot.error import check_parameters
from pymycobot.common import Command, MyCobotData


class MyCobot(MyCobotData):
    """MyCobot Python API (Chain operation: *)

    Possessed function:

        # Overall status
            version()
            power_on()
            power_off()
            is_power_on()
            release_all_servos()
            is_controller_connected()
            set_free_mode()

        # MDI mode and operation
            get_angles()
            send_angle()
            send_angles()
            sync_send_angles() *
            get_radians()
            send_radians()
            get_coords()
            send_coords()
            sync_send_coords() *
            pause()
            resume()
            stop()
            is_paused()
            is_in_position()
            is_moving() x

        # JOG mode and operation
            jog_angle()
            jog_coord()
            jog_stop()
            set_encoder()
            get_encoder()
            set_encoders()

        # Running status and Settings
            get_speed()
            set_speed() *
            get_joint_min_angle()
            get_joint_max_angle()

        # Servo control
            is_servo_enable()
            is_all_servo_enable()
            set_servo_data()
            get_servo_data()
            set_servo_calibration()
            release_servo()
            focus_servo()

        # Atom IO
            set_color() *
            set_led_color()
            set_pin_mode()
            set_digital_output()
            get_digital_input()
            set_pwm_mode() x
            set_pwm_output()
            get_gripper_value()
            set_gripper_state() *
            set_gripper_value()
            set_gripper_ini()
            is_gripper_moving()

        # Basic
            set_basic_output() *


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
        self._version = sys.version_info[:2][0]
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._serial_port = serial.Serial(port, baudrate, timeout=timeout)

    def _write(self, command):
        self.log.debug("_write: {}".format(command))

        self._serial_port.write(command)
        self._serial_port.flush()
        time.sleep(0.05)

    def _read(self, size=1024):
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
        else:
            self.log.debug("_read: no data can be read")

            data = None
        return data

    def __mesg(self, genre, *args, **kwargs):
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
        command_data = self._process_data_command(args)

        LEN = len(command_data) + 2
        command = [
            Command.HEADER,
            Command.HEADER,
            LEN,
            genre,
            command_data,
            Command.FOOTER,
        ]
        self._write(self._flatten(command))

        if "has_reply" in kwargs and kwargs["has_reply"]:
            data = self._read()
            res = self._process_received(data, genre)
            return res

    # System status
    def version(self):  # TODO: test method <11-03-21, yourname> #
        """Get cobot version

        Return:
            mycobot   : 1
            mycobotPro: 101
        """
        recv = self.__mesg(Command.VERSION, has_reply=True)
        return recv

    # Overall status
    def power_on(self):
        self.__mesg(Command.POWER_ON)

    def power_off(self):
        self.__mesg(Command.POWER_OFF)

    def is_power_on(self):
        """Adjust robot arm status

        Return:
            1 : power on
            0 : power off
            -1: error data
        """
        return self._process_single(self.__mesg(Command.IS_POWER_ON, has_reply=True))

    def release_all_servos(self):
        self.__mesg(Command.RELEASE_ALL_SERVOS)

    def is_controller_connected(self):
        return self._process_single(
            self.__mesg(Command.IS_CONTROLLER_CONNECTED, has_reply=True)
        )

    """
    def set_free_mode(self, flag):  # TODO:no finish
        if flag:
            self.__mesg(Command.SET_FREE_MODE, 1)
        else:
            self.__mesg(Command.SET_FREE_MODE, 0)

    def is_free_mode(self):  # TODO: no finish
        return self._process_single(self.__mesg(Command.IS_FREE_MODE, has_reply=True))
    """

    # MDI mode and operation
    def get_angles(self):
        """Get all angle return a list

        Return:
            data_list (list[angle...]):
        """
        angles = self.__mesg(Command.GET_ANGLES, has_reply=True)
        return [self._int_to_angle(angle) for angle in angles]

    @check_parameters(Command.SEND_ANGLE)
    def send_angle(self, id, degree, speed):
        """Send one angle

        Args:
            id (common.Angle):
            degree (float):
            speed (int): 0 ~100
        """
        self.__mesg(Command.SEND_ANGLE, id - 1, [self._angle_to_int(degree)], speed)

    # @check_parameters(Command.SEND_ANGLES)
    def send_angles(self, degrees, speed):
        """Send all angles

        Args:
            degrees (list): example [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            speed (int): 0 ~ 100
        """
        degrees = [self._angle_to_int(degree) for degree in degrees]
        # data = [degrees, speed]
        self.__mesg(Command.SEND_ANGLES, degrees, speed)

    def sync_send_angles(self, degrees, speed, timeout=7):
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f:
                break
            time.sleep(0.1)
        return self

    def get_radians(self):
        """Get all angle return a list

        Return:
            data_list (list[radian...]):
        """
        angles = self.__mesg(Command.GET_ANGLES, has_reply=True)
        return [
            round(self._int_to_angle(angle) * (math.pi / 180), 3) for angle in angles
        ]

    def send_radians(self, radians, speed):
        """Send all angles

        Args:
            radians (list): example [0, 0, 0, 0, 0, 0]
            speed (int): 0 ~ 100
        """
        degrees = [self._angle_to_int(radian * (180 / math.pi)) for radian in radians]
        return self.__mesg(Command.SEND_ANGLES, degrees, speed)

    def get_coords(self):
        """Get all coords.

        Return:
            data_list (list): [x, y, z, rx, ry, rz]
        """
        received = self.__mesg(Command.GET_COORDS, has_reply=True)
        if not received:
            return received

        res = []
        for idx in range(3):
            res.append(self._int_to_coord(received[idx]))
        for idx in range(3, 6):
            res.append(self._int_to_angle(received[idx]))
        return res

    @check_parameters(Command.SEND_COORD)
    def send_coord(self, id, coord, speed):
        """Send one coord

        Args:
            id(common.Coord):
            coord(float): mm
            speed(int):
        """
        return self.__mesg(
            Command.SEND_COORD, id - 1, [self._coord_to_int(coord)], speed
        )

    @check_parameters(Command.SEND_COORDS)
    def send_coords(self, coords, speed, mode):
        """Send all coords

        Args:
            coords: [x(mm), y, z, rx(angle), ry, rz]
            speed(int);
            mode(int): 0 - angluar, 1 - linear
        """
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord_to_int(coords[idx]))
        for idx in range(3, 6):
            coord_list.append(self._angle_to_int(coords[idx]))
        self.__mesg(Command.SEND_COORDS, coord_list, speed, mode)

    def sync_send_coords(self, coords, speed, mode, timeout=7):
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1):
                break
            time.sleep(0.1)
        return self

    def pause(self):
        self.__mesg(Command.PAUSE)

    def is_paused(self):
        return self._process_single(self.__mesg(Command.IS_PAUSED, has_reply=True))

    def resume(self):
        self.__mesg(Command.RESUME)

    def stop(self):
        self.__mesg(Command.STOP)

    @check_parameters(Command.IS_IN_POSITION)
    def is_in_position(self, data, id):
        """

        Args:
            id: 1 - coords, 0 - angles

        Return:
            0 : error position
            1 : right position
            -1: error data
        """

        if id == 1:
            data_list = []
            for idx in range(3):
                data_list.append(self._coord_to_int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle_to_int(data[idx]))
        elif id == 0:
            data_list = [self._angle_to_int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        received = self.__mesg(Command.IS_IN_POSITION, data_list, id, has_reply=True)
        return self._process_single(received)

    def is_moving(self):
        """

        Return:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._process_single(self.__mesg(Command.IS_MOVING, has_reply=True))

    # JOG mode and operation
    @check_parameters(Command.JOG_ANGLE)
    def jog_angle(self, joint_id, direction, speed):
        """Joint control

        Args:
            joint_id: string
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        self.__mesg(Command.JOG_ANGLE, joint_id, direction, speed)

    @check_parameters(Command.JOG_COORD)
    def jog_coord(self, coord_id, direction, speed):
        """Coord control

        Args:
            coord: string
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        self.__mesg(Command.JOG_COORD, coord_id, direction, speed)

    def jog_stop(self):
        self.__mesg(Command.JOG_STOP)

    def set_encoder(self, joint_id, encoder):
        """Set joint encoder value.

        Args:
            joint_id: Joint id 1 - 7
            encoder: The value of the set encoder.
        """
        self.__mesg(Command.SET_ENCODER, joint_id - 1, [encoder])

    def get_encoder(self, joint_id):
        return self.__mesg(Command.GET_ENCODER, joint_id - 1, has_reply=True)

    def set_encoders(self, encoders, sp):
        self.__mesg(Command.SET_ENCODERS, encoders, sp)

    # Running status and Settings
    def get_speed(self):
        return self._process_single(self.__mesg(Command.GET_SPEED, has_reply=True))

    @check_parameters(Command.SET_SPEED)
    def set_speed(self, speed):
        """Set speed value

        Args:
            speed (int): 0 - 100
        """
        self.__mesg(Command.SET_SPEED, speed)
        return self

    """
    def get_feed_override(self):
        return self._process_single(
            self.__mesg(Command.GET_FEED_OVERRIDE, has_reply=True)
        )

    def get_acceleration(self):
        return self._process_single(
            self.__mesg(Command.GET_ACCELERATION, has_reply=True)
        )
    """

    @check_parameters(Command.GET_JOINT_MIN_ANGLE)
    def get_joint_min_angle(self, joint_id):
        angle = self.__mesg(Command.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)
        return self._int_to_angle(angle[0]) if angle else 0

    @check_parameters(Command.GET_JOINT_MAX_ANGLE)
    def get_joint_max_angle(self, joint_id):
        angle = self.__mesg(Command.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)
        return self._int_to_angle(angle[0]) if angle else 0

    # Servo control
    @check_parameters(Command.IS_SERVO_ENABLE)
    def is_servo_enable(self, servo_id):
        return self._process_single(self.__mesg(Command.IS_SERVO_ENABLE, servo_id - 1))

    def is_all_servo_enable(self):
        return self._process_single(
            self.__mesg(Command.IS_ALL_SERVO_ENABLE, has_reply=True)
        )

    def set_servo_data(self, servo_no, data_id, value):
        self.__mesg(Command.SET_SERVO_DATA, servo_no - 1, data_id, value)

    def get_servo_data(self, servo_no, data_id):
        return self._process_single(
            self.__mesg(Command.GET_SERVO_DATA, servo_no - 1, data_id, has_reply=True)
        )

    def set_servo_calibration(self, servo_no):
        self.__mesg(Command.SET_SERVO_CALIBRATION, servo_no - 1)

    @check_parameters(Command.RELEASE_SERVO)
    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: 1 ~ 6

        """
        self.__mesg(Command.RELEASE_SERVO, servo_id)

    @check_parameters(Command.FOCUS_SERVO)
    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: 1 ~ 6

        """
        self.__mesg(Command.FOCUS_SERVO, servo_id)

    # Atom IO
    @check_parameters(Command.SET_COLOR)
    def set_color(self, r=0, g=0, b=0):
        """Set the light color

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.__mesg(Command.SET_COLOR, r, g, b)
        return self

    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int):
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        self.__mesg(Command.SET_PIN_MODE, pin_no, pin_mode)

    def set_digital_output(self, pin_no, pin_signal):
        """

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        self.__mesg(Command.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        return self._process_single(
            self.__mesg(Command.GET_DIGITAL_INPUT, pin_no, has_reply=True)
        )

    '''
    def set_pwm_mode(self, pin_no, channel):
        self.__mesg(Command.SET_PWM_MODE, pin_no, channel)
    '''

    def set_pwm_output(self, channel, frequency, pin_val):
        self.__mesg(Command.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    def get_gripper_value(self):
        return self._process_single(
            self.__mesg(Command.GET_GRIPPER_VALUE, has_reply=True)
        )

    @check_parameters(Command.SET_GRIPPER_STATE)
    def set_gripper_state(self, flag, speed):
        """Set gripper switch

        Args:
            flag  (int): 0 - open, 1 - close
            speed (int): 0 ~ 100
        """
        self.__mesg(Command.SET_GRIPPER_STATE, flag, speed)
        return self

    def set_gripper_value(self, value, speed):
        """Set gripper value

        Args:
            value (int): 0 ~ 4096
            speed (int): 0 ~ 100
        """
        self.__mesg(Command.SET_GRIPPER_VALUE, [value], speed)

    def set_gripper_ini(self):
        """Set the current position to zero

        Current position value is `2048`.
        """
        self.__mesg(Command.SET_GRIPPER_INI)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._process_single(
            self.__mesg(Command.IS_GRIPPER_MOVING, has_reply=True)
        )

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """

        Args:
            pin_signal: 0 / 1
        """
        self.__mesg(Command.SET_BASIC_OUTPUT, pin_no, pin_signal)
        return self

    # Other
    def wait(self, t):
        time.sleep(t)
        return self
