import math
import logging
import sys

from pymycobot.log import setup_logging
from pymycobot.error import check_datas
from pymycobot.common import Command, DataProcessor


class MycobotCommandGenerater(DataProcessor):
    """MyCobot Python API (Chain operation: *)

    Supported methods:

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

    def __init__(self, debug=False):
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

        real_command = self._flatten(command)
        has_reply = kwargs.get("has_reply", False)

        return real_command, has_reply

    # System status
    def version(self):  # TODO: test method <11-03-21, yourname> #
        """Get cobot version

        Return:
            mycobot   : 1
            mycobotPro: 101
        """
        recv = self._mesg(Command.VERSION, has_reply=True)
        return recv

    # Overall status
    def power_on(self):
        return self._mesg(Command.POWER_ON)

    def power_off(self):
        return self._mesg(Command.POWER_OFF)

    def is_power_on(self):
        """Adjust robot arm status

        Return:
            1 : power on
            0 : power off
            -1: error data
        """
        return self._mesg(Command.IS_POWER_ON, has_reply=True)

    def release_all_servos(self):
        return self._mesg(Command.RELEASE_ALL_SERVOS)

    def is_controller_connected(self):
        return self._mesg(Command.IS_CONTROLLER_CONNECTED, has_reply=True)

    """
    def set_free_mode(self, flag):  # TODO:no finish
        if flag:
            self._mesg(Command.SET_FREE_MODE, 1)
        else:
            self._mesg(Command.SET_FREE_MODE, 0)

    def is_free_mode(self):  # TODO: no finish
        return self._process_single(self._mesg(Command.IS_FREE_MODE, has_reply=True))
    """

    # MDI mode and operation
    def get_angles(self):
        """Get all angle return a list

        Return:
            data_list (list[angle...]):
        """
        return self._mesg(Command.GET_ANGLES, has_reply=True)

    def send_angle(self, id, degree, speed):
        """Send one angle

        Args:
            id (common.Angle):
            degree (float):
            speed (int): 0 ~100
        """
        check_datas(joint_id=id, degree=degree, speed=speed)
        return self._mesg(
            Command.SEND_ANGLE, id - 1, [self._angle_to_int(degree)], speed
        )

    # @check_parameters(Command.SEND_ANGLES)
    def send_angles(self, degrees, speed):
        """Send all angles

        Args:
            degrees (list): example [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            speed (int): 0 ~ 100
        """
        check_datas(degrees=degrees, len6=degrees, speed=speed)
        degrees = [self._angle_to_int(degree) for degree in degrees]
        # data = [degrees, speed]
        return self._mesg(Command.SEND_ANGLES, degrees, speed)

    def get_coords(self):
        """Get all coords.

        Return:
            data_list (list): [x, y, z, rx, ry, rz]
        """
        return self._mesg(Command.GET_COORDS, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one coord

        Args:
            id(common.Coord):
            coord(float): mm
            speed(int):
        """
        check_datas(speed=speed)
        return self._mesg(
            Command.SEND_COORD, id - 1, [self._coord_to_int(coord)], speed
        )

    def send_coords(self, coords, speed, mode):
        """Send all coords

        Args:
            coords: [x(mm), y, z, rx(angle), ry, rz]
            speed(int);
            mode(int): 0 - normal, 1 - angluar, 2 - linear
        """
        check_datas(len6=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord_to_int(coords[idx]))
        for idx in range(3, 6):
            coord_list.append(self._angle_to_int(coords[idx]))
        return self._mesg(Command.SEND_COORDS, coord_list, speed, mode)

    def pause(self):
        return self._mesg(Command.PAUSE)

    def is_paused(self):
        return self._mesg(Command.IS_PAUSED, has_reply=True)

    def resume(self):
        return self._mesg(Command.RESUME)

    def stop(self):
        return self._mesg(Command.STOP)

    def is_in_position(self, data, id):
        """

        Args:
            id: 1 - coords, 0 - angles

        Return:
            0 : error position
            1 : right position
            -1: error data
        """
        check_datas(len6=data)
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

        return self._mesg(Command.IS_IN_POSITION, data_list, id, has_reply=True)

    def is_moving(self):
        """

        Return:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._mesg(Command.IS_MOVING, has_reply=True)

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Joint control

        Args:
            joint_id: string
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        return self._mesg(Command.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Coord control

        Args:
            coord: string
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        return self._mesg(Command.JOG_COORD, coord_id, direction, speed)

    def jog_stop(self):
        return self._mesg(Command.JOG_STOP)

    def set_encoder(self, joint_id, encoder):
        """Set joint encoder value.

        Args:
            joint_id: Joint id 1 - 7
            encoder: The value of the set encoder.
        """
        return self._mesg(Command.SET_ENCODER, joint_id - 1, [encoder])

    def get_encoder(self, joint_id):
        return self._mesg(Command.GET_ENCODER, joint_id - 1, has_reply=True)

    def set_encoders(self, encoders, sp):
        return self._mesg(Command.SET_ENCODERS, encoders, sp)

    # Running status and Settings
    def get_speed(self):
        return self._mesg(Command.GET_SPEED, has_reply=True)

    def set_speed(self, speed):
        """Set speed value

        Args:
            speed (int): 0 - 100
        """
        check_datas(speed=speed)
        return self._mesg(Command.SET_SPEED, speed)

    """
    def get_feed_override(self):
        return self._process_single(
            self._mesg(Command.GET_FEED_OVERRIDE, has_reply=True)
        )

    def get_acceleration(self):
        return self._process_single(
            self._mesg(Command.GET_ACCELERATION, has_reply=True)
        )
    """

    def get_joint_min_angle(self, joint_id):
        check_datas(joint_id=joint_id)
        return self._mesg(Command.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        check_datas(joint_id=joint_id)
        return self._mesg(Command.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    # Servo control
    def is_servo_enable(self, servo_id):
        return self._mesg(Command.IS_SERVO_ENABLE, servo_id - 1)

    def is_all_servo_enable(self):
        return self._mesg(Command.IS_ALL_SERVO_ENABLE, has_reply=True)

    def set_servo_data(self, servo_no, data_id, value):
        return self._mesg(Command.SET_SERVO_DATA, servo_no - 1, data_id, value)

    def get_servo_data(self, servo_no, data_id):
        return self._mesg(Command.GET_SERVO_DATA, servo_no - 1, data_id, has_reply=True)

    def set_servo_calibration(self, servo_no):
        return self._mesg(Command.SET_SERVO_CALIBRATION, servo_no - 1)

    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: 1 ~ 6
        """
        return self._mesg(Command.RELEASE_SERVO, servo_id)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: 1 ~ 6

        """
        return self._mesg(Command.FOCUS_SERVO, servo_id)

    # Atom IO
    def set_color(self, r=0, g=0, b=0):
        """Set the light color

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        check_datas(rgb=[r, g, b])
        return self._mesg(Command.SET_COLOR, r, g, b)

    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int):
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        return self._mesg(Command.SET_PIN_MODE, pin_no, pin_mode)

    def set_digital_output(self, pin_no, pin_signal):
        """

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        return self._mesg(Command.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        return self._mesg(Command.GET_DIGITAL_INPUT, pin_no, has_reply=True)

    """
    def set_pwm_mode(self, pin_no, channel):
        self._mesg(Command.SET_PWM_MODE, pin_no, channel)
    """

    def set_pwm_output(self, channel, frequency, pin_val):
        return self._mesg(Command.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    def get_gripper_value(self):
        return self._mesg(Command.GET_GRIPPER_VALUE, has_reply=True)

    def set_gripper_state(self, flag, speed):
        """Set gripper switch

        Args:
            flag  (int): 0 - open, 1 - close
            speed (int): 0 ~ 100
        """
        return self._mesg(Command.SET_GRIPPER_STATE, flag, speed)

    def set_gripper_value(self, value, speed):
        """Set gripper value

        Args:
            value (int): 0 ~ 4096
            speed (int): 0 ~ 100
        """
        check_datas(speed=speed)
        return self._mesg(Command.SET_GRIPPER_VALUE, [value], speed)

    def set_gripper_ini(self):
        """Set the current position to zero

        Current position value is `2048`.
        """
        return self._mesg(Command.SET_GRIPPER_INI)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._mesg(Command.IS_GRIPPER_MOVING, has_reply=True)

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """

        Args:
            pin_signal: 0 / 1
        """
        return self._mesg(Command.SET_BASIC_OUTPUT, pin_no, pin_signal)
