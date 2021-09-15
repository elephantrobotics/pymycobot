# coding=utf-8

import sys
import logging

from pymycobot.log import setup_logging
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode, DataProcessor


class MyCobotCommandGenerator(DataProcessor):
    """MyCobot Python API
    (* = Chain operation)
    (x = ???)

    Supported methods:

        # Overall status
            version()
            power_on()
            power_off()
            is_power_on()
            release_all_servos()
            is_controller_connected()

        # MDI mode and operation
            get_angles()
            send_angle()
            send_angles()
            get_coords()
            send_coords()
            is_in_position()
            is_moving() x

        # JOG mode and operation
            jog_angle()
            jog_coord()
            jog_stop()
            set_encoder()
            get_encoder()
            set_encoders()
            pause()
            resume()
            stop()
            is_paused()

        # Running status and Settings
            get_encoder()
            set_encoder()
            get_encoders()
            set_encoders()
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
            set_basic_output()
            get_basic_input()
    """

    def __init__(self, debug=False):
        """
        Args:
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
            ProtocolCode.HEADER,
            ProtocolCode.HEADER,
            LEN,
            genre,
            command_data,
            ProtocolCode.FOOTER,
        ]

        real_command = self._flatten(command)
        has_reply = kwargs.get("has_reply", False)

        return real_command, has_reply

    # System status
    def version(self):  # TODO: test method <2021-03-11, yourname> #
        """Get cobot version

        Return:
            mycobot   : 1
            mycobotPro: 101
        """
        return self._mesg(ProtocolCode.VERSION, has_reply=True)

    # Overall status
    def power_on(self):
        """Open communication with Atom."""
        return self._mesg(ProtocolCode.POWER_ON)

    def power_off(self):
        """Close communication with Atom."""
        return self._mesg(ProtocolCode.POWER_OFF)

    def is_power_on(self):
        """Adjust robot arm status

        Return:
            1 : power on
            0 : power off
            -1: error data
        """
        return self._mesg(ProtocolCode.IS_POWER_ON, has_reply=True)

    def release_all_servos(self):
        return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS)

    def is_controller_connected(self):
        """Wether connected with Atom."""
        return self._mesg(ProtocolCode.IS_CONTROLLER_CONNECTED, has_reply=True)

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
        """Get all angle return a list.

        Return:
            data_list (list[angle...]):
        """
        return self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)

    def send_angle(self, id, degree, speed):
        """Send one angle.

        Args:
            id (common.Angle/int): Joint number.
            degree (float): angle value.
            speed (int): 0 ~ 100
        """
        self.calibration_parameters(id=id, degree=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed)

    # @check_parameters(Command.SEND_ANGLES)
    def send_angles(self, degrees, speed):
        """Send all angles

        Args:
            degrees (list): example [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            speed (int): 0 ~ 100
        """
        self.calibration_parameters(degrees=degrees, speed=speed)
        degrees = [self._angle2int(degree) for degree in degrees]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def get_coords(self):
        """Get all coords.

        Return:
            data_list (list): [x, y, z, rx, ry, rz]
        """
        return self._mesg(ProtocolCode.GET_COORDS, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one position of coordination.

        Args:
            id(common.Coord/int): coordination number.
            coord(float): mm
            speed(int): 0 ~ 100
        """
        self.calibration_parameters(id=id, speed=speed)
        value = self._coord2int(coord) if id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, id, [value], speed)

    def send_coords(self, coords, speed, mode):
        """Send all coordinations.

        Args:
            coords: [x(mm), y, z, rx(angle), ry, rz]
            speed(int);
            mode(int): 0 - normal, 1 - angluar, 2 - linear
        """
        self.calibration_parameters(coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, mode)

    def is_in_position(self, data, id=0):
        """Determine whether the given position is reached.

        Args:
            id: 1 - coords, 0 - angles
            data: The value list of angles or coordinations.

        Return:
            0 : error position
            1 : right position
            -1: error data
        """
        if id == 1:
            self.calibration_parameters(coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            self.calibration_parameters(degrees=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, id, has_reply=True)

    def is_moving(self):
        """

        Return:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._mesg(ProtocolCode.IS_MOVING, has_reply=True)

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Control joint in JOG mode.

        Args:
            joint_id(int): Joint number.
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Control coordination in JOG mode.

        Args:
            coord: string
            direction: int [0, 1]
            speed: int (0 - 100)
        """
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_stop(self):
        """Stop JOG movement."""
        return self._mesg(ProtocolCode.JOG_STOP)

    def pause(self):
        return self._mesg(ProtocolCode.PAUSE)

    def is_paused(self):
        return self._mesg(ProtocolCode.IS_PAUSED, has_reply=True)

    def resume(self):
        return self._mesg(ProtocolCode.RESUME)

    def stop(self):
        return self._mesg(ProtocolCode.STOP)

    def set_encoder(self, joint_id, encoder):
        """Set joint encoder value.

        Args:
            joint_id: Joint id 1 - 7
            encoder: The value of the set encoder.
        """
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder])

    def get_encoder(self, joint_id):
        """Get servo encoder

        Args:
            joint_id (int): servo number.

        Returns:
            (int16): servo encoder value.
        """
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id, has_reply=True)

    def set_encoders(self, encoders, sp):
        """Set all servo encoder.

        Args:
            encoders (list): encoder list
            sp (int): speed

        Returns:
            (str): command.
        """
        return self._mesg(ProtocolCode.SET_ENCODERS, encoders, sp)

    def get_encoders(self):
        return self._mesg(ProtocolCode.GET_ENCODERS, has_reply=True)

    # Running status and Settings
    def get_speed(self):
        return self._mesg(ProtocolCode.GET_SPEED, has_reply=True)

    def set_speed(self, speed):
        """Set speed value

        Args:
            speed (int): 0 - 100
        """
        self.calibration_parameters(speed=speed)
        return self._mesg(ProtocolCode.SET_SPEED, speed)

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
        self.calibration_parameters(id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        self.calibration_parameters(id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    # Servo control
    def is_servo_enable(self, servo_id):
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id, has_reply=True)

    def is_all_servo_enable(self):
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE, has_reply=True)

    def set_servo_data(self, servo_no, data_id, value):
        return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_no, data_id, value)

    def get_servo_data(self, servo_no, data_id):
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, servo_no, data_id, has_reply=True
        )

    def set_servo_calibration(self, servo_no):
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_no)

    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: 1 ~ 6
        """
        return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: 1 ~ 6

        """
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    # Atom IO
    def set_color(self, r=0, g=0, b=0):
        """Set the light color

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int):
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        return self._mesg(ProtocolCode.SET_PIN_MODE, pin_no, pin_mode)

    def set_digital_output(self, pin_no, pin_signal):
        """

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no, has_reply=True)

    """
    def set_pwm_mode(self, pin_no, channel):
        self._mesg(Command.SET_PWM_MODE, pin_no, channel)
    """

    def set_pwm_output(self, channel, frequency, pin_val):
        return self._mesg(ProtocolCode.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    def get_gripper_value(self):
        """Get the value of gripper."""
        return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, has_reply=True)

    def set_gripper_state(self, flag, speed):
        """Set gripper switch

        Args:
            flag  (int): 0 - open, 1 - close
            speed (int): 0 ~ 100
        """
        return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)

    def set_gripper_value(self, value, speed):
        """Set gripper value

        Args:
            value (int): 0 ~ 100
            speed (int): 0 ~ 100
        """
        self.calibration_parameters(speed=speed)
        return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, value, speed)

    def set_gripper_ini(self):
        """Set the current position to zero

        Current position value is `2048`.
        """
        return self._mesg(ProtocolCode.SET_GRIPPER_INI)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 : not moving
            1 : is moving
            -1: error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output for M5 version.

        Args:
            pin_no: pin port number.
            pin_signal: 0 / 1
        """
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_basic_input(self, pin_no):
        """Get basic input for M5 version.

        Args:
            pin_no: pin port number.
        """
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no, has_reply=True)
