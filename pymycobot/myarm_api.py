# coding=utf-8

from __future__ import division

import sys
import logging
import threading
import time

from pymycobot.common import ProtocolCode, write, read, DataProcessor
from pymycobot.error import calibration_parameters
from pymycobot.log import setup_logging
import serial


class MyArmAPI(DataProcessor):
    """

    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyArmAPI, self).__init__()
        self.calibration_parameters = calibration_parameters
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()
        self._version = sys.version_info[:2][0]
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)

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
        time.sleep(0.01)
        real_command, has_reply = super(MyArmAPI, self)._mesg(genre, *args, **kwargs)
        command = self._flatten(real_command)
        with self.lock:
            self._write(command)

            if not has_reply:
                return None

            timeout = kwargs.get('timeout', None)
            data = self._read(genre, command=command, timeout=timeout, _class=self.__class__.__name__)
            res = self._process_received(data, genre, arm=8)

            if len(res) == 0:
                return None

            return_single_genres = [
                ProtocolCode.GET_SERVO_MOTOR_COUNTER_CLOCKWISE,
                ProtocolCode.IS_MOVING,
                ProtocolCode.IS_POWER_ON,
                ProtocolCode.GET_ROBOT_MODIFIED_VERSION,
                ProtocolCode.GET_ROBOT_ATOM_MODIFIED_VERSION,
                ProtocolCode.GET_ROBOT_ERROR_CHECK_STATE,
                ProtocolCode.POWER_OFF,
                ProtocolCode.GET_RECV_QUEUE_SIZE,
                ProtocolCode.GET_RECV_QUEUE_LENGTH,
                ProtocolCode.COBOTX_GET_ANGLE,
                ProtocolCode.GET_ENCODER,
                ProtocolCode.SERVO_RESTORE,
                ProtocolCode.GET_ERROR_DETECT_MODE,
                ProtocolCode.GET_SERVO_MOTOR_CLOCKWISE,
                ProtocolCode.GET_SERVO_MOTOR_CONFIG,
                ProtocolCode.POWER_ON,
                ProtocolCode.GET_MASTER_PIN_STATUS,
                ProtocolCode.GET_ATOM_PIN_STATUS,
                ProtocolCode.GET_SERVO_D
            ]

            if genre in return_single_genres:
                result = self._process_single(res)
                if genre in (ProtocolCode.COBOTX_GET_ANGLE,):
                    result = self._int2angle(result)
                return result
            if genre in (ProtocolCode.GET_ATOM_PRESS_STATUS, ):
                if self.__class__.__name__ == 'MyArmM':
                    return res[0]
                return res
            elif genre in [
                ProtocolCode.GET_ANGLES, ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE,
                ProtocolCode.GET_JOINTS_COORD
            ]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_SERVO_VOLTAGES]:
                return [self._int2coord(angle) for angle in res]
            elif genre == ProtocolCode.GET_ROBOT_ERROR_STATUS:
                result = []
                for item in res:
                    if item == 0:
                        result.append(0)
                        continue

                    reverse_bins = reversed(bin(item)[2:])
                    rank = [i for i, e in enumerate(reverse_bins) if e != '0']
                    result.append(rank)
                return result
            elif genre in (ProtocolCode.GET_ROBOT_FIRMWARE_VERSION, ProtocolCode.GET_ROBOT_TOOL_FIRMWARE_VERSION):
                return self._int2coord(res[0])
            else:
                return res

    def get_robot_modified_version(self):
        """Get the bot correction version number

        Returns:
                version (int): the bot correction version number
        """
        return self._mesg(ProtocolCode.GET_ROBOT_MODIFIED_VERSION, has_reply=True)

    def get_robot_firmware_version(self):
        """Obtaining the Robot Firmware Version (Major and Minor Versions)

        Returns:
                version (int): the robot firmware
        """
        return self._mesg(ProtocolCode.GET_ROBOT_FIRMWARE_VERSION, has_reply=True)

    def get_robot_tool_modified_version(self):
        """Get the remediation version of the bot tool"""
        return self._mesg(ProtocolCode.GET_ROBOT_ATOM_MODIFIED_VERSION, has_reply=True)

    def get_robot_tool_firmware_version(self):
        """Get the Robot Tool Firmware Version (End Atom)"""
        return self._mesg(ProtocolCode.GET_ROBOT_TOOL_FIRMWARE_VERSION, has_reply=True)

    def set_robot_err_check_state(self, status):
        """Set Error Detection Status You can turn off error detection, but do not turn it off unless necessary

        Args:
            status (int): 1 open; o close
        """
        self._mesg(ProtocolCode.SET_ROBOT_ERROR_CHECK_STATE, status)

    def get_robot_err_check_state(self):
        """Read error detection status"""
        return self._mesg(ProtocolCode.GET_ROBOT_ERROR_CHECK_STATE, has_reply=True)

    def get_robot_error_status(self):
        """Get the bot error status

        Returns:
            No error: [0,0,0,0,0,0,0,0]
            assuming that error 1 and 3 are reported in section 1, it should return: [[1,3],0,0,0,0,0,0,0]
        """
        return self._mesg(ProtocolCode.GET_ROBOT_ERROR_STATUS, has_reply=True, timeout=90)

    def get_recv_queue_max_len(self):
        """The total length of the read command queue, the default length is 100"""
        return self._mesg(ProtocolCode.GET_RECV_QUEUE_SIZE, has_reply=True)

    def set_recv_queue_max_len(self, max_len):
        """Set the total length of the receiving command queue"""
        assert 0 < max_len <= 100, "queue size must be in range 1 - 100"
        self._mesg(ProtocolCode.SET_RECV_QUEUE_SIZE, max_len)

    def get_recv_queue_len(self):
        """The current length of the read receive queue"""
        return self._mesg(ProtocolCode.GET_RECV_QUEUE_LENGTH, has_reply=True)

    def get_joint_angle(self, joint_id):
        """
        Obtain the current angle of the specified joint, when the obtained angle is 200��,
        it means that the joint has no communication
        Args:
            joint_id (int): 0 - 254

        Returns:

        """
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id, has_reply=True)

    def get_joints_angle(self):
        """Obtain the current angle of all joints, if one of the angles is 200��,
        it means that the joint has no communication

        Returns:
            angles list(int): 0 - 254
        """
        return self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)

    def set_servo_calibrate(self, servo_id):
        """Sets the zero position of the specified servo motor

        Args:
            servo_id (int): 0 - 254
        """

        self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def get_servo_encoder(self, servo_id):
        """Gets the current encoder potential value for the specified servo motor

        Args:
            servo_id (int): 0 - 254

        Returns:
            encoder (int): 0-4095
        """
        return self._mesg(ProtocolCode.GET_ENCODER, servo_id, has_reply=True)

    def get_servos_encoder_drag(self):
        """ Reads the current encoder value and operating speed of all servo motors
        Returns:
            encoders (list[int * 8]): 0-4095
            speeds (list[int * 8]): 0-4095
        """
        return self._mesg(ProtocolCode.GET_SERVOS_ENCODER_DRAG, has_reply=True)

    def is_all_servos_enabled(self):
        """
        Get the connection status of multiple servo motors

        Returns:
            status: list[int*8] 0: The connection failed 1: The connection is successful
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE, has_reply=True)

    def get_servos_temp(self):
        """Obtain the temperature of multiple servo motors"""
        return self._mesg(ProtocolCode.GET_SERVO_TEMPS, has_reply=True)

    def get_servos_voltage(self):
        """Get the voltage of multiple servo motors"""
        return self._mesg(ProtocolCode.GET_SERVO_VOLTAGES, has_reply=True)

    def get_servos_current(self):
        """Obtain the current of multiple servo motors"""
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS, has_reply=True)

    def get_servos_status(self):
        """Get all the statuses of multiple servo motors"""
        return self._mesg(ProtocolCode.GET_SERVO_STATUS, has_reply=True)

    def get_servos_protect_current(self):
        """Obtain multiple servo motor protection currents"""
        return self._mesg(ProtocolCode.GET_SERVO_LASTPDI, has_reply=True)

    def get_servos_encoder(self):
        """Obtain the current encoder potential values for multiple servo motors"""
        return self._mesg(ProtocolCode.GET_ENCODERS, has_reply=True)

    def get_servos_speed(self):
        """Gets the current movement speed of multiple servo motors"""
        return self._mesg(ProtocolCode.GET_SERVO_SPEED, has_reply=True)

    def set_servo_p(self, servo_id, data):
        """Sets the proportionality factor of the position loop P of the specified servo motor

        Args:
            servo_id (int): 0-254
            data (int): 0-254

        """
        assert 0 <= data <= 254, "data must be between 0 and 254"
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id)
        self._mesg(ProtocolCode.SET_SERVO_P, servo_id, data)

    def get_servo_p(self, servo_id):
        """Reads the position loop P scale factor of the specified servo motor

        Args:
            servo_id (int): 0-254
        """
        return self._mesg(ProtocolCode.GET_SERVO_P, servo_id, has_reply=True)

    def set_servo_d(self, servo_id, data):
        """Sets the proportional factor for the position ring D of the specified servo motor

        Args:
            servo_id (int): 0-254
            data (int): 0-254
        """
        assert 0 <= data <= 254, "data must be between 0 and 254"
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id)
        self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE, servo_id, data)

    def get_servo_d(self, servo_id):
        """Reads the position ring D scale factor for the specified servo motor

        Args:
            servo_id (int): 0-254
        """
        return self._mesg(ProtocolCode.GET_SERVO_D, servo_id, has_reply=True)

    def set_servo_i(self, servo_id, data):
        """Set the proportional factor of the position ring I of the specified servo motor

        Args:
            servo_id (int): 0 - 254
            data (int): 0 - 254
        """
        assert 0 <= data <= 254, "data must be between 0 and 254"
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id)
        self._mesg(ProtocolCode.MERCURY_DRAG_TECH_PAUSE, servo_id, data)

    def get_servo_i(self, servo_id):
        """Reads the position loop I scale factor of the specified servo motor"""
        return self._mesg(ProtocolCode.GET_ERROR_DETECT_MODE, servo_id, has_reply=True)

    def set_servo_cw(self, servo_id, data):
        """Sets the clockwise insensitivity zone of the encoder for the specified servo motor

        Args:
            servo_id (int): 0 - 254
            data (int): 0 - 32
        """
        assert 0 <= data <= 32, "data must be between 0 and 32"
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id)
        self._mesg(ProtocolCode.SET_SERVO_MOTOR_CLOCKWISE, servo_id, data)

    def get_servo_cw(self, servo_id):
        """Reads the clockwise insensitive area of the encoder for the specified servo motor

        Args:
            servo_id (int): 0 - 254
        """
        return self._mesg(ProtocolCode.GET_SERVO_MOTOR_CLOCKWISE, servo_id, has_reply=True)

    def set_servo_cww(self, servo_id, data):
        """Sets the counterclockwise insensitive zone of the encoder for the specified servo motor

        Args:
            servo_id (int): 0 - 254
            data (int): 0 - 32
        """
        assert 0 <= data <= 32, "data must be between 0 and 32"
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_MOTOR_COUNTER_CLOCKWISE, servo_id, data)

    def get_servo_cww(self, servo_id):
        """Reads the counterclockwise insensitive area of the encoder of the specified servo motor

        Args:
            servo_id (int): 0 - 254
        """
        return self._mesg(ProtocolCode.GET_SERVO_MOTOR_COUNTER_CLOCKWISE, servo_id, has_reply=True)

    def set_servo_system_data(self, servo_id, addr, data, mode):
        """Set the system parameters for the specified servo motor

        Args:
            servo_id (int): 0 - 254:
            addr (int):
            data (int): 0 - 4096
            mode (int): 1 - data 1byte. 2 - data 2byte
        """
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=servo_id, servo_addr=addr)
        self._mesg(ProtocolCode.SET_SERVO_MOTOR_CONFIG, servo_id, addr, [data], mode)

    def get_servo_system_data(self, servo_id, addr, mode):
        """Read the system parameters of the specified servo motor

        Args:
            servo_id (int): 0 - 254
            addr (int):
            mode (int): 1 - data 1byte. 2 - data 2byte
        """
        return self._mesg(ProtocolCode.GET_SERVO_MOTOR_CONFIG, servo_id, addr, mode, has_reply=True)

    def set_master_out_io_state(self, io_number, status=1):
        """Set the host I/O pin status

        Args:
            io_number: 1 - 2
            status: 0/1; 0: low; 1: high. default: 1

        """
        self._mesg(ProtocolCode.SET_MASTER_PIN_STATUS, io_number, status)

    def get_master_in_io_state(self, io_number):
        """get the host I/O pin status

        Args:
            io_number (int): 1 - 2

        Returns:
                0 or 1. 1: high 0: low
        """
        return self._mesg(ProtocolCode.GET_MASTER_PIN_STATUS, io_number, has_reply=True)

    def set_tool_out_io_state(self, io_number, status=1):
        """Set the Atom pin status

        Args:
            io_number (int): 1 - 2
            status: 0 or 1; 0: low; 1: high. default: 1

        """
        self._mesg(ProtocolCode.SET_ATOM_PIN_STATUS, io_number, status)

    def get_tool_in_io_state(self, io_number):
        """Get the Atom pin status

        Args:
            io_number (int): pin number

        Returns:
            0 or 1. 1: high 0: low

        """
        return self._mesg(ProtocolCode.GET_ATOM_PIN_STATUS, io_number, has_reply=True)

    def set_tool_led_color(self, r, g, b):
        """Set the Atom LED color

        Args:
            r: 0-255
            g: 0-255
            b: 0-255

        """
        self._mesg(ProtocolCode.GET_ATOM_LED_COLOR, r, g, b)

    def restore_servo_system_param(self):
        """Restore servo motor system parameters"""
        self._mesg(ProtocolCode.RESTORE_SERVO_SYSTEM_PARAM)
