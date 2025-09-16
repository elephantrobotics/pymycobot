#!/usr/bin/python

from socket import socket, AF_INET, SOCK_STREAM
import sys
import time
from enum import Enum
import base64
import hashlib
import math
from multiprocessing import Lock
import logging
import numpy as np
from pymycobot.log import setup_logging
from pymycobot.tool_coords import *
from pymycobot.error import MyCobot630ProDataException
import time
from pymycobot.pro630common import Axis, Joint, DI, DO, AI, AO


COORDS_EPSILON = 0.50


class JogMode(Enum):
    JOG_JOINT = 0
    JOG_TELEOP = 1


mutex = Lock()

def parse_mixed_list_string(s):
        s = s.strip()
        if s.startswith("[") and s.endswith("]"):
            try:
                return ast.literal_eval(s)
            except (ValueError, SyntaxError):
                return None
        else:
            try:
                return [int(x.strip()) for x in s.split(",")]
            except ValueError:
                return None


class ElephantRobot(object):
    def __init__(self, host, port, debug=False):
        """Initializes the ElephantRobot instance.

        Args:
            host (str): IP address or hostname of the robot.
            port (int): Port number for the robot's socket server.
            debug (bool): Enables debug mode if True.
        """
        try:
            import numpy as np
        except ImportError:
            raise ImportError("Please install numpy")
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self.BUFFSIZE = 8 * 1024 * 1024
        self.ADDR = (host, port)
        self.tcp_client = socket(AF_INET, SOCK_STREAM)
        self.is_client_started = False
        tool_reference = np.array([0, 0, 0, 0, 0, 0])
        self.tool_matrix = self.set_tool_reference(tool_reference)

    def get_ip(self):
        """Returns tuple of (IP address or hostname, port) of robot with socket server.

        Returns:
            str: IP address or hostname
        """
        return self.ADDR

    def set_ip(self, host, port):
        """Sets IP address or hostname and port of robot with socket server.
        Cannot set or change IP address or hostname or port of already started client.
        Need to stop client before setting IP address/hostname/port.

        Args:
            host (str): IP address or hostname
            port (int): port

        Returns:
            bool: True if success, False otherwise
        """
        if self.is_client_started:
            print(
                "Error: Cannot change IP if client is started. Stop client to change IP/Host/Port"
            )
            return False
        self.ADDR = (host, port)
        return True

    def start_client(self):
        """Starts the TCP client connection to the robot.

        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            self.tcp_client.connect(self.ADDR)
            self.is_client_started = True
            return True
        except Exception as e:
            print(e)
            return False

    def stop_client(self):
        """Stops the TCP client connection."""
        self.tcp_client.close()
        self.is_client_started = False

    def send_command(self, command):
        """Sends a command to the robot and receives the response.

        Args:
            command (str): Command string to send.

        Returns:
            str: Response from the robot.
        """

        with mutex:
            if not command.endswith('\n'):
                command += '\n'

            self.tcp_client.send(command.encode())

            self.tcp_client.settimeout(10.0)  # Set a 10 second timeout
            try:
                recv_data = self.tcp_client.recv(self.BUFFSIZE).decode()
            except socket.timeout:
                print("Waiting for server response timeout")
                return ""

            res_str = str(recv_data)
            if self.debug:
                print("recv = " + res_str)
            res_arr = res_str.split(":")
            if len(res_arr) == 2:
                return res_arr[1]
            else:
                return ""
    def string_to_coords(self, data):
        """Converts a string representation of coordinates to a list.

        Args:
            data (str): String representation of coordinates.

        Returns:
            list: List of coordinates or invalid coordinates if conversion fails.
        """
        data = data.replace("[", "")
        data = data.replace("]", "")
        data_arr = data.split(",")
        if len(data_arr) == 6:
            try:
                coords_1 = round(float(data_arr[0]), 3)
                coords_2 = round(float(data_arr[1]), 3)
                coords_3 = round(float(data_arr[2]), 3)
                coords_4 = round(float(data_arr[3]), 3)
                coords_5 = round(float(data_arr[4]), 3)
                coords_6 = round(float(data_arr[5]), 3)
                coords = [coords_1, coords_2, coords_3, coords_4, coords_5, coords_6]
                return coords
            except:
                return self.invalid_coords()
        return self.invalid_coords()

    def string_to_double(self, data):
        """Converts a string to a double value.

        Args:
            data (str): String representation of a number.

        Returns:
            float: Converted double value or -9999.99 if conversion fails.
        """
        try:
            val = round(float(data), 3)
            return val
        except:
            return -9999.99

    def float_equal(self, a, b, epsilon=COORDS_EPSILON):
        """Compares 2 floats with given epsilon precision.

        Args:
            a (float): 1st float to compare
            b (float): 2nd float to compare
            epsilon (float): precision (epsilon) to compare

        Returns:
            bool: True if floats are considered equal with given precision,
                  False otherwise
        """
        return math.fabs(a - b) < epsilon

    def coords_equal(self, coords_1, coords_2):
        """Checks if specified coords are equal.

        Args:
            coords_1 (list): first coords to compare
            coords_2 (list): second coords to compare

        Returns:
            bool: True if coords are equal, False otherwise.
        """
        return (
            self.float_equal(coords_1[Axis.X.value], coords_2[Axis.X.value])
            and self.float_equal(coords_1[Axis.Y.value], coords_2[Axis.Y.value])
            and self.float_equal(coords_1[Axis.Z.value], coords_2[Axis.Z.value])
            and self.float_equal(coords_1[Axis.RX.value], coords_2[Axis.RX.value])
            and self.float_equal(coords_1[Axis.RY.value], coords_2[Axis.RY.value])
            and self.float_equal(coords_1[Axis.RZ.value], coords_2[Axis.RZ.value])
        )

    def angles_equal(self, angles_1, angles_2):
        """Checks if specified angles are equal.

        Args:
            angles_1 (list): first angles to compare
            angles_2 (list): second angles to compare

        Returns:
            bool: True if angles are equal, False otherwise.
        """
        return (
            self.float_equal(angles_1[Joint.J1.value], angles_2[Joint.J1.value])
            and self.float_equal(angles_1[Joint.J2.value], angles_2[Joint.J2.value])
            and self.float_equal(angles_1[Joint.J3.value], angles_2[Joint.J3.value])
            and self.float_equal(angles_1[Joint.J4.value], angles_2[Joint.J4.value])
            and self.float_equal(angles_1[Joint.J5.value], angles_2[Joint.J5.value])
            and self.float_equal(angles_1[Joint.J6.value], angles_2[Joint.J6.value])
        )

    def string_to_int(self, data):
        try:
            val = int(data)
            return val
        except:
            return -9999

    def invalid_coords(self):
        """Returns a predefined set of invalid coordinates.

        Returns:
            list: Invalid coordinates.
        """
        coords = [-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]
        return coords

    def detect_robot(self):
        """Detects the robot's model.

        Returns:
            int: Integer value indicating robot model.
        """
        return int(self.get_analog_in(55))

    def start_robot(self):
        """Starts robot.

        Returns:
            bool: currently always returns True
        """
        command = "start_robot()\n"
        res = self.send_command(command)
        return True

    def recover_robot(self):
        """Recovers robot after collision.

        Returns:
            bool: currently always returns True
        """
        return self.start_robot()

    def get_angles(self):
        """Retrieves the current joint angles of the robot.

        Returns:
            list: List of joint angles.
        """
        command = "get_angles()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_angle(self, joint: Joint) -> float:
        """Retrieves the angle of a specific joint.

        Args:
            joint (Joint): specific joint.

        Example:
            >>> e.get_angle(Joint.J1)
            >>> 1.234
            >>> e.get_angle(Joint.J2)
            >>> -92.345
            >>> e.get_angle(Joint.J3)
            >>> 3.456
            >>> e.get_angle(Joint.J4)
            >>> -94.567
            >>> e.get_angle(Joint.J5)
            >>> 5.678
            >>> e.get_angle(Joint.J6)
            >>> 6.789

        Returns:
            float: Angle of the specified joint.
        """
        command = "get_angle(" + str(joint) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def get_coords(self):
        """Retrieves the current coordinates of the robot.

        Returns:
            list: List of coordinates.
        """
        command = "get_coords()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_coord(self, axis: Axis) -> float:
        """Retrieves the value of a specific coordinate axis.

        Args:
            axis (Axis): specified Axis.

        Example:
            >>> e.get_coord(Axis.X)
            >>> 0.0
            >>> e.get_coord(Axis.Y)
            >>> 140.0
            >>> e.get_coord(Axis.Z)
            >>> 835.0
            >>> e.get_coord(Axis.RX)
            >>> 90.0
            >>> e.get_coord(Axis.RY)
            >>> 0.0
            >>> e.get_coord(Axis.RZ)
            >>> 180.0

        Returns:
            float: Value of the specified axis.
        """
        command = "get_coord(" + str(axis) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def get_speed(self):
        """Retrieves the current speed of the robot.

        Returns:
            float: Speed value.
        """
        command = "get_speed()\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def _power_on(self):
        command = "power_on()\n"
        res = self.send_command(command)
        return True

    def _power_off(self):
        command = "power_off()\n"
        res = self.send_command(command)
        return True

    def is_power_on(self):
        """Checks whether robot is powered on.

        Returns:
            bool: True if robot is powered on, False otherwise.
        """
        command = "is_power_on()\n"
        res = self.send_command(command)
        return res == "1"

    def _state_on(self):
        command = "state_on()\n"
        self.send_command(command)

    def _state_off(self):
        command = "state_off()\n"
        self.send_command(command)

    def state_check(self):
        """Checks if robot is enabled.

        Returns:
            bool: True if robot is enabled, False otherwise.
        """
        command = "state_check()\n"
        res = self.send_command(command)
        return res == "1"

    def check_running(self):
        """Checks whether robot is moving.

        Returns:
            bool: True if robot is moving, False otherwise.
        """
        command = "check_running()\n"
        res = self.send_command(command)
        return res == "1"

    def is_in_position(self, coords, jog_mode):
        """Returns True if current position equals passed coords.

        Args:
            coords (list[float]): coords or angles
            jog_mode (JogMode): JogMode enum value

        Returns:
            bool: True if robot is in passed coords, False otherwise
        """
        if type(jog_mode) is not JogMode:
            raise TypeError("jog_mode should be of type JogMode")
        if jog_mode == JogMode.JOG_TELEOP:
            return self.coords_equal(self.get_coords(), coords)
        else:
            return self.angles_equal(self.get_angles(), coords)

    def set_free_move_mode(self, on=True):
        """Enables or disables free move mode.

        Args:
            on (bool, optional): True to enable free move mode,
                                 False to disable free move mode.
                                 Defaults to True.
        """
        command = "set_free_move(" + str(int(on)) + ")\n"
        self.send_command(command)

    def set_payload(self, payload):
        """Sets payload value on the robot.

        Args:
            payload (float): payload value (0.0~2.0)
        """
        command = "set_payload(" + str(payload) + ")\n"
        self.send_command(command)

    def get_payload(self):
        """Returns current payload value.

        Returns:
            float: payload value
        """
        return self.get_analog_out(53)

    def upload_file(self, local_filename, remote_filename):
        """Upload local file to robot via socket. Permissions are checked before
           upload so it cannot overwrite system files (owned by root).

        Args:
            local_filename (str): absolute or relative path to a file to send
            remote_filename (str): absolute or relative file path to where
                                   to upload a file. If path is relative, it is
                                   relative to ~/RoboFlow/upload folder.

        Returns:
            str: ok if success or error message otherwise
        """
        with open(local_filename, "rb") as f:
            content = f.read()
        content_base64 = base64.b64encode(content).decode()
        content_sha256 = hashlib.sha256(content).hexdigest()
        command = "upload_file({},{},{})".format(
            content_base64, remote_filename, content_sha256
        )
        res = self.send_command(command)
        return res

    def read_text_file(self, remote_filename):
        """Reads the content of a text file from the robot.

        Args:
            remote_filename (str): Path to the file on the robot.

        Returns:
            str: Content of the file.
        """
        command = f"read_text_file({remote_filename})"
        res = self.send_command(command)
        return res

    def program_open(self, file_path):
        """Opens a program file on the robot. File must be a g-code file.

        Args:
            file_path (str): Path to the program (g-code) file.

        Returns:
            int: Status code indicating success or failure.
        """
        command = "program_open(" + file_path + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def program_run(self, start_line):
        """Runs a program on the robot starting from a specific line.
        Program must be opened before running using program_open() method.

        Args:
            start_line (int): Line number to start execution.

        Returns:
            int: Status code indicating success or failure.
        """
        command = "program_run(" + str(start_line) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def read_next_error(self):
        """Reads the next error message from the robot.

        Returns:
            str: Error message or an empty string if no errors.
        """
        command = "read_next_error()\n"
        res = self.send_command(command)
        return res

    def clear_all_errors(self):
        """Clears all error messages from the robot."""
        while self.read_next_error() != "":
            pass

    def get_robot_state(self) -> str:
        """Returns current robot state, which is 64 bits represented as a
        string of 64 zeroes or ones.

        Meaning of each bit from eft to right is as follows:
        Bit number | Meaning
        0          | Power on
        1          | Emergency stop button, 0 - pressed, 1 - released
        2          | Joint 6 status, 0 - error, 1 - OK
        3          | Joint 5 status, 0 - error, 1 - OK
        4          | Joint 4 status, 0 - error, 1 - OK
        5          | Joint 3 status, 0 - error, 1 - OK
        6          | Joint 2 status, 0 - error, 1 - OK
        7          | Joint 1 status, 0 - error, 1 - OK
        8          | Reserved
        9          | Reserved
        10         | Reserved
        11         | Motion enabled, 0 - disabled, 1 - enabled
        12         | Hardware Free Move
        13         | Joint 6 servo enabled, 0 - disabled, 1 - enabled
        14         | Joint 5 servo enabled, 0 - disabled, 1 - enabled
        15         | Joint 4 servo enabled, 0 - disabled, 1 - enabled
        16         | Joint 3 servo enabled, 0 - disabled, 1 - enabled
        17         | Joint 2 servo enabled, 0 - disabled, 1 - enabled
        18         | Joint 1 servo enabled, 0 - disabled, 1 - enabled
        19         | Brake activation running, 0 - not running, 1 - running
        20         | Hardware Pause Pressed, 0 - not pressed, 1 - pressed
        21         | Reserved
        22         | IO Run Triggered, 0 - not triggered, 1 - triggered
        23         | IO Stop Triggered, 0 - not triggered, 1 - triggered
        24         | Joint 6 communication status, 0 - error, 1 - OK
        25         | Joint 5 communication status, 0 - error, 1 - OK
        26         | Joint 4 communication status, 0 - error, 1 - OK
        27         | Joint 3 communication status, 0 - error, 1 - OK
        28         | Joint 2 communication status, 0 - error, 1 - OK
        29         | Joint 1 communication status, 0 - error, 1 - OK
        30         | Collision detected, 0 - not detected, 1 - detected
        31         | Reserved
        32         | Reserved
        33         | Reserved
        34         | Reserved
        35         | Reserved
        36         | Reserved
        37         | Reserved
        38         | Reserved
        39         | Reserved
        40         | Reserved
        41         | Reserved
        42         | Reserved
        43         | Reserved
        44         | Reserved
        45         | Reserved
        46         | C Series Physical User button
        47         | C Series Physical Stop button
        48         | C Series Physical Start button
        49         | Reserved
        50         | Reserved
        51         | Reserved
        52         | Reserved
        53         | Reserved
        54         | Reserved
        55         | Reserved
        56         | Reserved
        57         | Reserved
        58         | Reserved
        59         | Reserved
        60         | Reserved
        61         | Reserved
        62         | Reserved
        63         | Reserved

        For example, if robot is powered on, enabled and in position,
        the returned string will be:
        '1011111100000111111000001111110000000000000000110000000000000000'

        Returns:
            str: robot state as a string of 64 zeroes or ones.
        """
        command = "get_robot_state()\n"
        res = self.send_command(command)
        return res

    def set_servos_calibration(self) -> str:
        """Sets servos calibration, that is, current robot pose will become
        robot's zero position.

        This function will shutdown robot. You will need to restart the robot
        and reconnect to continue working with it.

        Returns:
            str: empty string if success, error message otherwise.
        """
        command = "set_servos_calibration()\n"
        res = self.send_command(command)
        return res

    def get_joint_loss_pkg(self, joint_number: Joint) -> int:
        """Retrieves the loss package count for a specific joint.

        Args:
            joint_number (Joint): The joint number for which the loss package
                                  value is requested.

        Returns:
            int: The loss package count for the specified joint.
        """
        command = "get_joint_loss_pkg(" + str(joint_number.value) + ")\n"
        res = self.send_command(command)
        return int(self.string_to_double(res))

    # TODO: rename to set_coords()
    def write_coords(self, coords, speed):
        """Sets the robot's coordinates.

        Args:
            coords (list[float]): List of coordinates [X, Y, Z, RX, RY, RZ].
            speed (int): Speed of movement.
        """
        command = "set_coords("
        for item in coords:
            command += str(round(item, 3)) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    # TODO: change to set_coord() socket API (already impemented)
    def write_coord(self, axis, value, speed):
        """Sets the value of a specific coordinate axis.

        Args:
            axis (int): Axis index from 0 to 5 (for X, Y, Z, RX, RY, RZ).
            value (float): Value to set for the axis.
            speed (int): Speed of movement.
        """
        coords = self.get_coords()
        if coords != self.invalid_coords():
            coords[axis] = value
            self.write_coords(coords, speed)

    # TODO: rename to set_angles()
    def write_angles(self, angles, speed):
        """Sets the robot's joint angles.

        Args:
            angles (list[float]): List of joint angles [J1, J2, J3, J4, J5, J6].
            speed (int): Speed of movement.
        """
        command = "set_angles("
        for item in angles:
            command += str(round(item, 3)) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    # TODO: change to set_angle() socket API (already impemented)
    def write_angle(self, joint, value, speed):
        """Sets the angle of a specific joint.

        Args:
            joint (int): Joint index from 0 to 5 (for J1, J2, J3, J4, J5, J6).
            value (float): Angle value to set.
            speed (int): Speed of movement.
        """
        angles = self.get_angles()
        if angles != self.invalid_coords():
            angles[joint] = value
            self.write_angles(angles, speed)

    def set_speed(self, percentage):
        """Sets the robot's movement speed.

        Args:
            percentage (int): Speed percentage (0-100).
        """
        command = "set_speed(" + str(percentage) + ")\n"
        self.send_command(command)

    def set_carte_torque_limit(self, axis_str, value):
        """Sets the torque limit for a specific Cartesian axis.

        Args:
            axis_str (str): Axis name (e.g., "X", "Y", "Z", "RX", "RY", "RZ").
            value (float): Torque limit value.
        """
        command = "set_torque_limit(" + axis_str + "," + str(value) + ")\n"
        self.send_command(command)

    def set_upside_down(self, up_down):
        """Sets the robot's upside-down mode.

        Args:
            up_down (bool): True to enable upside-down mode, False to disable.
        """
        up = "1"
        if up_down:
            up = "0"
        command = "set_upside_down(" + up + ")\n"
        self.send_command(command)

    def task_stop(self):
        """Stops the movement of robot."""
        command = "task_stop()\n"
        self.send_command(command)

    def jog_angle(self, joint_str, direction, speed):
        """Performs a jog operation on a specific joint.

        Args:
            joint_str (str): Joint identifier (e.g., 'J1', 'J2', ..., 'J6').
            direction (int): Direction of movement (-1 for negative, 1 for positive).
            speed (int): Speed of the jog operation.
        """
        command = (
            "jog_angle(" + joint_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)

    def jog_coord(self, axis_str, direction, speed):
        """Performs a jog operation on a specific coordinate axis.

        Args:
            axis_str (str): Axis identifier (e.g., 'X', 'Y', 'Z', 'RX', 'RY', 'RZ').
            direction (int): Direction of movement (-1 for negative, 1 for positive).
            speed (int): Speed of the jog operation.
        """
        command = (
            "jog_coord(" + axis_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)

    def get_digital_in(self, pin_number):
        """Retrieves the state of a digital input pin.

        Args:
            pin_number (int): Pin number (0-63).

        Returns:
            int: State of the pin (0 or 1).
        """
        command = "get_digital_in(" + str(int(pin_number)) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def get_digital_out(self, pin_number):
        """Retrieves the state of a digital output pin.

        Args:
            pin_number (int): Pin number (0-63).

        Returns:
            int: State of the pin (0 or 1).
        """
        command = "get_digital_out(" + str(int(pin_number)) + ")\n"
        print(command)
        res = self.send_command(command)
        return self.string_to_int(res)

    def set_digital_out(self, pin_number, pin_signal):
        """Sets the state of a digital output pin.

        Args:
            pin_number (int): Pin number (0-63).
            pin_signal (int): Signal to set (0 or 1).
        """
        command = (
            "set_digital_out("
            + str(int(pin_number))
            + ","
            + str(int(pin_signal))
            + ")\n"
        )
        self.send_command(command)

    def get_analog_in(self, pin_number):
        """Returns specified analog input pin value.

        Args:
            pin_number (int): pin number (0-63)

        Returns:
            float: pin value
        """
        command = "get_analog_in(" + str(int(pin_number)) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def get_analog_out(self, pin_number):
        """Returns specified analog output pin value.

        Args:
            pin_number (int): pin number (0-63)

        Returns:
            float: pin value
        """
        command = "get_analog_out(" + str(int(pin_number)) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def set_analog_out(self, pin_number, pin_value):
        """Sets specified analog output pin to given value.

        Args:
            pin_number (int): pin number (0~63).
            pin_value (float): pin value
        """
        command = (
            "set_analog_out(" + str(int(pin_number)) + "," + str(int(pin_value)) + ")\n"
        )
        self.send_command(command)

    def get_joint_current(self, joint_number):
        """Retrieves the current value of a specific joint.

        Args:
            joint_number (int): Joint index from 0 to 5 (for J1, J2, J3, J4, J5, J6).

        Returns:
            float: Current value of the specified joint.
        """
        command = "get_joint_current(" + str(joint_number) + ")\n"
        print(command)
        res = self.send_command(command)
        return self.string_to_double(res)

    def send_feed_override(self, override):
        """Sets the feed rate override percentage.

        Args:
            override (int): Feed rate override percentage (0-100).

        Returns:
            int: Status code indicating success or failure.
        """
        command = "set_feed_rate(" + str(override) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def get_acceleration(self):
        """Retrieves the current acceleration setting of the robot.

        Returns:
            int: Acceleration value.
        """
        command = "get_acceleration()\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def set_acceleration(self, acceleration):
        """Sets the acceleration value for the robot.

        Args:
            acceleration (int): Acceleration value to set.
        """
        command = "set_acceleration(" + str(acceleration) + ")\n"
        self.send_command(command)

    def command_wait_done(self):
        """Waits until the current command execution is completed."""
        command = "wait_command_done()\n"
        self.send_command(command)

    def wait(self, seconds):
        """Pauses execution for a specified number of seconds.

        Args:
            seconds (int): Number of seconds to wait.
        """
        command = "wait(" + str(seconds) + ")\n"
        self.send_command(command)

    def assign_variable(self, var_name, var_value):
        """Assigns a value to a variable on the robot.

        Args:
            var_name (str): Name of the variable.
            var_value (str): Value to assign to the variable.
        """
        command = 'assign_variable("' + str(var_name) + '",' + str(var_value) + ")\n"
        self.send_command(command)

    def get_variable(self, var_name):
        """Retrieves the value of a variable from the robot.

        Args:
            var_name (str): Name of the variable.

        Returns:
            str: Value of the variable.
        """
        command = 'get_variable("' + str(var_name) + '")\n'
        return self.send_command(command)

    def jog_relative(self, joint_id, angle, speed, mode):
        """Relative jog.

        Example:
            jog_relative('J1', 5, 600, 1)

        Args:
            joint_id (str): 'J1' - 'J6', 'X', 'Y', 'Z', 'RX', 'RY', 'RZ'
            angle (float): relative angle or coord value to move, can be negative
                           to move to other direction
            speed (int): speed value
            mode (int): 0 (coord mode) or 1 (joint mode)

        Returns:
            str: return data from socket
        """
        command = "jog_increment({},{},{},{})\n".format(joint_id, angle, speed, mode)
        return self.send_command(command)

    def enable_manual_brake_control(self, enable=True):
        """Enables or disables manual brake control.

        Args:
            enable (bool, optional): enable (True) or disable (False) manual
                                     brake control. Defaults to True.
        """
        self.set_digital_out(DO.BRAKE_MANUAL_MODE_ENABLE.value, enable)
        time.sleep(0.05)
        for joint in Joint:
            self.release_joint_brake(joint, False)

    def release_joint_brake(self, joint, release=True):
        """Releases or focuses (enables) specified joint's brake.

        Args:
            joint (Joint): joint Joint.J1 ~ Joint.J6
            release (bool): True to release, False to enable brake. Defaults to True.
        """
        self.set_digital_out(DO(joint.value + DO.J1_BRAKE_RELEASE.value).value, release)

    def is_collision_detected(self):
        """Checks if collision is detected.

        Returns:
            bool: True if collision is detected, False otherwise.
        """
        return self.get_digital_in(DI.COLLISION_DETECTED.value) == 1

    def set_gripper_state(self, state, speed):
        """Sets gripper state.

        Args:
            state (int): gripper state, 0 - open, 1 - close
            speed (int): speed, 1-100

        Returns:
            str: return message
        """
        command = "set_gripper_state(" + str(state) + "," + str(speed) + ")\n"
        return self.send_command(command)

    def set_gripper_value(self, value, speed):
        """Sets gripper open/close state.

        Args:
            value (int): gripper open value, 0-100
            speed (int): speed, 1-100

        Returns:
            str: return message
        """
        command = "set_gripper_value(" + str(value) + "," + str(speed) + ")\n"
        return self.send_command(command)

    def set_gripper_calibrate(self):
        """Sets gripper zero position.

        Returns:
            str: return message
        """
        command = "set_gripper_calibrate()\n"
        return self.send_command(command)

    def set_gripper_enabled(self, enabled):
        """Set gripper enabled.

        Args:
            enabled (int): 1 - enabled, 0 - disabled

        Returns:
            str: return message
        """
        command = "set_gripper_enabled(" + str(enabled) + ")\n"
        return self.send_command(command)

    def set_gripper_mode(self, mode):
        """Sets gripper mode. Default mode is IO, after setting mode 485
           cannot set mode back to IO (need gripper reboot).

        Args:
            mode (int): 0 - 485, 1 - IO

        Returns:
            str: return message
        """
        command = "set_gripper_mode(" + str(mode) + ")\n"
        return self.send_command(command)

    def init_ele_gripper(self):
        """Initializes Electric Gripper.

        Returns:
            str: return message
        """
        command = "init_ele_gripper()\n"
        return self.send_command(command)

    def set_ele_gripper_open(self):
        """Fully opens Electric Gripper.

        Returns:
            str: return message
        """
        command = "set_ele_gripper_open()\n"
        return self.send_command(command)

    def set_ele_gripper_close(self):
        """Fully closes Electric Gripper.

        Returns:
            str: return message
        """
        command = "set_ele_gripper_close()\n"
        return self.send_command(command)

    def set_tool_reference(self, tool_reference):
        """Set tool coordinate system.

        Args:
            tool_reference (list):

        Returns:
            _type_: _description_
        """
        rotation_matrix = np.eye(3)  # No rotation example
        translation_vector = tool_reference[:3]  # Tool offset along z-axis of flange
        rotation_matrix = CvtEulerAngleToRotationMatrix(
            tool_reference[3:6] * np.pi / 180.0
        )
        return transformation_matrix_from_parameters(
            rotation_matrix, translation_vector
        )

    def get_tool_coords(self):
        """Set tool coordinate system.

        Returns:
            _type_: _description_
        """
        current_coords = np.array(self.get_coords())
        tool_coords = flangeToTool(current_coords, self.tool_matrix)
        return tool_coords

    def write_tool_coords(self, tool_coords, speed):
        """Tool coordinate motion control

        Args:
            tool_coords (list): _description_
            speed (int): _description_
        """
        flange_coords = toolToflange(tool_coords, self.tool_matrix)
        self.write_coords(flange_coords, speed)

    #力控#
    def force_get_firmware(self, ID):
        """Obtain the main version number of the power control system

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return information
        """
        command = "force_GetFirmware(" + str(ID) + ")\n"
        value = int(self.send_command(command))
        if value == 255:
            return value
        return (int(self.send_command(command)))/10
    
    def force_get_modified(self, ID):
        """Obtain the updated version number of Power Control

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return information
        """
        command = "force_GetModified(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_set_id(self, ID, value):
        """Set force control ID

        Args:
            ID (int): Robot arm ID.
            Value (int): ID value (1~254).
        
        Returns:
            str: return message
        """
        if value < 1 or value > 254:
            raise MyCobot630ProDataException(
            "The Force_SetId value must be 1 ~ 254, but received {}".format(value))
        command = "force_SetGripperId(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_get_id(self, ID):
        """Obtain the ID of the force control machine

        Args:
            ID (int): Robot arm ID.
        
         Returns:
            str: return ID value
        """
        command = "force_GetGripperId(" + str(ID) + ")\n"
        return int(self.send_command(command))

    def force_set_enabled(self, ID, value):
        """Set force control enable

        Args:
            ID (int): Robot arm ID.
            Value (int): Enable value (0 or 1).

        Returns:
            str: return message
        """
        if value < 0 or value > 1:
            raise MyCobot630ProDataException(
            "The Force_SetEnabled value must be 0 ~ 1, but received {}".format(value))
        command = "force_SetGripperEnabled(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_set_angle(self, ID, value):
        """Set force control angle

        Args:
            ID (int): Robot arm ID.
            Value (int): Angle value (0~100).
        
        Returns:
            str: return message
        """
        if value < 0 or value > 100:
            raise MyCobot630ProDataException(
            "The Force_SetAngle value must be 0 ~ 100, but received {}".format(value))
        command = "force_SetAngle(" + str(ID) + "," + str(value) + ")\n"
        # print(command)
        return int(self.send_command(command))

    def force_get_angle(self, ID):
        """Obtain force control angle

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return angle value
        """
        command = "force_GetGripperAngle(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_set_calibrate(self, ID):
        """Initialize force control

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return message
        """
        command = "force_SetGripperCalibrate(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_get_gripper(self, ID):
        """Obtain the motion status of force control

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return status value
        """
        command = "force_GetGripper(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_set_torque(self, ID, value):
        """Set the torque value for force control

        Args:
            ID (int): Robot arm ID.
            Value (int): Torque value (0~100).
        Returns:
            str: return message
        """
        if value < 0 or value > 100:
            raise MyCobot630ProDataException(
            "The Force_SetTorque value must be 0 ~ 100, but received {}".format(value))
        command = "force_SetGripperTorque(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_get_torque(self, ID):
        """Obtain force control torque

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return torque value
        """
        command = "force_GetGripperTorque(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_set_open(self, ID, value):
        """Set the opening angle of the force control IO

        Args:
            ID (int): Robot arm ID.
            Value (int): Angle value (0~100).
        
        Returns:
            str: return message
        """
        if value < 0 or value > 100:
            raise MyCobot630ProDataException(
            "The Force_SetOpen value must be 0 ~ 100, but received {}".format(value))
        command = "force_SetOpen(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_set_close(self, ID, value):
        """Set the closing angle of the force control IO

        Args:
            ID (int): Robot arm ID.
            Value (int): Angle value (0~100).
        
        Returns:
            str: return angle value
        """
        if value < 0 or value > 100:
            raise MyCobot630ProDataException(
            "The Force_SetClose value must be 0 ~ 100, but received {}".format(value))
        command = "force_SetClose(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_set_speed(self, ID, value):
        """Set force control motion speed

        Args:
            ID (int): Robot arm ID.
            Value (int): Speed value (1~100).

        Returns:
            str: return message
        """
        if value < 1 or value > 100:
            raise MyCobot630ProDataException(
            "The Force_SetSpeed value must be 1 ~ 100, but received {}".format(value))
        command = "force_SetSpeed(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_get_speed(self, ID):
        """Obtain force controlled motion speed

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return speed value
        """
        command = "force_GetSpeed(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_get_open(self, ID):
        """Obtain the opening angle of the force control IO

        Args:
            ID (int): Robot arm ID.
            Value (int): Angle value (0~100).
        
        Returns:
            str: return angle value
        """
        command = "force_GetOpen(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_get_close(self, ID):
        """ Obtain the closing angle of the force control IO

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return angle value
        """
        command = "force_GetClose(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_set_absangle(self, ID, value):
        """Set the absolute angle of force control

        Args:
            ID (int): Robot arm ID.
            Value (int): Angle value (0~100).
        
        Returns:
            str: return message
        """
        if value < 0 or value > 100:
            raise MyCobot630ProDataException(
            "The SetAbsAngle  value must be 0 ~ 100, but received {}".format(value))
        command = "force_SetAbsAngle(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def force_pause(self, ID):
        """Pause absolute instruction queue

        Args:
           ID (int): Robot arm ID.
        
        Returns:
            str: return message
        """
        command = "force_Pause(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_resume(self, ID):
        """Open absolute command queue

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return message
        """
        
        command = "force_Resume(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def force_stop(self, ID):
        """Set absolute command to stop

        Args:
            ID (int): Robot arm ID.
        """
        command = "force_Stop(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def froce_get_queuecount(self, ID):
        """Obtain the number of absolute command queues for force control

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return message
        """
        command = "froce_GetCount(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    #三指#
    def hand_get_firmware(self, ID):
        """Obtain the main version number of the three fingers

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return major version value
        """
        command = "Hand_GetFirmware(" + str(ID) + ")\n"
        value = int(self.send_command(command))
        if value == 255:
            return value
        return (int(self.send_command(command)))/10
    
    def hand_get_modified(self, ID):
        """Obtain the updated version number of the three fingers

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return minor version
        """
        command = "Hand_GetModified(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def hand_set_id(self, ID, value):
        """Set three finger machine ID

        Args:
            ID (int): Robot arm ID.
            Value (int): hand id value (1~254).
        
        Returns:
            str: return message
        """
        if value < 1 or value > 254:
            raise MyCobot630ProDataException(
            "The Hand  value must be 1 ~ 254, but received {}".format(value))
        command = "Hand_SetId(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def hand_get_id(self, ID):
        """Obtain the three finger machine ID

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return id value
        """
        command = "Hand_GetId(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def hand_set_enabled(self, ID, value):
        """Claw enable

        Args:
            ID (int): Robot arm ID.
            Value (int): Enable value (0 or 1).
        
        Returns:
            str: return message
        """
        if value < 0 or value > 1:
            raise MyCobot630ProDataException(
            "The SetEnabled  value must be 0 ~ 1, but received {}".format(value))
        command = "Hand_SetEnabled(" + str(ID) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def hand_set_joint_angle(self, ID, Jiont, value):
            """Set the angle of a single joint.

            Args:
                ID (int): Robot arm ID.
                Joint (int): Joint number (1~6).
                Value (int): Angle value (0~100).
            
            Returns:
                str: return message
            """
            if value < 0 or value > 100:
                raise MyCobot630ProDataException(
            "The Angle value must be 0 ~ 100, but received {}".format(value))
            if Jiont < 1 or Jiont > 6:
                raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
            command = "Hand_SetJointAngle(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
            return int(self.send_command(command))
    
    def hand_get_joint_angle(self, ID, Jiont):
        """Obtain the angle of a single joint.

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
        
        Returns:
            str: return Single joint angle value
        """
        if Jiont < 1 or Jiont > 6:
            raise MyCobot630ProDataException(
            "The gripper Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_GetJointAngle(" + str(ID) + "," + str(Jiont) + ")\n"
        return int(self.send_command(command))
    
    
    def hand_set_joint_calibrate(self, ID, Jiont):
        """Initialize joints

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
        
        Returns:
            str: return message
        """
        if Jiont < 1 or Jiont > 6:
            raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_SetJointCalibrate(" + str(ID) + "," + str(Jiont) + ")\n"
        return int(self.send_command(command))
    

    def hand_get_state(self, ID):
        """Obtain the motion status of the three fingers.

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return Numerical value of motion state
        """
        command = "Hand_GetHand(" + str(ID) + ")\n"
        return int(self.send_command(command))
    
    def hand_set_torque(self, ID, Jiont, value):
        """Set the torque of the three finger joint.

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
            Value (int): Torque value (0~100).
        
        Returns:
            str: return message
        """
        if value < 0 or value > 100:
                raise MyCobot630ProDataException(
            "The hand torque value must be 0 ~ 100, but received {}".format(value))
        if Jiont < 1 or Jiont > 6:
            raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_SetTorque(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
        return int(self.send_command(command))
    
    def hand_get_torque(self, ID, Jiont):
        """Obtain the torque of the three finger joint.

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
        
        Returns:
            str: return Single Joint torque value
        """
        if Jiont < 1 or Jiont > 6:
            raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_GetTorque(" + str(ID) + "," + str(Jiont) + ")\n"
        return int(self.send_command(command))
    
   
    def hand_set_speed(self, ID, Jiont, value):
        """Set the speed of the three finger joint.

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
            Value (int): Speed value (1~100).
        
        Returns:
            str: return message
        """
        if value < 1 or value > 100:
                raise MyCobot630ProDataException(
            "The Hand speed value must be 1 ~ 100, but received {}".format(value))
        if Jiont < 1 or Jiont > 6:
             raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_SetSpeed(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
        return int(self.send_command(command))

 
    def hand_get_speed(self, ID, Jiont):
        """Obtain the velocity of the three finger joint.

        Args:
            ID (int): Robot arm ID.
            Joint (int): Joint number (1~6).
        
        Returns:
            str: return Single Joint speed value
        """
        if Jiont < 1 or Jiont > 6:
             raise MyCobot630ProDataException(
            "The Hand Jiont value must be 1 ~ 6, but received {}".format(Jiont))
        command = "Hand_GetSpeed(" + str(ID) + "," + str(Jiont) + ")\n"
        return int(self.send_command(command))
    
    def hand_set_fullangles(self, ID, angles, speed):
        """Set the angle of the three finger joint.

        Args:
            ID (int): Robot arm ID.
            Angles (list of int): A list containing 6 angle values, each value range must be (0~100).
            Value (int): Speed value (1~100).
        
        Returns:
            str: return message
        """
        if speed < 1 or speed > 100:
                 raise MyCobot630ProDataException(
            "The Hand speed value must be 1 ~ 100, but received {}".format(speed))
        if len(angles) == 6:
            if any(x < 0 or x > 100 for x in angles):
                 raise MyCobot630ProDataException(
            "The FullAngles value must be 0 ~ 100, but received {}".format(angles))
            else:
                command = "Hand_SetFullAngles(" + str(ID) + "," + str(angles[0]) + "," \
                                        + str(angles[1]) + "," + str(angles[2]) + "," \
                                        + str(angles[3]) + "," + str(angles[4]) + "," \
                                        + str(angles[5])+ "," + str(speed) + ")\n"
                return int(self.send_command(command))
        else:
            raise MyCobot630ProDataException(
            "Enter the number of angles must be 6, but received {}".format(len(angles)))

    def hand_get_fullangles(self, ID):
        """Obtain the total joint angle of three fingers.

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return All Joint angle value
。
        """
        command = "Hand_GetFullAngles(" + str(ID) + ")\n"
        angle_list= parse_mixed_list_string(self.send_command(command))
        return angle_list

    def hand_set_catch(self, ID, pose, value, num=0):   
        """Three finger gesture control.

        Args:
            ID (int): Robot arm ID.
            Pose (int): gesture type (0~4).
            Value (int): threshold (0~5), when pose is 4, value is (0~20).
            Num (int): defaults to all hand movements when no value is passed in, and only moves pose type joints when a value is passed in.

        Returns:
            str: return message
        """
        if pose < 0 or pose > 4:
                raise MyCobot630ProDataException(
            "The Hand pose value must be 0 ~ 4, but received {}".format(pose))
        if pose == 4:
            if value < 1 or value > 20:
                raise MyCobot630ProDataException(
            "The pose is 4 so value must be 1 ~ 20, but received {}".format(value))
        else:
            if value < 0 or value > 5:
                raise MyCobot630ProDataException(
            "The pose is less than 4 value must be 0 ~ 5, but received {}".format(value))
        if num <= 0:
            command = "Hand_SetCatch(" + str(ID) + "," + str(pose) + "," + str(value) + ")\n"
        else :
            command = "Hand_SetCatch(" + str(ID) + "," + str(pose) + ","+ str(value) +  "," + str(num) +")\n"
        return int(self.send_command(command))
    
    def hand_get_model(self,ID):    
        """Obtain the type of three finger machine, such as left-hand 0 or right-hand 1.

        Args:
            ID (int): Robot arm ID.
        
        Returns:
            str: return finger machine type
        """
        command = "Hand_GetModel(" + str(ID) + ")\n"
        return int(self.send_command(command))


    #末端
    def get_end_firmware(self):             
        """Obtain the final major version number.

        Returns:
            str: return Atom major version number

        """
        command = "GetFirmwareEnd()\n"
        value = self.send_command(command)
        if value == "25.5":
            value = int((float(value)) * 10)
            return value
        value = float(value)
        return value

    def get_end_modify(self):                
        """Obtain the end update version number.

        Returns:
            str: return Atom update version number

        """
        command = "GetModifyEnd()\n"
        return int(self.send_command(command))
    def get_end_bt_status(self):                
        """Get the status of the end button.

        Returns:
            str: return message

        """
        command = "SetEndBtStatus()\n"
        return int(self.send_command(command))
    def set_end_color(self, red, green, blue):
        """Obtain the end color.

        Args:
            red (int): (0 ~ 255)。
            green (int): (0 ~ 255)。
            blue (int): (0 ~ 255)。
        
        Returns:
            str: return message
        """
        if red < 0 or green < 0 or blue < 0:
            raise MyCobot630ProDataException(
            "The input RGB parameter should be: 0~255, but received {},{},{}".format(red,green,blue))
        if red > 255 or green > 255 or blue > 255:
            raise MyCobot630ProDataException(
            "The input RGB parameter should be: 0~255, but received {},{},{}".format(red,green,blue))

        command = "SetLedColor(" + str(red) + "," + str(green) + "," + str(blue) + ")\n"
        return int(self.send_command(command))
    
        

if __name__ == "__main__":
    ep = ElephantRobot("192.168.1.248", 5001)
    resp = ep.start_client()
    if resp != True:
        print(resp)
        sys.exit(1)
    print(ep.wait(5))
    time.sleep(2)
    ep.force_get_angle(14,0)
    print(ep.Force_SetAngle(14,20))
    print(ep.Force_SetId(14,255))
    print(ep.get_coords())
    print(ep.get_speed())
    print(ep._power_on())
    print(ep._power_off())
    print(ep.check_running())
    print(ep.state_check())
    print(ep.program_open("a.tax"))
    print(ep.program_run(0))
    print(ep.read_next_error())
    print(ep.write_coords([1, 2, 3, 4, 5, 6], 110))
    print(ep.write_coord(1, 100, 200))
    print(ep.write_angles([10, 20, 30, 40, 50, 60], 110))
    print(ep.write_angle(3, 180, 200))
    print(ep.set_speed(377))
    print(ep.set_carte_torque_limit("x", 55))
    print(ep.set_upside_down(False))
    print(ep.set_payload(100))
    print(ep._state_on())
    print(ep._state_off())
    print(ep.task_stop())
    print(ep.jog_angle("j2", 1, 300))
    print(ep.jog_coord("rY", 0, 200))
    print(ep.get_digital_in(3))
    print(ep.get_digital_out(3))
    print(ep.set_digital_out(3, 1))
    print(ep.get_digital_out(3))
    print(ep.set_analog_out(1, 3.5))
    print(ep.get_acceleration())
    print(ep.set_acceleration(55))
    print(ep.command_wait_done())
    print(ep.get_variable("f"))
    print(ep.assign_variable("ss", '"eee"'))
    print(ep.get_joint_current(1))
    print(ep.wait(5))
    time.sleep(2)
    ep.force_get_angle(14,0)
    print(ep.Force_SetAngle(14,20))
    print(ep.Force_SetId(14,255))
    print(ep.get_coords())
    print(ep.get_speed())
    print(ep._power_on())
    print(ep._power_off())
    print(ep.check_running())
    print(ep.state_check())
    print(ep.program_open("a.tax"))
    print(ep.program_run(0))
    print(ep.read_next_error())
    print(ep.write_coords([1, 2, 3, 4, 5, 6], 110))
    print(ep.write_coord(1, 100, 200))
    print(ep.write_angles([10, 20, 30, 40, 50, 60], 110))
    print(ep.write_angle(3, 180, 200))
    print(ep.set_speed(377))
    print(ep.set_carte_torque_limit("x", 55))
    print(ep.set_upside_down(False))
    print(ep.set_payload(100))
    print(ep._state_on())
    print(ep._state_off())
    print(ep.task_stop())
    print(ep.jog_angle("j2", 1, 300))
    print(ep.jog_coord("rY", 0, 200))
    print(ep.get_digital_in(3))
    print(ep.get_digital_out(3))
    print(ep.set_digital_out(3, 1))
    print(ep.get_digital_out(3))
    print(ep.set_analog_out(1, 3.5))
    print(ep.get_acceleration())
    print(ep.set_acceleration(55))
    print(ep.command_wait_done())
    print(ep.get_variable("f"))
    print(ep.assign_variable("ss", '"eee"'))
    print(ep.get_joint_current(1))

    print(ep.set_gripper_mode(1))
    print(ep.wait(2))
    print(ep.set_gripper_mode(0))

    print(ep.set_gripper_value(100, 20))
    print(ep.wait(2))
    print(ep.set_gripper_value(50, 20))
    print(ep.wait(2))
    print(ep.set_gripper_value(0, 20))
    print(ep.wait(2))

    print(ep.set_gripper_enabled(1))
    print(ep.wait(2))
    print(ep.set_gripper_enabled(0))

    ep.stop_client()


    # name = input("input:")

    # print(name)
