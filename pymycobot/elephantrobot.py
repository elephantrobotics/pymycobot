#!/usr/bin/python

from socket import socket, AF_INET, SOCK_STREAM
import sys
from enum import Enum
import base64
import hashlib
import math
import numpy as np
from multiprocessing import Lock
import logging
from pymycobot.log import setup_logging
from pymycobot.tool_coords import *
import time

error_For = "Parameter out of range, correct parameter range:"
error_Jion = "Joint out of range, correct joint range:"
error_Angle = "Angle out of range, correct angle range:"

COORDS_EPSILON = 0.50


class Axis(Enum):
    X = 0
    Y = 1
    Z = 2
    RX = 3
    RY = 4
    RZ = 5


class Joint(Enum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3
    J5 = 4
    J6 = 5


class JogMode(Enum):
    JOG_JOINT = 0
    JOG_TELEOP = 1


mutex = Lock()


class ElephantRobot(object):
    def __init__(self, host, port, debug=False):
        # setup connection
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
        try:
            self.tcp_client.connect(self.ADDR)
            # print("尝试连接到:", self.ADDR)
            self.is_client_started = True
            return True
        except Exception as e:
            print(e)
            return False

    def stop_client(self):
        self.tcp_client.close()
        self.is_client_started = False

    def send_command(self, command):
        # with mutex:
        #     self.tcp_client.send(command.encode())

        #     recv_data = self.tcp_client.recv(self.BUFFSIZE).decode()
        #     res_str = str(recv_data)
        #     if self.debug:
        #         print("recv = " + res_str)
        #     res_arr = res_str.split(":")
        #     if len(res_arr) == 2:
        #         return res_arr[1]
        #     else:
        #         return ""

        with mutex:
            if not command.endswith('\n'):
                command += '\n'

            self.tcp_client.send(command.encode())

            self.tcp_client.settimeout(10.0)  # 设置 2 秒超时
            try:
                recv_data = self.tcp_client.recv(self.BUFFSIZE).decode()
            except socket.timeout:
                print("等待服务器回应超时")
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
        coords = [-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]
        return coords

    def detect_robot(self):
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
        command = "get_angles()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_coords(self):
        command = "get_coords()\n"
        res = self.send_command(command)
        return self.string_to_coords(res)

    def get_speed(self):
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

    def _state_on(self):
        command = "state_on()\n"
        self.send_command(command)

    def _state_off(self):
        command = "state_off()\n"
        self.send_command(command)

    def state_check(self):
        command = "state_check()\n"
        res = self.send_command(command)
        return res == "1"

    def check_running(self):
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
        command = f"read_text_file({remote_filename})"
        res = self.send_command(command)
        return res

    def program_open(self, file_path):
        command = "program_open(" + file_path + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def program_run(self, start_line):
        command = "program_run(" + str(start_line) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def read_next_error(self):
        command = "read_next_error()\n"
        res = self.send_command(command)
        return res

    def clear_all_errors(self):
        while self.read_next_error() != "":
            pass

    def write_coords(self, coords, speed):
        command = "set_coords("
        for item in coords:
            command += str(round(item, 3)) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    def write_coord(self, axis, value, speed):
        coords = self.get_coords()
        if coords != self.invalid_coords():
            coords[axis] = value
            self.write_coords(coords, speed)

    def write_angles(self, angles, speed):
        command = "set_angles("
        for item in angles:
            command += str(round(item, 3)) + ","
        command += str(speed) + ")\n"
        self.send_command(command)

    def write_angle(self, joint, value, speed):
        angles = self.get_angles()
        if angles != self.invalid_coords():
            angles[joint] = value
            self.write_angles(angles, speed)

    def set_speed(self, percentage):
        command = "set_speed(" + str(percentage) + ")\n"
        self.send_command(command)

    def set_carte_torque_limit(self, axis_str, value):
        command = "set_torque_limit(" + axis_str + "," + str(value) + ")\n"
        self.send_command(command)

    def set_upside_down(self, up_down):
        up = "1"
        if up_down:
            up = "0"
        command = "set_upside_down(" + up + ")\n"
        self.send_command(command)

    def task_stop(self):
        command = "task_stop()\n"
        self.send_command(command)

    def jog_angle(self, joint_str, direction, speed):
        command = (
            "jog_angle(" + joint_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)

    def jog_coord(self, axis_str, direction, speed):
        command = (
            "jog_coord(" + axis_str + "," + str(direction) + "," + str(speed) + ")\n"
        )
        self.send_command(command)

    def get_digital_in(self, pin_number):
        command = "get_digital_in(" + str(pin_number) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def get_digital_out(self, pin_number):
        command = "get_digital_out(" + str(pin_number) + ")\n"
        print(command)
        res = self.send_command(command)
        return self.string_to_int(res)

    def set_digital_out(self, pin_number, pin_signal):
        command = "set_digital_out(" + str(pin_number) + "," + str(pin_signal) + ")\n"
        self.send_command(command)

    def get_analog_in(self, pin_number):
        """Returns specified analog input pin value.

        Args:
            pin_number (int): pin number (0-63)

        Returns:
            float: pin value
        """
        command = "get_analog_in(" + str(pin_number) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def get_analog_out(self, pin_number):
        """Returns specified analog output pin value.

        Args:
            pin_number (int): pin number (0-63)

        Returns:
            float: pin value
        """
        command = "get_analog_out(" + str(pin_number) + ")\n"
        res = self.send_command(command)
        return self.string_to_double(res)

    def set_analog_out(self, pin_number, pin_value):
        """Sets specified analog output pin to given value.

        Args:
            pin_number (int): pin number (0~63).
            pin_value (float): pin value
        """
        command = "set_analog_out(" + str(pin_number) + "," + str(pin_value) + ")\n"
        self.send_command(command)

    def get_joint_current(self, joint_number):
        command = "get_joint_current(" + str(joint_number) + ")\n"
        print(command)
        res = self.send_command(command)
        return self.string_to_double(res)

    def send_feed_override(self, override):
        command = "set_feed_rate(" + str(override) + ")\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def get_acceleration(self):
        command = "get_acceleration()\n"
        res = self.send_command(command)
        return self.string_to_int(res)

    def set_acceleration(self, acceleration):
        command = "set_acceleration(" + str(acceleration) + ")\n"
        self.send_command(command)

    def command_wait_done(self):
        command = "wait_command_done()\n"
        self.send_command(command)

    def wait(self, seconds):
        command = "wait(" + str(seconds) + ")\n"
        self.send_command(command)

    def assign_variable(self, var_name, var_value):
        command = 'assign_variable("' + str(var_name) + '",' + str(var_value) + ")\n"
        self.send_command(command)

    def get_variable(self, var_name):
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

    def force_gripper_set_angle(self, angle):
        """Sets angle of Force-Controlled gripper.

        Args:
            angle (int): angle, 0-100

        Returns:
            str: return message
        """
        command = "force_gripper_set_angle(" + str(angle) + ")\n"
        return self.send_command(command)

    def force_gripper_get_angle(self):
        """Returns current angle of force-controlled gripper.

        Returns:
            str: return message
        """
        command = "force_gripper_get_angle()\n"
        return self.send_command(command)

    def force_gripper_full_open(self):
        """Fully opens force-controlled gripper.

        Returns:
            str: return message
        """
        command = "force_gripper_full_open()\n"
        return self.send_command(command)

    def force_gripper_full_close(self):
        """Fully closes force-controlled gripper.

        Returns:
            str: return message
        """
        command = "force_gripper_full_close()\n"
        return self.send_command(command)

    def force_gripper_set_torque(self, torque):
        """Sets torque of force-controlled gripper.

        Args:
            torque (int): torque, 0-100 (mapped to 100-300)

        Returns:
            str: return message
        """
        command = "force_gripper_set_torque(" + str(torque) + ")\n"
        return self.send_command(command)

    def force_gripper_get_torque(self):
        """Returns current torque of force-controlled gripper.

        Returns:
            str: return message
        """
        command = "force_gripper_get_torque()\n"
        return self.send_command(command)
    
    def set_tool_reference(self, tool_reference):
        """Set tool coordinate system.

        Args:
            tool_reference (list): 

        Returns:
            _type_: _description_
        """
        rotation_matrix = np.eye(3)  # No rotation example
        translation_vector = tool_reference[:3] # Tool offset along z-axis of flange
        rotation_matrix = CvtEulerAngleToRotationMatrix(tool_reference[3:6]* np.pi/180.0)  
        return transformation_matrix_from_parameters(rotation_matrix, translation_vector)
    
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
    def Force_GetFirmware(self, ID):
        command = "force_GetFirmware(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_GetModified(self, ID):
        command = "force_GetModified(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_SetId(self, ID, value):
        if value < 1 or value > 254:
            return error_For + "1 - 254"
        command = "force_SetGripperId(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_GetId(self, ID):
        command = "force_GetGripperId(" + str(ID) + ")\n"
        return self.send_command(command)

    def Force_SetEnabled(self, ID, value):
        if value < 0 or value > 1:
            return error_For + "0 - 1"
        command = "force_SetGripperEnabled(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_SetAngle(self, ID, value):
        if value < 0 or value > 100:
            return error_For + "0 - 100"
        command = "force_SetAngle(" + str(ID) + "," + str(value) + ")\n"
        # print(command)
        return self.send_command(command)

    def Force_GetAngle(self, ID):
        command = "force_GetGripperAngle(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_SetCalibrate(self, ID):
        command = "force_SetGripperCalibrate(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_GetGripper(self, ID):
        command = "force_GetGripper(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_SetTorque(self, ID, value):
        if value < 0 or value > 100:
            return error_For + "0 - 100"
        command = "force_SetGripperTorque(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_GetTorque(self, ID):
        command = "force_GetGripperTorque(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_SetOpen(self, ID, value):
        if value < 0 or value > 100:
            return error_For + "0 - 100"
        command = "force_SetOpen(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_SetClose(self, ID, value):
        if value < 0 or value > 100:
            return error_For + "0 - 100"
        command = "force_SetClose(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_SetSpeed(self, ID, value):
        if value < 1 or value > 100:
            return error_For + "1 - 100"
        command = "force_SetSpeed(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_GetSpeed(self, ID):
        command = "force_GetSpeed(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_GetOpen(self, ID):
        command = "force_GetOpen(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_GetClose(self, ID):
        command = "force_GetClose(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_SetAbsAngle(self, ID, value):
        if value < 0 or value > 100:
            return error_For + "0 - 100"
        command = "force_SetAbsAngle(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Force_Pause(self, ID):
        command = "force_Pause(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_Resume(self, ID):
        command = "force_Resume(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Force_Stop(self, ID):
        command = "force_Stop(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Froce_GetQueueCount(self, ID):
        command = "froce_GetQueueCount(" + str(ID) + ")\n"
        return self.send_command(command)
    
    #三指#
    def Hand_GetFirmware(self, ID):
        command = "Hand_GetFirmware(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Hand_GetModified(self, ID):
        command = "Hand_GetModified(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Hand_SetId(self, ID, value):
        if value < 1 or value > 254:
            return error_For + "1 - 254"
        command = "Hand_SetId(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Hand_GetId(self, ID):
        command = "Hand_GetId(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Hand_SetEnabled(self, ID, value):
        if value < 0 or value > 1:
            return error_For + "0 - 1"
        command = "Hand_SetEnabled(" + str(ID) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Hand_SetJointAngle(self, ID, Jiont, value):
            if value < 0 or value > 100:
                return error_For + "0 - 100"
            if Jiont < 1 or Jiont > 6:
                return error_Jion + " 1 - 6"
            command = "Hand_SetJointAngle(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
            return self.send_command(command)
    
    def Hand_GetJointAngle(self, ID, Jiont):
        if Jiont < 1 or Jiont > 6:
            return error_Jion + " 1 - 6"
        command = "Hand_GetJointAngle(" + str(ID) + "," + str(Jiont) + ")\n"
        return self.send_command(command)
    
    
    def Hand_SetJointCalibrate(self, ID, Jiont):
        if Jiont < 1 or Jiont > 60:
            return error_Jion + " 1 - 6"
        command = "Hand_SetJointCalibrate(" + str(ID) + "," + str(Jiont) + ")\n"
        return self.send_command(command)
    

    def Hand_GetHand(self, ID):
        command = "Hand_GetHand(" + str(ID) + ")\n"
        return self.send_command(command)
    
    def Hand_SetTorque(self, ID, Jiont, value):
        if value < 0 or value > 100:
                return error_For + "0 - 100"
        if Jiont < 1 or Jiont > 6:
            return error_Jion + " 1 - 6"
        command = "Hand_SetTorque(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
        return self.send_command(command)
    
    def Hand_GetTorque(self, ID, Jiont):
        if Jiont < 1 or Jiont > 6:
            return error_Jion + " 1 - 6"
        command = "Hand_GetTorque(" + str(ID) + "," + str(Jiont) + ")\n"
        return self.send_command(command)
    
    def Hand_SetSpeed(self, ID, Jiont, value):
        if value < 1 or value > 100:
                return error_For + "1 - 100"
        if Jiont < 1 or Jiont > 6:
            return error_Jion + " 1 - 6"
        command = "Hand_SetSpeed(" + str(ID) + "," + str(Jiont) + "," + str(value) + ")\n"
        return self.send_command(command)

    def Hand_GetSpeed(self, ID, Jiont):
        if Jiont < 1 or Jiont > 6:
            return error_Jion + " 1 - 6"
        command = "Hand_GetSpeed(" + str(ID) + "," + str(Jiont) + ")\n"
        return self.send_command(command)
    
    def Hand_SetFullAngles(self, ID, angles, speed):
        if speed < 1 or speed > 100:
                return error_For + "1 - 100"
        if len(angles) == 6:
            if any(x < 0 or x > 100 for x in angles):
                return error_Angle + " 0 - 100"
            else:
                command = "Hand_SetFullAngles(" + str(ID) + "," + str(angles[0]) + "," \
                                        + str(angles[1]) + "," + str(angles[2]) + "," \
                                        + str(angles[3]) + "," + str(angles[4]) + "," \
                                        + str(angles[5])+ "," + str(speed) + ")\n"
                return self.send_command(command)
        else:
            return "Number of angles does not match"

    
    def Hand_GetFullAngles(self, ID):
        command = "Hand_GetFullAngles(" + str(ID) + ")\n"
        return self.send_command(command)

    def Hand_SetCatch(self, ID, pose, value, num=0):
        if pose < 0 or pose > 4:
                return error_For + "0 - 4"
        if pose == 4:
            if value < 1 or value > 20:
                return "The parameter range is: 1 - 20"
        else:
            if value < 0 or value > 5:
                return "The parameter range is: 0 - 5"
        if num <= 0:
            command = "Hand_SetCatch(" + str(ID) + "," + str(pose) + "," + str(value) + ")\n"
        else :
            command = "Hand_SetCatch(" + str(ID) + "," + str(pose) + ","+ str(value) +  "," + str(num) +")\n"
        return self.send_command(command)

    def Hand_GetModel(self,ID):
        command = "Hand_GetModel(" + str(ID) + ")\n"
        return self.send_command(command)


    #末端
    def get_end_Firmware(self):             #主版本
        command = "GetFirmwareEnd()\n"
        return self.send_command(command)
    def get_end_Modify(self):                #更新版本
        command = "GetModifyEnd()\n"
        return self.send_command(command)
    def get_end_bt_status(self):                #获取按键状态
        command = "SetEndBtStatus()\n"
        return self.send_command(command)
    def set_end_color(self, red, green, blue):
        if red < 0 or green < 0 or blue < 0:
            return "Parameter out of range, range should be: 0-255"
        if red > 255 or green > 255 or blue > 255:
            return "Parameter out of range, range should be: 0-255"

        command = "SetLedColor(" + str(red) + "," + str(green) + "," + str(blue) + ")\n"
        return self.send_command(command)
    
        

if __name__ == "__main__":
    ep = ElephantRobot("192.168.123.13", 5001)
    resp = ep.start_client()
    if resp != True:
        print(resp)
        sys.exit(1)
    # print(ep.wait(5))
    # print("发送指令")
    Jiont = [100,100,100,100,100,100]
    time.sleep(2)
    # ep.Force_SetAngle(14,0)
    # print(ep.Force_SetAngle(14,20))
    time.sleep(2)
    print(ep.Force_SetCalibrate(14))
    # print(ep.get_coords())
    # print(ep.get_speed())
    # # print(ep._power_on())
    # # print(ep._power_off())
    # print(ep.check_running())
    # print(ep.state_check())
    # print(ep.program_open("a.tax"))
    # print(ep.program_run(0))
    # print(ep.read_next_error())
    # print(ep.write_coords([1, 2, 3, 4, 5, 6], 110))
    # print(ep.write_coord(1, 100, 200))
    # print(ep.write_angles([10, 20, 30, 40, 50, 60], 110))
    # print(ep.write_angle(3, 180, 200))
    # print(ep.set_speed(377))
    # print(ep.set_carte_torque_limit("x", 55))
    # print(ep.set_upside_down(False))
    # print(ep.set_payload(100))
    # print(ep._state_on())
    # print(ep._state_off())
    # print(ep.task_stop())
    # print(ep.jog_angle("j2", 1, 300))
    # print(ep.jog_coord("rY", 0, 200))
    # print(ep.get_digital_in(3))
    # print(ep.get_digital_out(3))
    # print(ep.set_digital_out(3, 1))
    # print(ep.get_digital_out(3))
    # print(ep.set_analog_out(1, 3.5))
    # print(ep.get_acceleration())
    # print(ep.set_acceleration(55))
    # print(ep.command_wait_done())
    # print(ep.get_variable("f"))
    # print(ep.assign_variable("ss", '"eee"'))
    # print(ep.get_joint_current(1))

    # print(ep.set_gripper_mode(1))
    # print(ep.wait(2))
    # print(ep.set_gripper_mode(0))

    # print(ep.set_gripper_value(100, 20))
    # print(ep.wait(2))
    # print(ep.set_gripper_value(50, 20))
    # print(ep.wait(2))
    # print(ep.set_gripper_value(0, 20))
    # print(ep.wait(2))

    # print(ep.set_gripper_enabled(1))
    # print(ep.wait(2))
    # print(ep.set_gripper_enabled(0))

    ep.stop_client()


    # name = input("input:")

    # print(name)
