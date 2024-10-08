#!/usr/bin/python

from socket import socket, AF_INET, SOCK_STREAM
import sys
import base64
import hashlib
from multiprocessing import Lock
import logging
from pymycobot.log import setup_logging


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

    def start_client(self):
        try:
            self.tcp_client.connect(self.ADDR)
            return True
        except Exception as e:
            print(e)
            return False

    def stop_client(self):
        self.tcp_client.close()

    def send_command(self, command):
        with mutex:
            self.tcp_client.send(command.encode())
            recv_data = self.tcp_client.recv(self.BUFFSIZE).decode()
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

    def string_to_int(self, data):
        try:
            val = int(data)
            return val
        except:
            return -9999

    def invalid_coords(self):
        coords = [-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]
        return coords

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

    def power_on(self):
        command = "power_on()\n"
        res = self.send_command(command)
        return True

    def power_off(self):
        command = "power_off()\n"
        res = self.send_command(command)
        return True

    def start_robot(self):
        command = "start_robot()\n"
        res = self.send_command(command)
        return True

    def check_running(self):
        command = "check_running()\n"
        res = self.send_command(command)
        return res == "1"

    def state_check(self):
        command = "state_check()\n"
        res = self.send_command(command)
        return res == "1"

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

    def set_payload(self, payload):
        command = "set_payload(" + str(payload) + ")\n"
        self.send_command(command)

    def state_on(self):
        command = "state_on()\n"
        self.send_command(command)

    def state_off(self):
        command = "state_off()\n"
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

    def get_joint_current(self, joint_number):
        command = "get_joint_current(" + str(joint_number) + ")\n"
        print(command)
        res = self.send_command(command)
        return self.string_to_double(res)

    def set_digital_out(self, pin_number, pin_signal):
        command = "set_digital_out(" + str(pin_number) + "," + str(pin_signal) + ")\n"
        self.send_command(command)

    def set_analog_out(self, pin_number, pin_signal):
        command = "set_analog_out(" + str(pin_number) + "," + str(pin_signal) + ")\n"
        self.send_command(command)

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


if __name__ == "__main__":
    ep = ElephantRobot("192.168.124.28", 5001)
    resp = ep.start_client()
    if resp != "":
        print(resp)
        sys.exit(1)
    print(ep.wait(5))
    print(ep.get_angles())
    print(ep.get_coords())
    print(ep.get_speed())
    # print(ep.power_on())
    # print(ep.power_off())
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
    print(ep.state_on())
    print(ep.state_off())
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

    name = input("input:")
    print(name)
