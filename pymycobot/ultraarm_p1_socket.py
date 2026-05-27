#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ultraarm_p1_socket.py

Python socket interface for the ultraArmP1 robotic arm.

Author: weijian.wang
Date: 2026-03-11
"""
import locale
import logging
import os
import re
import socket
import threading
import time
import datetime
import select

from pymycobot.log import setup_logging
from pymycobot.common import ProtocolCode
from pymycobot.error import calibration_parameters
from pymycobot.robot_info import UltraArmP1RobotInfo, RobotLimit


class UltraArmP1Socket:
    """Socket communication interface for ultraArmP1."""

    def __init__(self, ip, netport=9000, timeout=0.05, debug=False):
        """Initialize the ultraArmP1 robot communication.

        Args:
            ip     : Server IP address
            netport : Socket port number, default is 9000
            timeout (float, optional): Serial read timeout in seconds. Defaults to 0.05.
            debug (bool, optional): Whether to print debug information. Defaults to False.
        """
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.sock.settimeout(timeout)
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self.calibration_parameters = calibration_parameters
        self.lock = threading.Lock()
        time.sleep(0.5)

        self.language, _ = locale.getdefaultlocale()
        if self.language not in ["zh_CN", "en_US"]:
            self.language = "en_US"

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

    # ---------------------- Debug / time helpers ----------------------
    def _now(self):
        """Return timestamp string with millisecond precision."""
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _debug_write(self, data: str):
        if self.debug:
            self.log.debug("_write: {}".format(data))

    def _debug_read(self, data: str):
        if self.debug:
            self.log.debug(" _read: {}".format(data))

    # ---------------------- Socket helpers ----------------------

    def _socket_in_waiting(self):

        ready = select.select([self.sock], [], [], 0)
        if ready[0]:
            return 1
        return 0

    def _read_available_bytes(self):

        if not self._socket_in_waiting():
            return b""

        try:
            data = self.sock.recv(1024)
            return data
        except Exception:
            return b""

    # ---------------------- Response waiting & parsing ----------------------
    def _response(self, _async=True, _gcode=False, is_set=False):
        """Wait for device response from the serial buffer.

        Returns 'ok' when keyword is found, False on timeout.
        """
        if _gcode:
            _async = False

        if not _async and not _gcode:
            return 1
        start_time = time.time()
        wait_time = 300

        response_timeout = wait_time
        if is_set and wait_time == 300:
            response_timeout = 3

        received_data = b""
        text_buffer = ""
        if _gcode:
            keyword = b"start"
        else:  # _async=True
            keyword = b"end"

        movement_status_wait = (
                _async and not _gcode and not is_set and wait_time == 300
        )
        if is_set:
            response_timeout = 7

        status_timeout = 3
        status_query_interval = 0.2
        last_status_time = start_time
        last_status_query_time = start_time - status_query_interval

        while time.time() - start_time < response_timeout:
            chunk = self._read_available_bytes()
            if chunk:
                received_data += chunk
                # try decode for debug
                try:
                    chunk_text = chunk.decode("utf-8", errors="ignore")
                except Exception:
                    chunk_text = str(chunk)

                # accumulate text buffer
                text_buffer += chunk_text
                while "\n" in text_buffer:
                    line, text_buffer = text_buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue
                    # keep original frame for debug
                    origin_line = line

                    # ----------------------------------------------------------
                    # checksum verify (compatible old protocol)
                    # ----------------------------------------------------------
                    if "*" in line:
                        if not self._verify_checksum(line):
                            if self.debug:
                                self.log.warning(f"Checksum failed: {repr(line)}")
                            continue
                        # remove checksum
                        line = line.split("*", 1)[0]
                    # remove '$'
                    line = line.lstrip("$")
                    # debug original frame
                    if self.debug:
                        self._debug_read(origin_line)

                    text_lower = line.lower()
                    if is_set:
                        if "ok" in text_lower:
                            return 'ok'
                        elif "error:" in text_lower:
                            r = self._parse_colon_values(text_lower, "error", int, single=True)
                            if r is not None:
                                if 'm450' in text_lower:
                                    return self._parse_mapped_error_code(
                                        r, UltraArmP1RobotInfo.ERROR_M450_MAP, self.language)
                                elif 'g11' in text_lower:
                                    return self._parse_mapped_error_code(
                                        r, UltraArmP1RobotInfo.ERROR_G11_MAP, self.language)
                                elif 'm431' in text_lower:
                                    return self._parse_mapped_error_code(
                                        r, UltraArmP1RobotInfo.ERROR_M431_MAP, self.language)
                                return r
                    # Limit error
                    if "limiterror" in text_lower:
                        res = self._parse_colon_values(text_lower, "limiterror", int, single=True)
                        if res is not None:
                            return self._parse_mapped_error_code(
                                res, UltraArmP1RobotInfo.ERROR_MOTION_MAP, self.language)

                    # Collision detection
                    if "collisiondetectionerror" in text_lower:
                        res = self._parse_colon_values(text_lower, "collisiondetectionerror", int, single=True)
                        if res is not None:
                            return self._parse_mapped_error_code(
                                res, UltraArmP1RobotInfo.ERROR_COLLISION_MAP, self.language)

                    try:
                        if text_lower.lower().count(keyword.decode()) >= 2:
                            return 'ok'
                    except Exception:
                        # fallback to raw bytes check
                        if received_data.lower().count(keyword) >= 2:
                            return 'ok'

                    if movement_status_wait:
                        # M200 returns mainmoving:0/1.  Treat 0 as the same
                        # closed-loop completion signal as the firmware's end text.
                        run_status = self._parse_colon_values(text_lower, "mainmoving", int, single=True)
                        if run_status is not None:
                            last_status_time = time.time()
                            if run_status == 0:
                                # If the robot was already in an error state before
                                # this motion command, it may only report not moving.
                                error_info = self._query_error_information()
                                if error_info and error_info != "ok":
                                    return error_info
                                return 'ok'

            if movement_status_wait:
                time.sleep(0.1)
                now = time.time()
                if now - last_status_time >= status_timeout:
                    error_info = self._query_error_information()
                    if error_info and error_info != "ok":
                        return error_info
                    break
                if now - last_status_query_time >= status_query_interval:
                    # Query run status without calling get_run_status(), since
                    # callers already hold self.lock while waiting here.
                    self._send_command(ProtocolCode.GET_RUNNING_STATUS_P1)
                    last_status_query_time = now

                time.sleep(0.01)
            elif is_set and response_timeout != wait_time:
                time.sleep(0.01)
            # time.sleep(0.01)
        # Timeout
        if self.debug:
            try:
                self.log.error(f"_timeout read: {received_data}")
            except Exception:
                self.log.error(f"_timeout received data")

        return -1

    def _request(self, flag=""):
        """
        Improved request handler:
        - clear input before reading
        - accumulate chunks
        - parse by flag until success or timeout
        """
        timeout = 1
        if flag == "check_sd_card":
            timeout = 3
        elif flag in ['angle', 'coord', 'io', 'motorenable', "get_queue_size"]:
            timeout = 0.2

        raw_data = ""
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                chunk = self._read_available_bytes()
            except Exception:
                chunk = b""

            if chunk:
                try:
                    chunk_str = chunk.decode(errors="ignore")
                    raw_data += chunk_str

                    while "\n" in raw_data:
                        # print('raw_data:', repr(raw_data))
                        line_data, raw_data = raw_data.split("\n", 1)
                        line_data = line_data.strip()

                        if not line_data:
                            continue

                        # keep original line for debug
                        origin_line_data = line_data

                        # checksum verify
                        if "*" in line_data:
                            if not self._verify_checksum(line_data):
                                if self.debug:
                                    self.log.warning(f"Checksum failed: {repr(line_data)}")
                                continue

                            # remove checksum part
                            line_data = line_data.split("*", 1)[0]

                        # remove checksum part
                        if "*" in line_data:
                            line_data = line_data.split("*", 1)[0]

                        # remove '$'
                        line_data = line_data.lstrip("$")

                        lower = line_data.lower()

                        # debug
                        if self.debug:
                            display = origin_line_data if len(origin_line_data) < 1000 else origin_line_data[-1000:]
                            self._debug_read(display)

                        # -------- dispatch by flag --------
                        if flag == "angle":
                            r = self._parse_colon_values(lower, "angles", float, 2)
                            if r is not None and len(r) ==4:
                                return r

                        elif flag == "coord":
                            r = self._parse_colon_values(lower, "coords", float, 2)
                            if r is not None and len(r) ==4:
                                return r

                        elif flag == "error_information":
                            r = self._parse_colon_values(lower, "error", int, single=True)
                            if r is not None:
                                value = r
                                return self._parse_error_code(value, self.language)

                        elif flag == "get_gripper_angle":
                            r = self._parse_colon_values(lower, "gripperangle", int, single=True)
                            if r is not None:
                                return r

                        elif flag == "zero_calibration_state":
                            r = self._parse_colon_values(lower, "zerostate", int)
                            if r is not None:
                                return r

                        elif flag == "system_version":
                            r = self._parse_colon_values(
                                lower, "getsystemversion", float, 1, single=True
                            )
                            if r is not None:
                                return r / 10

                        elif flag == "modify_version":
                            r = self._parse_colon_values(
                                lower, "getmodifyversion", int, single=True
                            )
                            if r is not None:
                                return r

                        elif flag == "get_screen_version":
                            r = self._parse_colon_values(
                                lower, "getscreenversion", float, 1, single=True
                            )
                            if r is not None:
                                return r

                        elif flag == "get_screen_modify_version":
                            r = self._parse_colon_values(
                                lower, "getscreenmodifyversion", int, single=True
                            )
                            if r is not None:
                                return r

                        elif flag == "run_status":
                            r = self._parse_colon_values(
                                lower, "mainmoving", int, single=True
                            )
                            if r is not None:
                                return r

                        elif flag == "get_gripper_run_status":
                            r = self._parse_colon_values(
                                lower, "motionstate", int, single=True
                            )
                            if r is not None:
                                return r

                        elif flag == "get_gripper_parameter":
                            r = self._parse_colon_values(
                                lower, "gripperparameters", int, single=True
                            )
                            if r is not None:
                                return r
                        elif flag == "check_sd_card":
                            if "ok" in lower:
                                return 'ok'
                            if "error:" in lower:
                                r = self._parse_colon_values(lower, "error", int, single=True)
                                if r is not None:
                                    print('SD 卡不存在' if self.language == "zh_CN" else 'SD card not present')
                                    return r
                        elif flag == "get_motor_enable_status":
                            r = self._parse_colon_values(lower, "motorenable", int)
                            if r is not None:
                                return r
                        elif flag == "get_base_io_state":
                            r = self._parse_colon_values(lower, "io", int)
                            if r is not None:
                                return r
                        elif flag == "get_end_io_state":
                            r = self._parse_colon_values(lower, "io", int)
                            if r is not None:
                                return r
                        elif flag == "get_sd_space":
                            r = self._parse_colon_values(lower, "space", int)
                            if r is not None:
                                return r
                        elif flag == "get_queue_size":
                            r = self._parse_colon_values(lower, "queue_size", int, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_sn_code':
                            r = self._parse_colon_values(lower, "sn", int, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_robot_id':
                            r = self._parse_colon_values(lower, "id", str, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_wifi_ip':
                            if 'error' in lower:
                                return None
                            r = self._parse_colon_values(lower, "ip", str, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_bluetooth_mac':
                            if 'error' in lower:
                                return None
                            r = self._parse_colon_values(lower, "mac", str, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_end_button_state':
                            if 'error' in lower:
                                return None
                            r = self._parse_colon_values(lower, "btn", int, single=True)
                            if r is not None:
                                return r
                        elif flag in ['get_bluetooth_signal_strength','get_wifi_signal_strength']:
                            if 'error' in lower:
                                return None
                            r = self._parse_colon_values(lower, "dbm", int, single=True)
                            if r is not None:
                                return r
                        elif flag == 'get_inverse_solution_angles':
                            r = self._parse_solution_values(lower, ["a", "b", "c", "d"])
                            if r is not None and len(r) ==4:
                                return r
                        elif flag == 'get_correct_solution_coords':
                            r = self._parse_solution_values(lower,["x", "y", "z", "r"])
                            if r is not None and len(r) ==4:
                                return r

                        elif flag is None:
                            return -1

                except Exception as e:
                    if self.debug:
                        self.log.error(f"socket read exception: {e}")
                    return -1
            # time.sleep(0.001)

        if self.debug:
            self.log.warning(f"request timeout, received buffer: {raw_data}")
        return -1

    def _verify_checksum(self, line):

        try:
            line = line.strip()
            if not line.startswith("$"):
                return False

            if "*" not in line:
                return False

            data, recv_checksum = line.rsplit("*", 1)
            data = data[1:]
            xor_value = 0

            for c in data:
                xor_value ^= ord(c)

            calc_checksum = f"{xor_value:02X}"

            return calc_checksum.upper() == recv_checksum.upper()

        except Exception:
            return False

    def _request_with_retry(self, command, flag, attempts=3):
        for attempt in range(attempts):
            self._send_command(command)
            result = self._request(flag)
            if result != -1:
                return result
            if self.debug and attempt < attempts - 1:
                self.log.warning(
                    f"request retry {attempt + 1}/{attempts - 1}, flag: {flag}"
                )
        return -1

    def _parse_colon_values(self, lower: str, keyword: str, value_type=float, round_ndigits=None, single=False):
        """
        Parse keyword:value1,value2,... format
        Args:
            lower (str): lower-case received buffer
            keyword (str): keyword to search (lower-case)
            value_type: int or float or str
            round_ndigits (int|None): rounding digits for float
            single (bool): return first value only
        Returns:
            list | int | float | None
        Example:
            angles:0.00,0.00,89.90,0.20
        """
        idx = lower.find(keyword)
        if idx == -1:
            return None

        colon_idx = lower.find(":", idx)
        if colon_idx == -1:
            return None

        end_idx = lower.find("\n", colon_idx)
        if end_idx == -1:
            end_idx = len(lower)

        try:
            sub = lower[colon_idx + 1:end_idx].strip()
            items = [x.strip() for x in sub.split(",") if x.strip() != ""]

            values = []
            for x in items:
                v = value_type(x)
                if value_type is float and round_ndigits is not None:
                    v = round(v, round_ndigits)
                values.append(v)

            return values[0] if single else values
        except Exception as e:
            if self.debug:
                self.log.error(f"serial read exception: {e}")
            return None

    def _parse_solution_values(self, text, keys):
        """
        Example:angle:A+0.000 B+0.000 C+90.000 D+0.000
        """
        try:
            result = []
            for key in keys:
                match = re.search(
                    rf"{key}([+-]?\d+(?:\.\d+)?)",
                    text,
                    re.IGNORECASE
                )
                if not match:
                    return None
                result.append(float(match.group(1)))
            return result
        except Exception as e:
            if self.debug:
                self.log.error(f"serial read exception: {e}")
            return None

    def _query_error_information(self, timeout=0.3):
        time.sleep(0.15)
        self._send_command(ProtocolCode.GET_ERROR_INFO_P1)

        raw_data = ""

        start_time = time.time()

        while time.time() - start_time < timeout:
            chunk = self._read_available_bytes()

            if chunk:
                try:
                    chunk_text = chunk.decode(errors="ignore")
                except Exception:
                    chunk_text = str(chunk)
                raw_data += chunk_text
                while "\n" in raw_data:
                    line_data, raw_data = raw_data.split("\n", 1)
                    line_data = line_data.strip()

                    if not line_data:
                        continue
                    # keep original frame
                    origin_line = line_data
                    # checksum verify
                    if "*" in line_data:
                        if not self._verify_checksum(line_data):
                            # if self.debug:
                            #     self.log.warning(f"Checksum failed: {repr(line_data)}")
                            continue
                        # remove checksum
                        line_data = line_data.split("*", 1)[0]
                    # remove '$'
                    line_data = line_data.lstrip("$")
                    # debug original frame
                    if self.debug:
                        self._debug_read(origin_line)

                    r = self._parse_colon_values(line_data.lower(),"error", int, single=True)

                    if r is not None:
                        return self._parse_error_code(r, self.language)
            time.sleep(0.01)
        return None

    def _parse_error_code(self, value: int, lang="en_US"):
        if value == 0:
            return "ok" if lang == "zh_CN" else "ok"

        errors = []

        for i in range(32):
            if value & (1 << i):
                info = UltraArmP1RobotInfo.ERROR_MAP.get(i)
                if info:
                    errors.append(info.get(lang, info["en_US"]))
                else:
                    errors.append(
                        f"未知错误(bit{i})" if lang == "zh_CN"
                        else f"Unknown error (bit{i})"
                    )

        return "; ".join(errors)

    def _parse_mapped_error_code(self, value: int, error_map, lang="en_US"):
        # if value == 0:
        #     return "ok"

        info = error_map.get(value)
        if info:
            return info.get(lang, info["en_US"])

        return (
            f"未知错误({value})" if lang == "zh_CN"
            else f"Unknown error ({value})"
        )

    def _append_checksum(self, command):
        """Append XOR checksum to command frame."""
        xor_value = 0
        for c in command:
            xor_value ^= ord(c)

        checksum = f"{xor_value:02X}"

        return f"${command}*{checksum}"

    def _send_command(self, command: str):
        """Send commands to serial port"""
        command = self._append_checksum(command)
        command += ProtocolCode.END
        self._debug_write(command)
        try:
            self.sock.sendall(command.encode())
        except Exception as e:
            self.log.error(f"socket send error: {e}")

    def _wait_queue_safe(self, timeout=5.0):
        start = time.time()

        last_queue_size = None

        while True:

            retry = 0
            valid_queue_size = None

            while retry < 3:
                queue_size = self.get_queue_size()
                # print("M600 Queue_size:", queue_size)

                # Basic anomalies
                if queue_size is None or queue_size < 0:
                    retry += 1
                    time.sleep(0.02)
                    continue

                # Out-of-range anomaly
                if queue_size > 100:
                    # print("queue size overflow", queue_size)
                    retry += 1
                    time.sleep(0.005)
                    continue

                # The difference from the previous value is too large.
                if last_queue_size is not None and abs(queue_size - last_queue_size) >= 10:
                    # print("current & last too large", queue_size, last_queue_size)
                    retry += 1
                    time.sleep(0.005)
                    continue

                # Truly effective data
                valid_queue_size = queue_size
                break

            # Continuous anomalies
            if valid_queue_size is None:
                print("queue size abnormal, exit play")
                return False

            last_queue_size = valid_queue_size
            queue_size = valid_queue_size

            # Unblocked
            if not self._queue_blocked:
                if queue_size >= 80:
                    self._queue_blocked = True
                    continue
                else:
                    return True
            # Blocked
            else:
                if queue_size <= 40:
                    self._queue_blocked = False
                    return True

            time.sleep(0.01)

    def _normalize_gcode_line(self, line):
        line = line.strip()

        if not line or line.startswith(";"):
            return None

        tokens = line.strip().split()

        if tokens[0].upper() == "G0":
            tokens[0] = "G1"

        return " ".join(tokens)

    def finish_firmware_upgrade(self):
        """Download complete"""
        command = ProtocolCode.FINISH_DOWNLOAD_FIRMWARE
        self._send_command(command)

    # ---------------------- Control methods ----------------------
    def set_reboot(self):
        """Reboot the robot controller board.(Internal Interface)"""
        with self.lock:
            self._send_command(ProtocolCode.SET_REBOOT)
            return self._response(_async=True, is_set=True)

    def set_joint_release(self, joint_id):
        """release the robot joints.
        Args:
            joint_id (int): Joint number (1~4). 0 for all joints."""
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=joint_id)
        with self.lock:
            command = ProtocolCode.SET_JOINT_DISABLE
            command += f" J{joint_id}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_joint_enable(self, joint_id):
        """Enable the robot joints.
        Args:
            joint_id (int): Joint number (1~4). 0 for all joints."""
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id=joint_id)
        with self.lock:
            command = ProtocolCode.SET_JOINT_ENABLE
            command += f" J{joint_id}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_angles_info(self):
        """Get the current joint angles of the robot.

        Returns:
            list[float] or int: Joint angles [J1, J2, J3, J4] or -1 if failed.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_ANGLES_P1, "angle")

    def get_coords_info(self):
        """Get the current Cartesian coordinates of the robot.

        Returns:
            list[float] or int: Coordinates [X, Y, Z, E] or -1 if failed.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_COORDS_P1, "coord")

    def set_coords_max_speed(self, coords, _async=True, _gcode=False):
        """The robot moves at its maximum speed using Cartesian coordinates.

        Args:
            coords (list[float]): Coordinates [X, Y, Z, RX].
            _async: (bool): Closed-loop switch
            _gcode: (bool): GCode switch
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords)
        with self.lock:
            command = ProtocolCode.SET_COORDS_MAX_SPEED
            if len(coords) > 0 and coords[0] is not None:
                command += f" X{coords[0]}"
            if len(coords) > 1 and coords[1] is not None:
                command += f" Y{coords[1]}"
            if len(coords) > 2 and coords[2] is not None:
                command += f" Z{coords[2]}"
            if len(coords) > 3 and coords[3] is not None:
                command += f" R{coords[3]}"

            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_coords(self, coords, speed, _async=True, _gcode=False):
        """Move the robot using Cartesian coordinate control.

        Args:
            coords (list[float]): Coordinates [X, Y, Z].
            speed (int): Movement speed (1~100).
            _async: (bool): Closed-loop switch
            _gcode: (bool): GCode switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_COORDS
            if len(coords) > 0 and coords[0] is not None:
                command += f" X{coords[0]}"
            if len(coords) > 1 and coords[1] is not None:
                command += f" Y{coords[1]}"
            if len(coords) > 2 and coords[2] is not None:
                command += f" Z{coords[2]}"
            if len(coords) > 3 and coords[3] is not None:
                command += f" R{coords[3]}"
            if speed is not None and 1 <= speed <= 100:
                command += f" F{speed}"

            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_coord(self, coord_id, coord, speed, _async=True, _gcode=False):
        """Set single coordinate.

        Args:
            coord_id (str): 'X', 'Y', 'Z', 'R'
            coord (float): coordinate value
            speed (int): movement speed 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__,coord_id=coord_id,coord=coord,speed=speed)
        with self.lock:
            command = ProtocolCode.SET_COORDS
            command += f" {coord_id}{coord}"
            command += f" F{speed}"
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_angle(self, joint_id, angle, speed, _async=True, _gcode=False):
        """Set a single joint angle.

        Args:
            joint_id (int): Joint number (1~4).
            angle (float): Angle value.
            speed (int): Movement speed (1~100).
            _async: (bool): Closed-loop switch
            _gcode: (bool): Closed-loop switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, angle=angle, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_ANGLE_P1
            joint_map = {1: "A", 2: "B", 3: "C", 4: "D"}
            if joint_id in joint_map:
                command += f" {joint_map[joint_id]}{angle}"
            if speed > 0:
                command += f" F{speed}"
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_angles(self, angles, speed, _async=True, _gcode=False):
        """Move robot using joint angle control.

        Args:
            angles (list[float]): Joint angles [J1, J2, J3, J4].
            speed (int): Movement speed (1~100).
            _async: (bool): Closed-loop switch
            _gcode: (bool): Closed-loop switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_ANGLES_P1
            if len(angles) > 0 and angles[0] is not None:
                command += f" A{angles[0]}"
            if len(angles) > 1 and angles[1] is not None:
                command += f" B{angles[1]}"
            if len(angles) > 2 and angles[2] is not None:
                command += f" C{angles[2]}"
            if len(angles) > 3 and angles[3] is not None:
                command += f" D{angles[3]}"
            if speed is not None and 1 <= speed <= 100:
                command += f" F{speed}"

            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def get_system_version(self):
        """Get system firmware version

        Returns:
            (float) Firmware version
        """
        with self.lock:
            return  self._request_with_retry(ProtocolCode.GET_SYSTEM_VERSION_P1, "system_version")

    def get_modify_version(self):
        """Get firmware modify version

        Returns:
            (int) modify version
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_MODIFY_VERSION_P1, "modify_version")

    def stop(self):
        """Stop movement"""
        with self.lock:
            self._send_command(ProtocolCode.SET_STOP_P1)
            return self._response(_async=True, is_set=True)

    def set_jog_angle(self, joint_id, direction, speed, _async=True, _gcode=False):
        """Start jog movement with angle

        Args:
            joint_id : 1 ~ 4

            direction :
                0 : Negative motion
                1 : Positive motion
            speed : (int) 1-100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id, direction=direction,
                                    jog_speed=speed)
        with self.lock:
            command = ProtocolCode.SET_JOG_ANGLE_P1
            command += " J" + str(joint_id)
            command += " D" + str(direction)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_jog_coord(self, axis_id, direction, speed, _async=True, _gcode=False):
        """Start jog movement with coord

        Args:
            axis_id(int) : axis 1-X, 2-Y, 3-Z, 4-RX

            direction:
                0 : Negative motion
                1 : Positive motion
            speed : (int) 1-100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, axis_id=axis_id, direction=direction,
                                    jog_speed=speed)
        with self.lock:
            command = ProtocolCode.SET_JOG_COORD_P1
            command += " J" + str(axis_id)
            command += " D" + str(direction)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def jog_increment_angle(self, joint_id, increment, speed, _async=True, _gcode=False):
        """Single angle incremental motion control.

        Args:
            joint_id: Joint id 1 - 4
            increment: Angle increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, increment_angle=increment, jog_speed=speed)
        with self.lock:
            command = ProtocolCode.JOG_INCREMENT_ANGLE_P1
            command += " J" + str(joint_id)
            command += " T" + str(increment)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def jog_increment_coord(self, coord_id, increment, speed, _async=True, _gcode=False):
        """Single coordinate incremental motion control.

        Args:
            coord_id: axis id 1 - 4.
            increment: Coord increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, jog_coord_id=coord_id, increment_coord=increment, speed=speed)
        with self.lock:
            command = ProtocolCode.JOG_INCREMENT_COORD_P1
            command += " J" + str(coord_id)
            command += " T" + str(increment)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def get_error_information(self):
        """Read error message"""
        with self.lock:
            return self._request_with_retry(
                ProtocolCode.GET_ERROR_INFO_P1, "error_information"
            )

    def set_zero_calibration(self, joint_number):
        """Set zero-point calibration.

        Args:
            joint_number (int) : 0 ~ 4
                0 : All joint
                1: J1
                2: J2
                3: J3
                4: J4
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_number=joint_number)
        with self.lock:
            command = ProtocolCode.SET_JOINT_ZERO_CALIBRATION_P1
            command += " J" + str(joint_number)
            self._send_command(command)
            return self._response(_async=True)

    def get_zero_calibration_state(self):
        """Read zero-point calibration status.

        Returns:
            (list) zero-point calibration status, len 4
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_BACK_ZERO_STATUS_P1, "zero_calibration_state")

    def set_joint1_encoder_calibration(self):
        """Set the 730 encoder calibration for J1.(Internal Interface)"""
        with self.lock:
            self._send_command(ProtocolCode.SET_J1_ENCODER_CALIBRATION_P1)
            return self._response(_async=True, is_set=True)

    def get_run_status(self):
        """Read running status."""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_RUNNING_STATUS_P1, "run_status")

    def set_pwm_laser_mode(self, state):
        """Set PWM Laser mode.

        Args:
            state (int): 0 - close; 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            command = ProtocolCode.SET_PWM_LASER_MODE
            command += " K" + str(state)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_pwm_laser(self, p_value):
        """Set PWM Level - Laser

        Args:
            p_value (int) : Duty cycle 0 ~ 255;
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__,  p_value=p_value)
        with self.lock:
            command = ProtocolCode.SET_PWM_LASER
            command += " S" + str(p_value)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_pwm_custom_mode(self, state):
        """Set PWM custom mode

        Args:
            state (int): 0 - close; 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            command = ProtocolCode.SET_PWM_CUSTOM_MODE
            command += " K" + str(state)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_pwm_custom(self, p_value):
        """Set PWM Level - Custom

        Args:
            p_value (int) : Duty cycle 0 ~ 255;
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__,  p_value=p_value)
        with self.lock:
            command = ProtocolCode.SET_PWM_CUSTOM
            command += " S" + str(p_value)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_gripper_angle(self, gripper_angle, gripper_speed):
        """Set gripper angle.

        Args:
            gripper_angle (int): 1 - 100

            gripper_speed(int): 1 - 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle,
                                    gripper_speed=gripper_speed)
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_ANGLE_P1
            command += " P" + str(gripper_angle)
            command += " F" + str(gripper_speed)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_gripper_angle(self):
        """Read gripper angle.

        Returns: (int) gripper angle.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_GRIPPER_ANGLE_P1, "get_gripper_angle")

    def set_gripper_parameter(self, addr, parameter_value):
        """Set gripper parameter

        Args:
            addr (int) : 1 ~ 69
            parameter_value (int) : 0 ~ 65535
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_addr=addr, parameter_value=parameter_value)
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_PARAMETER_P1
            command += " J" + str(addr)
            command += " L" + str(parameter_value)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_gripper_parameter(self, addr):
        """Get gripper parameter.

        Args:
            addr (int) : 1 ~ 69

        Returns: (int) gripper parameter. 0 ~ 65535
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_addr=addr)
        with self.lock:
            command = ProtocolCode.GET_GRIPPER_PARAMETER_P1
            command += " J" + str(addr)
            return self._request_with_retry(command, "get_gripper_parameter")

    def get_gripper_run_status(self):
        """Get gripper running status.

        Returns: gripper status.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_GRIPPER_RUN_STATUS_P1,"get_gripper_run_status")

    def set_gripper_enable_status(self, state):
        """set gripper enable status.

        Args:
            state (int) :
                0 - disabled
                1 - enabled
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_ENABLE_STATUS_P1
            command += " S" + str(state)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_gripper_zero(self):
        """Set gripper zero."""
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_ZERO_P1
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_pump_state(self, pump_state):
        """Set the suction pump's on/off state.

        Args:
            pump_state (int) :
                0 - open
                1 - release
                2 - closed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pump_state=pump_state)
        with self.lock:
            command = ProtocolCode.SET_PUMP_STATE_P1
            command += " S" + str(pump_state)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_base_io_output(self, pin_no, pin_status, pin_signal):
        """Set the status of the base output pin.

        Args:
            pin_no (int) : 1 ~ 10
            pin_status (int) : 0 ~ 1
                0 - input
                1 - output
            pin_signal (int) : 0 ~ 1
                0 - Low level
                1 - High level
        """
        self.calibration_parameters(class_name=self.__class__.__name__, basic_pin_no=pin_no, basic_pin_status=pin_status, pin_signal=pin_signal)
        with self.lock:
            command = ProtocolCode.SET_BASIC_OUTPUT_P1
            command += " P" + str(pin_no)
            command += " K" + str(pin_status)
            command += " S" + str(pin_signal)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_digital_io_output(self, pin_no, pin_signal):
        """Set the state of the end output pin.

        Args:
            pin_no (int) : 3 ~ 4
            pin_signal (int) : 0 ~ 1
                0 - Low level
                1 - High level
        """
        self.calibration_parameters(class_name=self.__class__.__name__, set_end_pin_no=pin_no, pin_signal=pin_signal)
        with self.lock:
            command = ProtocolCode.SET_DIGITAL_OUTPUT_P1
            command += " P" + str(pin_no)
            command += " S" + str(pin_signal)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_i2c_data(self, data_state, data_addr, data_len, data_value):
        """Set i2c data.

        Args:
            data_state (int) : 0 ~ 1
                0 - read
                1 - write
            data_addr (int) : 0 ~ 255
            data_len (int) : 0 ~ 64
            data_value (int) : 0 ~ 255
        """
        self.calibration_parameters(class_name=self.__class__.__name__, data_state=data_state,
                                    data_addr=data_addr, data_len=data_len, data_value=data_value)
        with self.lock:
            command = ProtocolCode.SET_I2C_P1
            command += " S" + str(data_state)
            command += " L" + str(data_addr)
            command += " N" + str(data_len)
            command += " M" + str(data_value)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def play_gcode_file(self, filename):
        """Play the imported track file

        Args:
            filename (str): Path to a G-code file (.gcode or .nc or .ngc)
        """

        self.calibration_parameters(
            class_name=self.__class__.__name__,
            filename=filename
        )

        try:
            with open(filename) as f:
                lines = f.readlines()
        except Exception:
            print("There is no such file!")
            return

        with self.lock:
            for raw_line in lines:
                line = self._normalize_gcode_line(raw_line)

                if line is None:
                    continue

                command = line + ProtocolCode.END
                if self._wait_queue_safe() != 1:
                    print("queue play error")
                    break
                self.sock.sendall(command.encode())
                time.sleep(0.02)
                self._debug_write(command)

    def get_system_screen_version(self):
        """Read system screen version.

        Returns: (float) screen version.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_SYSTEM_SCREEN_VERSION_P1,"get_screen_version")

    def get_screen_modify_version(self):
        """Read screen modify version.

        Returns: (float) modify screen version.
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_MODIFY_SCREEN_VERSION_P1,"get_screen_modify_version")

    def set_communication_baud_rate(self, baud_rate):
        """set communication baud rate

        Args:
            baud_rate (int) : 115200 or 1000000
            """
        self.calibration_parameters(class_name=self.__class__.__name__, baud_rate=baud_rate)
        with self.lock:
            command = ProtocolCode.SET_COMMUNICATION_BAUD_RATE_P1
            command += " B" + str(baud_rate)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def go_home(self, speed=20, _async=True):
        return self.set_angles([0, 0, 90, 0], speed, _async=_async)

    def close(self):
        """Close the socket connect."""
        with self.lock:
            try:
                self.sock.close()
            except Exception:
                pass

    def open(self):
        """Open the socket connect."""
        with self.lock:
            try:
                self.sock = self.connect_socket()
            except Exception:
                pass

    def set_wifi_password(self,  wifi_name, password):
        """Set WiFi password

        Args:
            wifi_name (str) : ssid, WiFi name
            password (str) : WiFi password
        """
        self.calibration_parameters(class_name=self.__class__.__name__, password=password)
        with self.lock:
            command = ProtocolCode.SET_WIFI_PASSWORD
            command += " " + str(wifi_name) + '|' + str(password)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def check_sd_card(self):
        """Check if there is an SD card."""
        with self.lock:
            command = ProtocolCode.CHECK_SD_CARD
            return self._request_with_retry(command, "check_sd_card")

    def upgrade_restart(self):
        """Upgrade and restart"""
        with self.lock:
            self._send_command(ProtocolCode.UPGRADE_RESTART)
            return self._response(_async=True, is_set=True)

    def get_motor_enable_status(self):
        """Retrieve motor enable status"""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_MOTOR_ENABLE_STATUS,'get_motor_enable_status')

    def clear_zero_calibration_status(self, joint_id):
        """Clear zero calibration status

        Args:
            joint_id (int): joint ID, range is 1 ~ 4
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id)
        with self.lock:
            command = ProtocolCode.CLEAR_ZERO_CALIBRATION_STATUS
            command += " J" + str(joint_id)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_all_base_io_states(self):
        """Get All bottom I/O pin status.

        Returns:
            pin_status (list) : List of numbers in the range 0 to 3, len is 10
                0: Input, level = 0 (low level)
                1: Input, level = 1 (high level)
                2: Output, level = 0 (low level)
                3: Output, level = 1 (high level)
        """
        with self.lock:
            return self._request_with_retry(
                ProtocolCode.GET_BASE_IO_STATE_P1, 'get_base_io_state'
            )

    def get_base_io_state(self, pin_no):
        """Get bottom I/O pin status.

        Args:
            pin_no (int): bottom pin number, range is 1 ~ 10
        Returns:
            pin_status (int) : range 0 to 3
                0: Input, level = 0 (low level)
                1: Input, level = 1 (high level)
                2: Output, level = 0 (low level)
                3: Output, level = 1 (high level)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, basic_pin_no=pin_no)
        with self.lock:
            res_data = self._request_with_retry(
                ProtocolCode.GET_BASE_IO_STATE_P1, 'get_base_io_state'
            )
            if isinstance(res_data, list):
                return res_data[pin_no - 1]
            return -1

    def get_all_end_io_states(self):
        """Get all end I/O pin status.

        Returns:
            pin_status (list) : List of numbers in the range 0 to 3, len is 4
                0: Input, level = 0 (low level)
                1: Input, level = 1 (high level)
                2: Output, level = 0 (low level)
                3: Output, level = 1 (high level)
        """
        with self.lock:
            return self._request_with_retry(
                ProtocolCode.GET_END_IO_STATE_P1, 'get_end_io_state'
            )

    def get_end_io_state(self, pin_no):
        """Get end I/O pin status.

        Args:
            pin_no (int): end pin number, range is 1 ~ 4
        Returns:
            pin_status (int) : range 0 to 3
                0: Input, level = 0 (low level)
                1: Input, level = 1 (high level)
                2: Output, level = 0 (low level)
                3: Output, level = 1 (high level)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, end_pin_no=pin_no)
        with self.lock:
            res_data = self._request_with_retry(
                ProtocolCode.GET_END_IO_STATE_P1, 'get_end_io_state'
            )
            if isinstance(res_data, list):
                return res_data[pin_no - 1]
            return -1

    def set_end_button_disable(self):
        """Disable the settings end button."""
        with self.lock:
            self._send_command(ProtocolCode.SET_BUTTON_DISABLE)
            return self._response(_async=True, is_set=True)

    def set_end_button_enable(self):
        """Enable the settings end button."""
        with self.lock:
            self._send_command(ProtocolCode.SET_BUTTON_ENABLE)
            return self._response(_async=True, is_set=True)

    def forced_reset_zero(self):
        """Forced reset to zero."""
        with self.lock:
            self._send_command(ProtocolCode.FORCED_RESET_ZERO)
            return self._response(_async=True)

    def set_conveyor_control(self, state, direction, speed, distance):
        """Conveyor belt control.

        Args:
            state (int): 0 ~ 1, Conveyor belt state, 0 - close; 1 - open
            direction (int): 0 ~ 1, Conveyor belt direction, 0 - forward; 1 - backward
            speed (int): Conveyor belt speed (50~500000)
            distance (int): Conveyor belt distance (1~500000)
        """
        with self.lock:
            command = ProtocolCode.CONVEYOR_BELT_CONTROL
            command += " J" + str(state)
            command += " K" + str(direction)
            command += " L" + str(speed)
            command += " S" + str(distance)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_color(self, r, g, b):
        """Set the color of the RGB light panel

        Args:
            r (int): Red color, range is 0 ~ 255
            g (int): Green color, range is 0 ~ 255
            b (int): Blue color, range is 0 ~ 255
        """
        self.calibration_parameters(class_name=self.__class__.__name__, rgb=[r, g, b])
        with self.lock:
            command = ProtocolCode.SET_RGB_COLOR
            command += " R" + str(r)
            command += " G" + str(g)
            command += " B" + str(b)
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_preview_mode(self, coords):
        """Set Coordinate Trajectory Preview Mode

        Args:
            coords (list[float]): Coordinates [X, Y, Z, R].
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords)
        with self.lock:
            command = ProtocolCode.SET_PREVIEW_MODE
            if len(coords) > 0 and coords[0] is not None:
                command += f" X{coords[0]}"
            if len(coords) > 1 and coords[1] is not None:
                command += f" Y{coords[1]}"
            if len(coords) > 2 and coords[2] is not None:
                command += f" Z{coords[2]}"
            if len(coords) > 3 and coords[3] is not None:
                command += f" R{coords[3]}"

            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_sd_card_space(self):
        """Get SD Card Total and Remaining Memory

        Returns:
            space (list) : Total Memory and Remaining Memory, For example: [Total Memory, Remaining Memory]
        """
        with self.lock:
            return self._request_with_retry(
                ProtocolCode.GET_SD_CARD_MEMORY, 'get_sd_space'
            )

    def collision_unlock(self):
        """Unlock After Collision Detection."""
        with self.lock:
            self._send_command(ProtocolCode.COLLISION_UNLOCK)
            return self._response(_async=True, is_set=True)

    def clear_error_status(self):
        """Clear Error Status."""
        with self.lock:
            self._send_command(ProtocolCode.CLEAR_ERROR_STATUS)
            return self._response(_async=True, is_set=True)

    def get_queue_size(self):
        """Get Buffer Queue Size.

        Returns:
            `int` queue size
        """
        return self._request_with_retry(
            ProtocolCode.GET_QUEUE_SIZE_P1, 'get_queue_size')

    def set_sn_code(self, sn_code):
        """Set SN Code.

        Args:
            sn_code (str): SN Code, len is 11
        """
        self.calibration_parameters(class_name=self.__class__.__name__, sn_code=sn_code)
        with self.lock:
            command = ProtocolCode.SET_SN_CODE
            command += f" {str(sn_code)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_sn_code(self):
        """Get SN Code."""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_SN_CODE, 'get_sn_code')

    def set_robot_id(self, robot_id):
        """Set Robot ID.

        Args:
              robot_id (str): Robot ID, len is 3
        """
        self.calibration_parameters(class_name=self.__class__.__name__, robot_id=robot_id)
        with self.lock:
            command = ProtocolCode.SET_ROBOT_ID_P1
            command += f" {str(robot_id)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def get_robot_id(self):
        """Get Robot ID."""

        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_ROBOT_ID_P1, 'get_robot_id')

    def get_wifi_ip(self):
        """Get WiFi IP Address"""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_WIFI_IP_PORT_P1, 'get_wifi_ip')

    def get_bluetooth_mac(self):
        """Get Bluetooth MAC."""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_BLUETOOTH_MAC_P1, 'get_bluetooth_mac')

    def get_end_button_state(self):
        """Get end button status.
        Returns:
            1 - pressed
            0 - released
        """
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_END_BUTTON_STATUS, 'get_end_button_state')

    def get_wifi_signal_strength(self):
        """Get WiFi signal strength."""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_WIFI_SIGNAL_P1, 'get_wifi_signal_strength')

    def get_bluetooth_signal_strength(self):
        """Get Bluetooth signal strength."""
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_BLUETOOTH_SIGNAL_P1, 'get_bluetooth_signal_strength')

    def set_modbus_mode(self, state):
        """Set modbus mode.

        Args:
              state (int): 0 - close; 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            command = ProtocolCode.SET_MODBUS_MODE_P1
            command += f" S{str(state)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def set_collision_threshold(self, joint_id, threshold):
        """Set collision threshold.

        Args:
            joint_id (int): Joint ID, 1 ~ 4; 0 - all joints
            threshold (float): Collision threshold
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_number=joint_id, threshold_value=threshold)
        with self.lock:
            command = ProtocolCode.SET_COLLISION_THRESHOLD_P1
            command += f"J {str(joint_id)}"
            command += f"P {str(threshold)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)
