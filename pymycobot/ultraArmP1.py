#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ultraArmP1.py

Python interface for the ultraArmP1 robotic arm.

Author: weijian.wang
Date: 2025-11-25
Description: None
"""

import threading
import time
import datetime

from pymycobot.common import ProtocolCode
from pymycobot.error import calibration_parameters


class UltraArmP1:
    """Class for controlling the ultraArmP1 robotic arm via serial communication.

    """

    def __init__(self, port, baudrate=1000000, timeout=0.05, debug=False):
        """Initialize the ultraArmP1 robot communication.

        Args:
            port (str): Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int, optional): Communication baud rate. Defaults to 1000000.
            timeout (float, optional): Serial read timeout in seconds. Defaults to 0.05.
            debug (bool, optional): Whether to print debug information. Defaults to False.
        """
        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.dtr = True
        self._serial_port.open()
        self.debug = debug
        self.calibration_parameters = calibration_parameters
        self.lock = threading.Lock()
        time.sleep(0.5)

    # ---------------------- Debug / time helpers ----------------------
    def _now(self):
        """Return timestamp string with millisecond precision."""
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _debug_write(self, data: str):
        if self.debug:
            print(f"{self._now()} DEBU [UltraArmP1] _write: {data}")

    def _debug_read(self, data: str):
        if self.debug:
            print(f"{self._now()} DEBU [UltraArmP1] _read : {data}")

    # ---------------------- Serial helpers ----------------------
    def _serial_in_waiting(self):
        try:
            return int(self._serial_port.in_waiting)
        except Exception:
            try:
                return int(self._serial_port.inWaiting())
            except Exception:
                return 0

    def _read_available_bytes(self):
        n = self._serial_in_waiting()
        if n <= 0:
            return b""
        try:
            return self._serial_port.read(n)
        except Exception:
            return b""

    # ---------------------- Response waiting & parsing ----------------------
    def _response(self, timeout=90, _async=True, _gcode=False):
        """Wait for device response from the serial buffer.

        Returns 'ok' when keyword is found, False on timeout.
        """
        if _gcode:
            _async = False

        if not _async and not _gcode:
            return 1
        start_time = time.time()
        received_data = b""
        if _gcode:
            keyword = b"start"
        else:  # _async=True
            keyword = b"end"
        no_data_timeout = 1
        last_data_time = time.time()
        while time.time() - start_time < timeout:
            chunk = self._read_available_bytes()
            # print('chunk:', chunk)
            # if not chunk:
            #     return -1
            if chunk:
                received_data += chunk
                # try decode for debug
                try:
                    text = received_data.decode(errors='ignore')
                except Exception:
                    text = str(received_data)
                self._debug_read(text)

                try:
                    if text.lower().count(keyword.decode()) >= 1:
                        return 'ok'
                except Exception:
                    # fallback to raw bytes check
                    if received_data.lower().count(keyword) >= 1:
                        return 'ok'
            # else:
                # print('no data')
            #     if time.time() - last_data_time > no_data_timeout:
            #         return -1
            time.sleep(0.01)
        # Timeout
        if self.debug:
            try:
                print(f"{self._now()} DEBU [UltraArmP1] _timeout : {received_data}")
            except Exception:
                print(f"{self._now()} DEBU [UltraArmP1] _timeout : <binary>")

        return False

    def _request(self, flag=""):
        """
        Improved request handler:
        - clear input before reading
        - accumulate chunks
        - parse by flag until success or timeout
        """
        timeout = 0.1
        if flag == "check_sd_card":
            timeout = 3

        raw_data = ""
        start_time = time.time()
        # clear stale input
        # try:
        #     if hasattr(self._serial_port, "reset_input_buffer"):
        #         self._serial_port.reset_input_buffer()
        #     elif hasattr(self._serial_port, "flushInput"):
        #         self._serial_port.flushInput()
        # except Exception:
        #     pass
        self._clear_serial_buffer()

        while time.time() - start_time < timeout:
            try:
                n = self._serial_port.inWaiting()
            except Exception:
                n = 0

            if n > 0:
                try:
                    chunk = self._serial_port.read(n)
                    chunk_str = chunk.decode(errors="ignore")
                    raw_data += chunk_str
                    if self.debug:
                        display = raw_data if len(raw_data) < 1000 else raw_data[-1000:]
                        self._debug_read(display)
                    lower = raw_data.lower()
                    # common error
                    if "error: command not recognized" in lower:
                        return -1
                    # -------- dispatch by flag --------
                    if flag == "angle":
                        start_time = time.time()
                        r = self._parse_bracket_values(lower, "angles", float, 2)
                        if r is not None:
                            return r

                    elif flag == "coord":
                        r = self._parse_bracket_values(lower, "coords", float, 2)
                        if r is not None:
                            return r

                    elif flag == "error_information":
                        r = self._parse_bracket_values(lower, "error", int)
                        if r is not None:
                            return r

                    elif flag == "get_gripper_angle":
                        r = self._parse_bracket_values(lower, "gripperangle", int, single=True)
                        if r is not None:
                            return r

                    elif flag == "zero_calibration_state":
                        r = self._parse_bracket_values(lower, "zero state", int)
                        if r is not None:
                            return r

                    elif flag == "system_version":
                        r = self._parse_bracket_values(
                            lower, "getsystemversion", float, 1, single=True
                        )
                        if r is not None:
                            return r / 10

                    elif flag == "modify_version":
                        r = self._parse_bracket_values(
                            lower, "getmodifyversion", int, single=True
                        )
                        if r is not None:
                            return r

                    elif flag == "get_screen_version":
                        r = self._parse_bracket_values(
                            lower, "getscreenversion", float, 1, single=True
                        )
                        if r is not None:
                            return r

                    elif flag == "get_screen_modify_version":
                        r = self._parse_bracket_values(
                            lower, "getscreenmodifyversion", float, 1, single=True
                        )
                        if r is not None:
                            return r

                    elif flag == "run_status":
                        r = self._parse_bracket_values(
                            lower, "mainmoving", int, single=True
                        )
                        if r is not None:
                            return r

                    elif flag == "get_gripper_run_status":
                        r = self._parse_bracket_values(
                            lower, "motionstate", int, single=True
                        )
                        if r is not None:
                            return r

                    elif flag == "get_gripper_parameter":
                        r = self._parse_bracket_values(
                            lower, "gripperparameters", int, single=True
                        )
                        if r is not None:
                            return r
                    elif flag == "check_sd_card":
                        r = self._parse_bracket_values(lower, "sdcard", str, single=True)
                        if r is not None:
                            return r

                    elif flag is None:
                        return -1

                except Exception as e:
                    if self.debug:
                        print(f"{self._now()} DEBU [UltraArmP1] _error : serial read exception: {e}")
            time.sleep(0.001)

        if self.debug:
            print(
                f"{self._now()} DEBU [UltraArmP1] _warn : request timeout, received buffer: {raw_data!r}"
            )
        return -1

    def _parse_bracket_values(self, lower: str, keyword: str, value_type=float, round_ndigits=None, single=False):
        """
        Parse keyword[...] values from lower string.

        Args:
            lower (str): lower-case received buffer
            keyword (str): keyword to search (lower-case)
            value_type: int or float or str
            round_ndigits (int|None): rounding digits for float
            single (bool): return first value only

        Returns:
            list | int | float | None
        """
        idx = lower.find(keyword)
        if idx == -1:
            return None

        bracket_start = lower.find("[", idx)
        bracket_end = lower.find("]", idx)
        if bracket_start == -1 or bracket_end == -1 or bracket_end <= bracket_start:
            return None

        try:
            sub = lower[bracket_start + 1: bracket_end]
            items = [x.strip() for x in sub.split(",") if x.strip() != ""]

            values = []
            for x in items:
                v = value_type(x)
                if value_type is float and round_ndigits is not None:
                    v = round(v, round_ndigits)
                values.append(v)

            return values[0] if single else values
        except Exception:
            return None

    def _send_command(self, command: str):
        """Send commands to serial port"""
        command += ProtocolCode.END
        self._debug_write(command)
        self._serial_port.write(command.encode())
        self._serial_port.flush()

    def _clear_serial_buffer(self):
        """Clear the serial port buffer before sending commands."""
        try:
            if hasattr(self._serial_port, "reset_input_buffer"):
                self._serial_port.reset_input_buffer()
        except Exception:
            pass

    def _fw_calc_crc(self, payload: bytes):
        """
        CRC = sum(CMD + IDX_H + IDX_L + LEN_H + LEN_L + DATA) & 0xFF
        """
        return sum(payload) & 0xFF

    def _fw_build_packet(self, idx: int, data: bytes):
        """Build data packets"""
        frame = bytearray()
        frame += b'\xA5\x5A'  # Frame header
        frame += b'\x01'  # CMD: PC send data
        frame += idx.to_bytes(2, 'big')  # Packet index
        frame += len(data).to_bytes(2, 'big')
        frame += data

        crc = self._fw_calc_crc(frame[2:])  # exclude header
        frame.append(crc)
        return bytes(frame)

    def _fw_read_ack(self, timeout=1.0):
        """Read screen response data"""
        start = time.time()
        buf = bytearray()

        while time.time() - start < timeout:
            n = self._serial_in_waiting()
            if n > 0:
                buf += self._serial_port.read(n)

                # At least 8 bytes are needed for an ACK.
                while len(buf) >= 8:
                    if buf[0:2] != b'\xA5\x5A':
                        buf.pop(0)
                        continue

                    frame = bytes(buf[:8])
                    buf[:] = buf[8:]
                    self._debug_read(frame.hex(' ').upper())
                    cmd = frame[2]
                    idx = int.from_bytes(frame[3:5], 'big')
                    return cmd, idx

            time.sleep(0.002)

        return None

    def _fw_enter_upgrade(self, filename: str):
        """Start downloading"""
        command = ProtocolCode.START_DOWNLOAD_FIRMWARE
        command += f" {filename}"
        self._send_command(command)

    def _fw_finish_upgrade(self):
        """Download complete"""
        command = ProtocolCode.FINISH_DOWNLOAD_FIRMWARE
        self._send_command(command)

    def _download_progress(self, percent):
        print(f"Download progress: {percent}%")

    # ---------------------- Control methods ----------------------
    def set_reboot(self):
        """Reboot the robot controller board.(Internal Interface)"""
        with self.lock:
            self._send_command(ProtocolCode.SET_REBOOT)
            return self._response(_async=False)

    def set_joint_release(self):
        """release the robot joints."""
        with self.lock:
            self._send_command(ProtocolCode.SET_JOINT_DISABLE)
            return self._response(_async=False)

    def set_joint_enable(self):
        """Enable the robot joints."""
        with self.lock:
            self._send_command(ProtocolCode.SET_JOINT_ENABLE)
            return self._response(_async=False)

    def get_angles_info(self):
        """Get the current joint angles of the robot.

        Returns:
            list[float] or int: Joint angles [J1, J2, J3, J4] or -1 if failed.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_JOINT_ANGLES_COORDS)
            return self._request("angle")

    def get_coords_info(self):
        """Get the current Cartesian coordinates of the robot.

        Returns:
            list[float] or int: Coordinates [X, Y, Z, E] or -1 if failed.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_JOINT_ANGLES_COORDS)
            return self._request("coord")

    def set_coords_max_speed(self, coords, _async=True, _gcode=False):
        """The robot moves at its maximum speed using Cartesian coordinates.

        Args:
            coords (list[float]): Coordinates [X, Y, Z, RX].
            _async: (bool): Closed-loop switch
            _gcode: (bool): GCode switch
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords)
        with self.lock:
            self._clear_serial_buffer()
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
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
            _gcode: (bool): GCode switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords, speed=speed)
        with self.lock:
            self._clear_serial_buffer()
            command = ProtocolCode.SET_COORDS
            if len(coords) > 0 and coords[0] is not None:
                command += f" X{coords[0]}"
            if len(coords) > 1 and coords[1] is not None:
                command += f" Y{coords[1]}"
            if len(coords) > 2 and coords[2] is not None:
                command += f" Z{coords[2]}"
            if len(coords) > 3 and coords[3] is not None:
                command += f" R{coords[3]}"
            if speed is not None and 1 <= speed <= 5700:
                command += f" F{speed}"

            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_coord(self, coord_id, coord, speed, _async=True, _gcode=False):
        """Set single coordinate.

        Args:
            coord_id (str): 'X', 'Y', 'Z', 'R'
            coord (float): coordinate value
            speed (int): movement speed 1 ~ 5700
        """
        self.calibration_parameters(class_name=self.__class__.__name__,coord_id=coord_id,coord=coord,speed=speed)
        with self.lock:
            self._clear_serial_buffer()
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
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
            _gcode: (bool): Closed-loop switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, angle=angle, speed=speed)
        with self.lock:
            self._clear_serial_buffer()
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
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
            _gcode: (bool): Closed-loop switch
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles, speed=speed)
        with self.lock:
            self._clear_serial_buffer()
            command = ProtocolCode.SET_ANGLES_P1
            if len(angles) > 0 and angles[0] is not None:
                command += f" A{angles[0]}"
            if len(angles) > 1 and angles[1] is not None:
                command += f" B{angles[1]}"
            if len(angles) > 2 and angles[2] is not None:
                command += f" C{angles[2]}"
            if len(angles) > 3 and angles[3] is not None:
                command += f" D{angles[3]}"
            if speed is not None and 1 <= speed <= 5700:
                command += f" F{speed}"

            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def get_system_version(self):
        """Get system firmware version

        Returns:
            (float) Firmware version
        """
        self._send_command(ProtocolCode.GET_SYSTEM_VERSION_P1)
        return self._request("system_version")

    def get_modify_version(self):
        """Get firmware modify version

        Returns:
            (int) modify version
        """
        self._send_command(ProtocolCode.GET_MODIFY_VERSION_P1)
        return self._request("modify_version")

    def stop(self):
        """Stop movement"""
        with self.lock:
            self._send_command(ProtocolCode.SET_STOP_P1)
            return self._response(_async=False)

    def set_jog_angle(self, joint_id, direction, speed, _async=True, _gcode=False):
        """Start jog movement with angle

        Args:
            joint_id : 1 ~ 4

            direction :
                1 : Negative motion
                0 : Positive motion
            speed : (int) 1-5700
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id, direction=direction,
                                    jog_speed=speed)
        with self.lock:
            self._clear_serial_buffer()
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
                1 : Negative motion
                0 : Positive motion
            speed : (int) 1-5700
        """
        self.calibration_parameters(class_name=self.__class__.__name__, axis_id=axis_id, direction=direction,
                                    jog_speed=speed)
        with self.lock:
            self._clear_serial_buffer()
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
            speed: int (1 - 5700)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, increment_angle=increment, jog_speed=speed)
        with self.lock:
            self._clear_serial_buffer()
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
            speed: int (1 - 5700)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, jog_coord_id=coord_id, increment_coord=increment, speed=speed)
        with self.lock:
            self._clear_serial_buffer()
            command = ProtocolCode.JOG_INCREMENT_COORD_P1
            command += " J" + str(coord_id)
            command += " T" + str(increment)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def get_error_information(self):
        """Read error message"""
        self._send_command(ProtocolCode.GET_ERROR_INFO_P1)
        return self._request("error_information")

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
            return self._response(_async=False)

    def get_zero_calibration_state(self):
        """Read zero-point calibration status.

        Returns:
            (list) zero-point calibration status, len 4
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_BACK_ZERO_STATUS_P1)
            return self._request("zero_calibration_state")

    def set_joint1_encoder_calibration(self):
        """Set the 730 encoder calibration for J1.(Internal Interface)"""
        with self.lock:
            self._send_command(ProtocolCode.SET_J1_ENCODER_CALIBRATION_P1)
            return self._response(_async=False)

    def get_run_status(self):
        """Read running status."""
        with self.lock:
            self._send_command(ProtocolCode.GET_RUNNING_STATUS_P1)
            return self._request("run_status")

    def open_laser(self):
        """Open laser"""
        with self.lock:
            self._send_command(ProtocolCode.OPEN_LASER)
            return self._response(_async=False)

    def close_laser(self):
        """Close laser"""
        with self.lock:
            self._send_command(ProtocolCode.CLOSE_LASER)
            return self._response(_async=False)

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
            return self._response(_async=False)

    def get_gripper_angle(self):
        """Read gripper angle.

        Returns: (int) gripper angle.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_GRIPPER_ANGLE_P1)
            return self._request("get_gripper_angle")

    def set_gripper_parameter(self, addr, mode, parameter_value):
        """Set gripper parameter

        Args:
            addr (int) : 1 ~ 69
            mode (int) : 1 - 2
            parameter_value (int) :
                mode is 1: 0 ~ 255
                mode is 2: > 255

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_addr=addr,
                                    gripper_mode=mode, parameter_value=parameter_value)
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_PARAMETER_P1
            command += " J" + str(addr)
            command += " K" + str(mode)
            command += " L" + str(parameter_value)
            self._send_command(command)
            return self._response(_async=False)

    def get_gripper_parameter(self, addr, mode):
        """Get gripper parameter.

        Args:
            addr (int) : 1 ~ 69
            mode (int) : 1 - 2

        Returns: (int) gripper parameter.
            mode is 1: 0 ~ 255
            mode is 2: > 255
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_addr=addr, gripper_mode=mode)
        with self.lock:
            command = ProtocolCode.GET_GRIPPER_PARAMETER_P1
            command += " J" + str(addr)
            command += " K" + str(mode)
            self._send_command(command)
            return self._request("get_gripper_parameter")

    def get_gripper_run_status(self):
        """Get gripper running status.

        Returns: gripper status.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_GRIPPER_RUN_STATUS_P1)
            return self._request("get_gripper_run_status")

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
            return self._response(_async=False)

    def set_gripper_zero(self):
        """Set gripper zero."""
        with self.lock:
            command = ProtocolCode.SET_GRIPPER_ZERO_P1
            self._send_command(command)
            return self._response(_async=False)

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
            return self._response(_async=False)

    def set_basic_io_output(self, pin_no, pin_signal):
        """Set the status of the base output pin.

        Args:
            pin_no (int) : 1 ~ 10
            pin_signal (int) : 0 ~ 1
                0 - Low level
                1 - High level
        """
        self.calibration_parameters(class_name=self.__class__.__name__, basic_pin_no=pin_no, pin_signal=pin_signal)
        with self.lock:
            command = ProtocolCode.SET_BASIC_OUTPUT_P1
            command += " P" + str(pin_no)
            command += " S" + str(pin_signal)
            self._send_command(command)
            return self._response(_async=False)

    def set_digital_io_output(self, pin_no, pin_signal):
        """Set the state of the end output pin.

        Args:
            pin_no (int) : 1 ~ 4
            pin_signal (int) : 0 ~ 1
                0 - Low level
                1 - High level
        """
        self.calibration_parameters(class_name=self.__class__.__name__, end_pin_no=pin_no, pin_signal=pin_signal)
        with self.lock:
            command = ProtocolCode.SET_DIGITAL_OUTPUT_P1
            command += " P" + str(pin_no)
            command += " S" + str(pin_signal)
            self._send_command(command)
            return self._response(_async=False)

    def set_outer_shaft(self, shaft_state, speed):
        """Set the state of the end output pin.

        Args:
            shaft_state (int) : 0 ~ 1
                0 - close
                1 - open
            speed (int) : 1 ~ 5700
        """
        self.calibration_parameters(class_name=self.__class__.__name__, shaft_state=shaft_state, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_OUTER_SHAFT_P1
            command += " S" + str(shaft_state)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=False)

    def set_pwm(self, p_value):
        """PWM control.

        Args:
            p_value (int) : Duty cycle 0 ~ 5;
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, p_value=p_value)
        with self.lock:
            command = ProtocolCode.SET_PWM_VALUE_P1
            command += " P" + str(p_value)
            self._send_command(command)
            return self._response(_async=False)

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
            command = ProtocolCode.SET_OUTER_SHAFT_P1
            command += " S" + str(data_state)
            command += " L" + str(data_addr)
            command += " N" + str(data_len)
            command += " M" + str(data_value)
            self._send_command(command)
            return self._response(_async=False)

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

                self._serial_port.write(command.encode())
                self._serial_port.flush()
                time.sleep(0.02)
                self._debug_write(command)

    def _normalize_gcode_line(self, line):
        line = line.strip()

        if not line or line.startswith(";"):
            return None

        tokens = line.strip().split()

        if tokens[0].upper() == "G0":
            tokens[0] = "G1"

        return " ".join(tokens)

    def drag_teach_start(self):
        """Drag teach start"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_START_P1
            self._send_command(command)
            return self._response(_async=False)

    def drag_teach_save(self):
        """Drag teach save"""
        with self.lock:
            self._send_command(ProtocolCode.DRAG_TEACH_SAVE_P1)
            return self._response(_async=False)

    def drag_teach_pause(self):
        """Drag teach pause"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_PAUSE_P1
            self._send_command(command)
            return self._response(_async=False)

    def drag_teach_resume(self, number_data):
        """Drag teach resume

        Args:
            number_data (int) : 0 ~ 49
        """
        self.calibration_parameters(class_name=self.__class__.__name__, number_data=number_data)
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_PAUSE_P1
            command += " B" + str(number_data)
            self._send_command(command)
            return self._response(_async=False)

    def drag_teach_stop(self):
        """Drag teach stop"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_STOP_P1
            self._send_command(command)
            return self._response(_async=False)

    def drag_teach_execute(self):
        """Drag teach execute"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_EXECUTE_P1
            self._send_command(command)
            return self._response(_async=False)

    def wifi_open(self):
        """wifi open"""
        with self.lock:
            command = ProtocolCode.WIFI_OPEN_P1
            self._send_command(command)
            return self._response(_async=False)

    def get_system_screen_version(self):
        """Read system screen version.

        Returns: (float) screen version.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_SYSTEM_SCREEN_VERSION_P1)
            return self._request("get_screen_version")

    def get_screen_modify_version(self):
        """Read screen modify version.

        Returns: (float) modify screen version.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_MODIFY_SCREEN_VERSION_P1)
            return self._request("get_screen_modify_version")

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
            return self._response(_async=False)

    def update_stm_firmware(self):
        """update stm firmware"""
        with self.lock:
            command = ProtocolCode.UPDATE_STM32_FIRMWARE
            self._send_command(command)
            return self._response(_async=False)

    def receive_485_data(self):
        """receive 485 data"""
        with self.lock:
            command = ProtocolCode.RECEIVE_485_DATA_P1
            self._send_command(command)
            return self._response(_async=False)

    def go_home(self, speed=2000, _async=True):
        self.set_angles([0, 0, 90, 0], speed, _async=_async)

    def close(self):
        """Close the serial port."""
        with self.lock:
            try:
                self._serial_port.close()
            except Exception:
                pass

    def open(self):
        """Open the serial port."""
        with self.lock:
            try:
                self._serial_port.open()
            except Exception:
                pass

    def set_wifi_password(self, password):
        """Set WiFi password

        Args:
            password (str) : WiFi password
        """
        self.calibration_parameters(class_name=self.__class__.__name__, password=password)
        with self.lock:
            command = ProtocolCode.SET_WIFI_PASSWORD
            command += " P" + str(password)
            self._send_command(command)
            return self._response(_async=False)

    def check_sd_card(self):
        """Check if there is an SD card."""
        with self.lock:
            command = ProtocolCode.CHECK_SD_CARD
            self._send_command(command)
            return self._request("check_sd_card")

    def download_firmware_sd(self, filename, show_progress=True):
        """
        Download firmware to the SD card via M450/M451 commands.

        Args:
            filename (str): name of the firmware file, and must be a .bin file
            show_progress (bool): whether to show download progress
        """
        self.calibration_parameters(class_name=self.__class__.__name__, download_filename=filename)
        if show_progress:
            # callback(percent:int) to report progress
            progress_cb = self._download_progress
        else:
            progress_cb = None
        with self.lock:
            self._clear_serial_buffer()

            # Entering upgrade mode.
            self._fw_enter_upgrade(filename)
            time.sleep(0.2)

            # read bin
            with open(filename, "rb") as f:
                bin_data = f.read()

            chunk_size = 512
            total_packets = (len(bin_data) + chunk_size - 1) // chunk_size

            idx = 1
            while idx <= total_packets:
                offset = (idx - 1) * chunk_size
                data = bin_data[offset: offset + chunk_size]

                pkt = self._fw_build_packet(idx, data)
                self._debug_write(pkt.hex(' ').upper())
                self._serial_port.write(pkt)
                self._serial_port.flush()

                ack = self._fw_read_ack(timeout=1.0)
                if ack is None:
                    continue  # timeout -> resend
                cmd, next_idx = ack

                if cmd == 2:  # success
                    idx = next_idx
                    if progress_cb:
                        progress_cb(int((idx - 1) * 100 / total_packets))

                elif cmd == 3:  # resend
                    idx = next_idx
                else:
                    raise RuntimeError(f"Unknown ACK CMD: {cmd}")

            # Finish
            self._fw_finish_upgrade()