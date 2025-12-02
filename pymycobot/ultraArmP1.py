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

from demo.myArm_demo.myarm_handle_control import command
from pymycobot.common import ProtocolCode
from pymycobot.error import calibration_parameters


class UltraArmP1:
    """Class for controlling the ultraArmP1 robotic arm via serial communication.

    """

    def __init__(self, port, baudrate=115200, timeout=0.1, debug=False):
        """Initialize the ultraArmP1 robot communication.

        Args:
            port (str): Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int, optional): Communication baud rate. Defaults to 115200.
            timeout (float, optional): Serial read timeout in seconds. Defaults to 0.1.
            debug (bool, optional): Whether to print debug information. Defaults to False.
        """
        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = True
        self._serial_port.dtr = True
        self._serial_port.open()
        self.debug = debug
        self.calibration_parameters = calibration_parameters
        self.lock = threading.Lock()
        time.sleep(1)

    # ---------------------- Debug / time helpers ----------------------
    def _now(self):
        """Return timestamp string with millisecond precision."""
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _debug_write(self, data: str):
        if self._debug:
            print(f"{self._now()} DEBU [UltraArmP1] _write: {data}")

    def _debug_read(self, data: str):
        if self._debug:
            print(f"{self._now()} DEBU [UltraArmP1] _read : {data}")

    # Keep original _debug for backward compatibility (calls new writer)
    def _debug(self, data):
        if self.debug:
            self._debug_write(data)

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

        while time.time() - start_time < timeout:
            chunk = self._read_available_bytes()
            # print('chunk:', chunk)
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
            time.sleep(0.01)
        # Timeout
        if self.debug:
            try:
                print(f"{self._now()} DEBU [UltraArmP1] _timeout : {received_data}")
            except Exception:
                print(f"{self._now()} DEBU [UltraArmP1] _timeout : <binary>")

        return False

    def _request(self, flag=""):
        """Send a request and read data from the robot.
        Improved: clear input before send (caller should send cmd before calling _request),
        accumulate chunks, wait until 'angles' or 'coords' fully parsed or timeout.
        """
        # total timeout (seconds) to wait for a complete response
        TOTAL_TIMEOUT = 1.0
        # small sleep between polls
        POLL_SLEEP = 0.01

        start_time = time.time()
        raw_data = ""

        # If available, flush leftover input first to avoid stale data interference
        try:
            # pyserial: reset_input_buffer() is preferred in newer versions
            if hasattr(self._serial_port, "reset_input_buffer"):
                self._serial_port.reset_input_buffer()
            elif hasattr(self._serial_port, "flushInput"):
                self._serial_port.flushInput()
        except Exception:
            # non-fatal
            pass

        # Poll until timeout
        while time.time() - start_time < TOTAL_TIMEOUT:
            try:
                n = self._serial_port.inWaiting()
            except Exception:
                n = 0

            if n > 0:
                try:
                    chunk = self._serial_port.read(n)
                    try:
                        chunk_str = chunk.decode(errors="ignore")
                    except Exception:
                        chunk_str = str(chunk)
                    raw_data += chunk_str

                    # debug print only when we have new data; do not treat first fragment as final
                    if self.debug:
                        # show accumulated buffer (shortened if too long)
                        display = raw_data if len(raw_data) < 1000 else raw_data[-1000:]
                        self._debug_read(display)

                    lower = raw_data.lower()

                    # Check command-not-recognized
                    if "error: command not recognized" in lower:
                        return -1

                    # parse angles when requested
                    if flag == "angle":
                        if "angles" in lower:
                            # find the first '[...'] after 'angles'
                            idx = lower.find("angles")
                            bracket_start = lower.find("[", idx)
                            bracket_end = lower.find("]", idx)
                            if bracket_start != -1 and bracket_end != -1 and bracket_end > bracket_start:
                                try:
                                    sub = lower[bracket_start + 1:bracket_end]
                                    angles_list = [round(float(x), 2) for x in sub.split(",") if x.strip() != ""]
                                    return angles_list
                                except Exception:
                                    # parsing failed -> keep accumulating until timeout
                                    if self.debug:
                                        print(
                                            f"{self._now()} DEBU [UltraArmP1] _warn : angles parse failed, continue receiving...")
                                    # continue waiting
                    # parse coords when requested
                    elif flag == "coord":
                        if "coords" in lower:
                            idx = lower.find("coords")
                            bracket_start = lower.find("[", idx)
                            bracket_end = lower.find("]", idx)
                            if bracket_start != -1 and bracket_end != -1 and bracket_end > bracket_start:
                                try:
                                    sub = lower[bracket_start + 1:bracket_end]
                                    coords_list = [round(float(x), 2) for x in sub.split(",") if x.strip() != ""]
                                    return coords_list
                                except Exception:
                                    if self.debug:
                                        print(
                                            f"{self._now()} DEBU [UltraArmP1] _warn : coords parse failed, continue receiving...")
                                    # continue waiting
                    elif flag is None:
                        return -1

                except Exception as e:
                    if self.debug:
                        print(f"{self._now()} DEBU [UltraArmP1] _error : serial read exception: {e}")
                    # small sleep and continue
            # no data right now
            time.sleep(POLL_SLEEP)

        # timeout
        if self.debug:
            print(f"{self._now()} DEBU [UltraArmP1] _warn : request timeout, received buffer: {raw_data!r}")
        return -1

    def _send_command(self, command: str):
        """Send commands to serial port"""
        command += ProtocolCode.END
        self._debug_write(command)
        self._serial_port.write(command.encode())
        self._serial_port.flush()

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

    def set_jog_angle(self, joint_id, direction, speed, _async=False, _gcode=False):
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
            command = ProtocolCode.SET_JOG_ANGLE_P1
            command += " J" + str(joint_id)
            command += " D" + str(direction)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def set_jog_coord(self, axis_id, direction, speed, _async=False, _gcode=False):
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
            command = ProtocolCode.SET_JOG_COORD_P1
            command += " J" + str(axis_id)
            command += " D" + str(direction)
            command += " F" + str(speed)
            self._send_command(command)
            return self._response(_async=_async, _gcode=_gcode)

    def jog_increment_angle(self, joint_id, increment, speed, _async=False, _gcode=False):
        """Single angle incremental motion control.

        Args:
            joint_id: Joint id 1 - 4
            increment: Angle increment value
            speed: int (1 - 5700)
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

    def jog_increment_coord(self, coord_id, increment, speed, _async=False, _gcode=False):
        """Single coordinate incremental motion control.

        Args:
            coord_id: axis id 1 - 4.
            increment: Coord increment value
            speed: int (1 - 5700)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, jog_coord_id=coord_id, increment_coord=increment, speed=speed)
        with self.lock:
            command = ProtocolCode.JOG_INCREMENT_ANGLE_P1
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
            self._send_command(ProtocolCode.SET_JOINT_ZERO_CALIBRATION_P1)
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
            self._response(_async=False)

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
            self._response(_async=False)

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
            self._response(_async=False)

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
            self._response(_async=False)

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
            self._response(_async=False)

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
            self._response(_async=False)

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
            self._response(_async=False)

    def set_pwm(self, p_value):
        """PWM control.

        Args:
            p_value (int) : Duty cycle 0 ~ 255; 128 means 50%
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, p_value=p_value)
        with self.lock:
            command = ProtocolCode.SET_PWM_VALUE_P1
            command += " P" + str(p_value)
            self._send_command(command)
            self._response(_async=False)

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
            self._response(_async=False)

    def drag_teach_start(self):
        """Drag teach start"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_START_P1
            self._send_command(command)
            self._response(_async=False)

    def drag_teach_reproduction(self, number_data):
        """Drag teach reproduction

        Args:
            number_data (int) : 0 ~ 49
        """
        self.calibration_parameters(class_name=self.__class__.__name__, number_data=number_data)
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_REPRODUCTION_P1
            command += " B" + str(number_data)
            self._send_command(command)
            self._response(_async=False)

    def drag_teach_pause(self):
        """Drag teach pause"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_PAUSE_P1
            self._send_command(command)
            self._response(_async=False)

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
            self._response(_async=False)

    def drag_teach_stop(self):
        """Drag teach stop"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_STOP_P1
            self._send_command(command)
            self._response(_async=False)

    def drag_teach_execute(self):
        """Drag teach execute"""
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_EXECUTE_P1
            self._send_command(command)
            self._response(_async=False)

    def wifi_open(self):
        """wifi open"""
        with self.lock:
            command = ProtocolCode.WIFI_OPEN_P1
            self._send_command(command)
            self._response(_async=False)

    def get_system_screen_version(self):
        """Read system screen version.

        Returns: (float) screen version.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_GRIPPER_ANGLE_P1)
            return self._request("get_screen_version")

    def get_modify_screen_version(self):
        """Read modify screen version.

        Returns: (float) modify screen version.
        """
        with self.lock:
            self._send_command(ProtocolCode.GET_GRIPPER_ANGLE_P1)
            return self._request("get_modify_screen_version")

    def set_communication_baud_rate(self, baud_rate):
        """set communication baud rate

        Args:
            baud_rate (int) : 1 ~ 65535
            """
        self.calibration_parameters(class_name=self.__class__.__name__, baud_rate=baud_rate)
        with self.lock:
            command = ProtocolCode.DRAG_TEACH_EXECUTE_P1
            command += " B" + str(baud_rate)
            self._send_command(command)
            self._response(_async=False)

    def update_stm_firmware(self):
        """update stm firmware"""
        with self.lock:
            command = ProtocolCode.UPDATE_STM32_FIRMWARE
            self._send_command(command)
            self._response(_async=False)

    def receive_485_data(self):
        """receive 485 data"""
        with self.lock:
            command = ProtocolCode.RECEIVE_485_DATA_P1
            self._send_command(command)
            self._response(_async=False)

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


