#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ultraArmP1.py

Python interface for the ultraArmP1 robotic arm.

Author: weijian.wang
Date: 2025-09-11
Description: None
"""

import threading
import time

from pymycobot.common import ProtocolCode


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
        self.lock = threading.Lock()
        time.sleep(1)

    def _respone(self, timeout=90, _async=True):
        """Wait for device response from the serial buffer.

        This method continuously reads data from the serial port until either:
          - A specific keyword ("timx") is detected in the incoming data, or
          - The timeout is reached.

        Args:
            timeout (float): Maximum time (in seconds) to wait for a valid response.
            _async (bool): Whether to wait for a response.

        Returns:
            str: 'ok' if the keyword response ("timx") is detected.
            bool: False if the timeout occurs without receiving a valid response.
        """

        import time
        if not _async:
            return 1
        start_time = time.time()
        received_data = b""
        while time.time() - start_time < timeout:
            received_data += self._serial_port.read(self._serial_port.inWaiting())
            # if b"end" in received_data.lower():
            #     return 'ok'
            end_count = received_data.lower().count(b"end")
            if end_count >= 2:
                return 'ok'
            time.sleep(0.02)
        # Timeout
        if self.debug:
            print("data (timeout):", received_data)

        return False

    def _request(self, flag=""):
        """Send a request and read data from the robot.

        Args:
            flag (str, optional): Type of data expected ('angle', 'coord', or None).

        Returns:
            list[float] or int: Parsed angles/coordinates, or -1 if failed.
        """
        raw_data = None
        attempt = 0

        while attempt < 5:
            if self._serial_port.inWaiting() > 0:
                raw_data = self._serial_port.read(self._serial_port.inWaiting()).decode()
                if self.debug:
                    print("\nReceived data:\n%s**********************\n" % raw_data)

                if "ERROR: COMMAND NOT RECOGNIZED" in raw_data:
                    flag = None

                if flag == "angle":
                    data_lower = raw_data.lower()
                    if "angles" in data_lower:
                        angle_str = data_lower[data_lower.find("angles"):]
                        start_idx = angle_str.find("[")
                        end_idx = angle_str.find("]")
                        try:
                            angles_list = list(map(float, angle_str[start_idx + 1:end_idx].split(",")))
                            angles_list = [round(a, 2) for a in angles_list]
                            return angles_list
                        except Exception:
                            print("Received angles is not completed! Retry receive...")
                            attempt += 1
                            continue
                    else:
                        return -1

                elif flag == "coord":
                    data_lower = raw_data.lower()
                    if "coords" in data_lower:
                        coord_str = data_lower[data_lower.find("coords"):]
                        start_idx = coord_str.find("[")
                        end_idx = coord_str.find("]")
                        try:
                            coords_list = list(map(float, coord_str[start_idx + 1:end_idx].split(",")))
                            coords_list = [round(c, 2) for c in coords_list]
                            return coords_list
                        except Exception:
                            print("Received coords is not completed! Retry receive...")
                            attempt += 1
                            continue
                    else:
                        return -1

                elif flag is None:
                    return -1

            attempt += 1
            time.sleep(0.01)

        return -1

    def _debug(self, data):
        """Print debug information if debug mode is enabled.

        Args:
            data (str): Command or data to print.
        """
        if self.debug:
            print("\n***** Debug Info *****\nsend command: %s" % data)

    def set_unlock(self):
        """Unlock the robot's safety state."""
        with self.lock:
            command = ProtocolCode.SET_UNLOCK + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone()

    def set_reboot(self):
        """Reboot the robot controller board."""
        with self.lock:
            command = ProtocolCode.SET_REBOOT + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone()

    def set_joint_disable(self):
        """Disable the robot joints."""
        with self.lock:
            command = ProtocolCode.SET_JOINT_DISABLE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone()

    def set_joint_enable(self):
        """Enable the robot joints."""
        with self.lock:
            command = ProtocolCode.SET_JOINT_ENABLE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone()

    def get_angles_info(self):
        """Get the current joint angles of the robot.

        Returns:
            list[float] or int: Joint angles [J1, J2, J3, J4] or -1 if failed.
        """
        with self.lock:
            command = ProtocolCode.GET_CURRENT_ANGLES_COORDS_INFO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("angle")

    def get_coords_info(self):
        """Get the current Cartesian coordinates of the robot.

        Returns:
            list[float] or int: Coordinates [X, Y, Z, E] or -1 if failed.
        """
        with self.lock:
            command = ProtocolCode.GET_CURRENT_ANGLES_COORDS_INFO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("coord")

    def set_coords(self, coords, speed, _async=True):
        """Move the robot using Cartesian coordinate control.

        Args:
            coords (list[float]): Coordinates [X, Y, Z].
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
        """
        with self.lock:
            command = ProtocolCode.SET_ANGLES_COORDS
            if len(coords) > 0 and coords[0] is not None:
                command += f" X{coords[0]}"
            if len(coords) > 1 and coords[1] is not None:
                command += f" Y{coords[1]}"
            if len(coords) > 2 and coords[2] is not None:
                command += f" Z{coords[2]}"
            if speed is not None and 1 <= speed <= 5700:
                command += f" F{speed}"

            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone(_async=_async)

    def set_angle(self, joint_id, angle, speed, _async=True):
        """Set a single joint angle.

        Args:
            joint_id (int): Joint number (1~4).
            angle (float): Angle value.
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
        """
        with self.lock:
            command = ProtocolCode.SET_ANGLES_COORDS
            joint_map = {1: "A", 2: "B", 3: "C", 4: "D"}
            if joint_id in joint_map:
                command += f" {joint_map[joint_id]}{angle}"
            if speed > 0:
                command += f" F{speed}"
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone(_async=_async)

    def set_angles(self, angles, speed, _async=True):
        """Move robot using joint angle control.

        Args:
            angles (list[float]): Joint angles [J1, J2, J3, J4].
            speed (int): Movement speed (1~5700).
            _async: (bool): Closed-loop switch
        """
        with self.lock:
            command = ProtocolCode.SET_ANGLES_COORDS
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

            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._respone(_async=_async)

    def close(self):
        """Close the serial port."""
        with self.lock:
            self._serial_port.close()

    def open(self):
        """Open the serial port."""
        with self.lock:
            self._serial_port.open()
