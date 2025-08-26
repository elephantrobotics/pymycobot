#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import threading
import time

from .myagvapi import CommunicationProtocol, Utils, setup_serial_connect, setup_logging


class ProtocolCode:
    HEADER = (0xFE, 0xFE)
    RESTORE = (0x01, 0x00)
    SET_LED = (0x01, 0x02)
    SET_LED_MODE = (0x01, 0x0A)
    SET_MOTION_CONTROL = (0x01, 0x0B)
    GET_FIRMWARE_VERSION = (0x01, 0x03)
    GET_MODIFIED_VERSION = (0x01, 0x09)
    SET_GYRO_STATE = (0x01, 0x07)
    SET_AUTO_REPORT_STATE = (0x01, 0x0c)
    GET_AUTO_REPORT_STATE = (0x01, 0x0d)
    GET_GYRO_STATE = (0x01, 0x08)
    GET_MCU_INFO = ()


class MyAGVCommandProtocolApi(CommunicationProtocol):

    def __init__(self, debug=False):
        self.__movement = False
        self.__mutex = threading.Lock()
        self.log = setup_logging(name=f"{__name__}.{self.__class__.__name__}", debug=debug)

    def _read_command_buffer(self, timeout=1, conditional=None):
        previous_frame = b""
        is_record = False
        commands = b""

        start_time = time.time()
        while time.time() - start_time < timeout:
            current_frame = self.read()
            if current_frame == b'':
                time.sleep(0.001)
                continue

            if current_frame == b"\xfe" and previous_frame == b"\xfe" and is_record is False:
                is_record = True
                commands += b"\xfe\xfe"
                continue

            previous_frame = current_frame
            if is_record is False:
                continue

            commands += current_frame

            cond_res = True if conditional is None else conditional(commands)
            if sum(commands[2:-1]) & 0xff == commands[-1] and cond_res:
                break
        else:
            commands = b""
        return commands

    def _compose_complete_command(self, genre: ProtocolCode, params):  # packing command
        command_args = Utils.process_data_command(params)
        command_args = Utils.flatten(command_args)

        command = [*ProtocolCode.HEADER]
        if isinstance(genre, tuple):
            command.extend(genre)
        else:
            command.append(genre)

        command.extend(command_args)
        command.append(sum(command[2:]) & 0xff)
        return command

    def _parse_reply_instruction(self, genre, reply_data):  # unpacking command
        if len(reply_data) == 0:
            return None

        if genre == ProtocolCode.GET_FIRMWARE_VERSION:
            return Utils.float(reply_data[4], 1)

        elif genre == ProtocolCode.GET_MCU_INFO:
            index = 0
            res = []
            datas = reply_data[2:][:-1]  # header and footer frames are not counted
            while index < len(datas):
                if index in range(0, 3):
                    res.append(datas[index])
                    index += 1

                elif index in range(3, 15, 2):
                    data = Utils.decode_int16(datas[index:index + 2])
                    res.append(Utils.float(data, 2))
                    index += 2

                elif index == 15:
                    binary = bin(datas[index])[2:]
                    binary = binary.zfill(6)
                    res.append(binary)
                    index += 1

                elif index in (16, 17):
                    res.append(datas[index] / 10)
                    index += 1

                elif index in range(18, 26, 2):
                    res.append(Utils.float(Utils.decode_int16(datas[index:index + 2]), 3))
                    index += 2

                elif index in range(26, 32, 2):
                    res.append(Utils.float(Utils.decode_int16(datas[index:index + 2]), 3))
                    index += 2

                elif index in range(32, 42, 1):
                    res.append(datas[index])
                    index += 1
                else:
                    index += 1

            return res
        return reply_data[4]

    def _merge(self, genre, *args, has_reply=False):
        with self.__mutex:
            self.clear()
            if genre not in (ProtocolCode.GET_MCU_INFO,):
                real_command = self._compose_complete_command(genre, args)
                self.log.info(f"write: {' '.join(f'{x:02x}' for x in real_command)}")
                self.write(real_command)
                if has_reply is False:
                    return None

                for _ in range(8):
                    reply_data = self._read_command_buffer(conditional=lambda x: len(x) > 5, timeout=1)
                    if len(reply_data) == 0:
                        time.sleep(0.01)
                        continue

                    if reply_data[2] == genre[0] and reply_data[3] == genre[1]:
                        break

                    time.sleep(0.01)

                else:
                    reply_data = b""
            else:
                reply_data = self._read_command_buffer(conditional=lambda x: len(x) in (45, 29), timeout=1)

            self.log.info(f" read: {' '.join(f'{x:02x}' for x in reply_data)}")
            return self._parse_reply_instruction(genre, reply_data)


class MyAGVCommandApi(MyAGVCommandProtocolApi):
    def set_led(self, mode, r, g, b):
        """Set up LED lights

        Args:
            mode (int):
                1 - Set LED light color.
                2 - Set the LED light to blink
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255
        """
        if mode not in (1, 2):
            raise ValueError("mode must be 1 or 2")

        if r not in range(256):
            raise ValueError("r must be 0 ~ 255")

        if g not in range(256):
            raise ValueError("g must be 0 ~ 255")

        if b not in range(256):
            raise ValueError("b must be 0 ~ 255")

        return self._merge(ProtocolCode.SET_LED, mode, r, g, b)

    def set_led_mode(self, mode: int):
        """Set the LED light mode

        Args:
            mode (int): 0 - charging mode, 1 - DIY mode
        """
        if mode not in [0, 1]:
            raise ValueError("mode must be 0 or 1")
        return self._merge(ProtocolCode.SET_LED_MODE, mode)

    def __basic_move_control(self, *genre, timeout: int = 5):
        t = time.time()
        self.__movement = True
        while time.time() - t < timeout:
            if self.__movement is False:
                break
            self._merge(ProtocolCode.SET_MOTION_CONTROL, *genre)
            time.sleep(0.1)
        self.stop()

    def go_ahead(self, speed: int, timeout: int = 5):
        """
        Control the car to move forward.
        Send control commands every 100ms.
        with a default motion time of 5 seconds.

        Args:
            speed (int): 1 ~ 127 is forward.
            timeout (int): default 5 s.
        """
        if not (0 < speed < 128):
            raise ValueError("speed must be between 1 and 127")
        self.__basic_move_control(128 + speed, 128, 128, timeout=timeout)

    def retreat(self, speed, timeout=5):
        """
        Control the car back. Send control commands every 100ms.
        with a default motion time of 5 seconds

        Args:
            speed (int): 1 ~ 127 is backward
            timeout (int): default 5 s.
        """
        if not (0 < speed < 128):
            raise ValueError("back_speed must be between 1 and 127")
        self.__basic_move_control(128 - speed, 128, 128, timeout=timeout)

    def pan_left(self, speed, timeout=5):
        """
        Control the car to pan to the left.
        Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        if not (0 < speed < 128):
            raise ValueError("pan_left_speed must be between 1 and 127")
        self.__basic_move_control(128, 128 + speed, 128, timeout=timeout)

    def pan_right(self, speed: int, timeout=5):
        """
        Control the car to pan to the right.
        Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        if not (0 < speed < 128):
            raise ValueError("pan_right_speed must be between 1 and 127")
        self.__basic_move_control(128, 128 - speed, 128, timeout=timeout)

    def clockwise_rotation(self, speed: int, timeout=5):
        """
        Control the car to rotate clockwise.
        Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        if speed < 1 or speed > 127:
            raise ValueError("speed must be between 1 and 127")
        self.__basic_move_control(128, 128, 128 - speed, timeout=timeout)

    def counterclockwise_rotation(self, speed: int, timeout=5):
        """
        Control the car to rotate counterclockwise.
        Send control commands every 100ms. with a default motion time of 5 seconds
        Args:
            speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        if speed < 1 or speed > 127:
            raise ValueError("speed must be between 1 and 127")
        self.__basic_move_control(128, 128, 128 + speed, timeout=timeout)

    def stop(self):
        """stop-motion"""
        self._merge(ProtocolCode.SET_MOTION_CONTROL, 128, 128, 128)
        self.__movement = False

    def get_mcu_info(self):
        """
        Get MCU information
        Returns:
            MCU Version(list):
                version 1.0:
                    0-3: vx, vy, vz,
                    3-6: ax, ay, az,
                    6-9: wx, wy, wz,
                    10: battery status[str](
                        0-1: battery 2/1 interface access
                        2: The power adapter is plugged in
                        3: Plug in the charging pile
                        4-5: battery 1/2 LED status
                        ),
                    11-13: batter1/2 voltage,
                    13-15: battery1/2 current
                version 1.1:
                    15-18: Angle range (rollover, pitch, yaw)
                    18: battery status 1 - normal, 0 - abnormal
                    19: MPU status 1 - normal, 0 - abnormal
                    20-24: stall state 1 - stalled, 0 - normal
                    24-28: encoder status 1 - normal, 0 - abnormal
        """
        return self._merge(ProtocolCode.GET_MCU_INFO, has_reply=True)

    def restore(self):
        """Motor stall recovery"""
        self._merge(ProtocolCode.RESTORE, 0x01)

    def set_gyro_state(self, state=1):
        """Set gyroscope calibration status (save after power failure)

        Args:
            state (int, optional): 1 - open. 0 - close. Defaults to 1.
        """
        if state not in (0, 1):
            raise ValueError("state must be 0 or 1")
        self._merge(ProtocolCode.SET_GYRO_STATE, state)

    def get_gyro_state(self):
        """Get gyroscope calibration status

        Return:
            1 - open
            0 - close
        """
        return self._merge(ProtocolCode.GET_GYRO_STATE, has_reply=True)

    def get_firmware_version(self):
        """Get firmware version number"""
        return self._merge(ProtocolCode.GET_FIRMWARE_VERSION, has_reply=True)

    def get_modified_version(self):
        """Get modified version number"""
        return self._merge(ProtocolCode.GET_MODIFIED_VERSION, has_reply=True)

    def set_auto_report_state(self, state):
        """Set the state of automatic reporting

        Args:
            state (int): 1 - open. 0 - close. Defaults to 0.
        """
        if state not in (0, 1):
            raise ValueError("state must be 0 or 1")

        self._merge(ProtocolCode.SET_AUTO_REPORT_STATE, state)

    def get_auto_report_state(self):
        """Get the state of automatic reporting

        Return:
            1 - open
            0 - close
        """

        return self._merge(ProtocolCode.GET_AUTO_REPORT_STATE, has_reply=True)


class MyAgv(MyAGVCommandApi):

    def __init__(self, comport, baudrate=115200, timeout=0.1, debug=False):
        super().__init__(debug=debug)
        self._serial_port = setup_serial_connect(comport, baudrate, timeout)

    def write(self, command):
        self._serial_port.write(bytearray(command))

    def read(self, size=1):
        return self._serial_port.read(size)

    def close(self):
        if self._serial_port.is_open:
            self._serial_port.close()

    def open(self):
        if not self._serial_port.is_open:
            self._serial_port.open()

    def is_open(self):
        return self._serial_port.is_open

    def clear(self):
        self._serial_port.reset_output_buffer()
        self._serial_port.reset_input_buffer()
