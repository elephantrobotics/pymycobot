#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import re
import threading
import time
import enum

from .myagvapi import CommunicationProtocol, Utils, setup_serial_connect, setup_logging


class ProtocolCode(enum.Enum):
    # System & Product Information
    GET_MODIFY_VERSION = 0x01
    GET_SYSTEM_VERSION = 0x02
    POWER_ON = 0x10
    POWER_ON_ONLY = 0x19
    POWER_OFF = 0x11
    GET_POWER_STATE = 0x12

    # Motion control
    AGV_MOTION_CONTROL = 0x21
    AGV_STOP_MOVING = 0x22
    SET_AUTO_REPORT_STATE = 0x23
    GET_AUTO_REPORT_STATE = 0x24
    GET_AUTO_REPORT_MESSAGE = 0x25

    # aided
    SET_MOTOR_ENABLED = 0x30
    GET_MOTOR_STATUS = 0x31
    SET_COMMUNICATION_MODE = 0x32
    GET_COMMUNICATION_MODE = 0x33
    SET_LED_COLOR = 0x34

    GET_MOTOR_TEMPERATURE = 0x35
    GET_MOTOR_SPEEDS = 0x36
    GET_MOTOR_TORQUES = 0x37
    GET_MOTOR_ENABLE_STATUS = 0x38
    GET_MOTOR_LOSS_COUNT = 0x39

    SET_OUTPUT_IO = 0x40
    GET_INPUT_IO = 0x41

    SET_LED_MODE = 0x3A

    GET_WIFI_ACCOUNT = 0x50
    GET_WIFI_IP_PORT = 0x51
    GET_BLUETOOTH_UUID = 0x52
    GET_BLUETOOTH_ADDRESS = 0x53

    def equal(self, other):
        return self.value == other.value


PLAINTEXT_REPLY_PROTOCOL_CODE = (
    ProtocolCode.GET_WIFI_ACCOUNT,
    ProtocolCode.GET_WIFI_IP_PORT,
    ProtocolCode.GET_BLUETOOTH_UUID,
    ProtocolCode.GET_BLUETOOTH_ADDRESS
)


class MyAGVProCommandProtocolApi(CommunicationProtocol):
    genre_timeout_table = {
        ProtocolCode.POWER_ON: 2.1
    }

    def __init__(self, debug=False, save_serial_log=False):
        self._mutex = threading.Lock()
        self._save_serial_log = save_serial_log
        self._data_buffer = []
        self._serial_filename = "serial.log"
        self._communication_mode = 0
        self.log = setup_logging(name=f"{__name__}.{self.__class__.__name__}", debug=debug)

    def _save_buffer_data(self, data, to_local=False):
        if self._save_serial_log is False:
            return

        self._data_buffer.append(data)

        if to_local is True:
            self._save_file(self._serial_filename, " ".join(map(str, self._data_buffer)) + "\n")
            self._data_buffer.clear()

    @staticmethod
    def _save_file(filename, data):
        with open(filename, "a+") as f:
            f.write(data)

    def _match_protocol_data(self, genre, timeout=0.1):
        is_save_mac_addr = False
        for _ in range(10):
            if ProtocolCode.SET_COMMUNICATION_MODE.equal(genre) and self._communication_mode == 2:
                data = self._read_plaintext_data(timeout=timeout)
                if len(data) == 0:
                    continue

                if is_save_mac_addr is False:
                    mac_addr = self._parsing_data(ProtocolCode.GET_BLUETOOTH_ADDRESS, data.decode('utf-8'))
                    info = f"MyAGVPro Bluetooth MAC Address: {mac_addr}"
                    self._save_file("AGVPro_BLUETOOTH_MAC_ADDR", info)
                    is_save_mac_addr = True
                    print(info)

                reply_data = self._read_protocol_data(timeout=timeout)
                if len(reply_data) == 0:
                    continue

                break

            if genre in PLAINTEXT_REPLY_PROTOCOL_CODE:
                reply_data = self._read_plaintext_data(timeout=timeout)
            else:
                reply_data = self._read_protocol_data(timeout=timeout)

            if len(reply_data) == 0:
                continue

            self.log.info(f" read: {' '.join(f'{x:02x}' for x in reply_data)}")
            break
        else:
            reply_data = None
        return reply_data

    def _read_plaintext_data(self, timeout=0.1):
        reply_data = self._read_command_buffer(
            start_flag_caller=lambda: b'AGVPro:',
            end_flag_caller=lambda cmds: b'\r\n',
            timeout=timeout
        )
        return reply_data

    def _read_protocol_data(self, timeout=0.1):
        reply_data = self._read_command_buffer(
            start_flag_caller=lambda: b'\xfe\xfe',
            end_flag_caller=lambda cmds: Utils.crc16_check_bytes(cmds[:-2]),
            timeout=timeout
        )
        return reply_data

    def _merge(self, genre, *args):
        with self._mutex:
            self.clear()
            timeout = self.genre_timeout_table.get(genre, 0.1)
            if not ProtocolCode.GET_AUTO_REPORT_MESSAGE.equal(genre):
                real_command = self._combination(genre, args)
                self.log.info(f"write: {' '.join(f'{x:02x}' for x in real_command)}")
                self.write(real_command)

            reply_data = self._match_protocol_data(genre, timeout)
            decode_respond = self._instruction_decoding(genre, reply_data)
            data = self._parsing_data(genre, decode_respond)
            self._save_buffer_data(b'', to_local=True)
            return data

    @classmethod
    def _filter(cls, data, target):
        for index in range(len(data) - 1, -1, -1):
            if data[index] != target:
                return data[:index + 1]
        return data

    @classmethod
    def _instruction_decoding(cls, genre, reply_data):
        if not reply_data:
            return None

        respond_body = reply_data[4: -2]
        if genre in PLAINTEXT_REPLY_PROTOCOL_CODE:
            respond_body = reply_data.decode("utf-8")

        if genre in (
                ProtocolCode.GET_MOTOR_TEMPERATURE,
                ProtocolCode.GET_MOTOR_SPEEDS,
                ProtocolCode.GET_MOTOR_TORQUES,
                ProtocolCode.GET_MOTOR_STATUS,
                ProtocolCode.GET_MOTOR_LOSS_COUNT
        ):
            respond = []
            for i in range(0, len(respond_body), 2):
                data = Utils.decode_int16(respond_body[i:i + 2])
                respond.append(data)
            return respond
        return respond_body

    @classmethod
    def _parsing_data(cls, genre, reply_data):
        if not reply_data:
            return None

        if ProtocolCode.GET_SYSTEM_VERSION.equal(genre):
            return reply_data[0] / 10

        if ProtocolCode.GET_MOTOR_TEMPERATURE.equal(genre):
            return list(data / 10 for data in reply_data)

        if genre in (ProtocolCode.GET_MOTOR_SPEEDS, ProtocolCode.GET_MOTOR_TORQUES):
            return list(data / 100 for data in reply_data)

        if ProtocolCode.GET_MOTOR_LOSS_COUNT.equal(genre):
            return list(reply_data)

        if ProtocolCode.GET_WIFI_ACCOUNT.equal(genre):
            data = re.findall(r'AGVPro:WIFI:S:(.*);P:(.*);', reply_data)
            return data[0] if len(data) == 1 else None

        if ProtocolCode.GET_WIFI_IP_PORT.equal(genre):
            data = re.findall(r'AGVPro:WIFI:IP:(.*);PORT:(.*);', reply_data)
            if len(data) == 0:
                return None
            return data[0][0], int(data[0][1])

        if ProtocolCode.GET_BLUETOOTH_UUID.equal(genre):
            data = re.findall(r'AGVPro:BLE::Name:(.*);Service_UUID:(.*);CHAR_UUID:(.*);', reply_data)
            return data[0] if len(data) == 1 else None

        if ProtocolCode.GET_BLUETOOTH_ADDRESS.equal(genre):
            data = re.findall(r'AGVPro:BLE:MAC:(.*)', reply_data)
            return data[0] if len(data) == 1 else None

        if ProtocolCode.GET_MOTOR_STATUS.equal(genre):
            respond = []
            for item in reply_data:
                if item == 0:
                    respond.append(0)
                    continue

                rank = Utils.get_bits(item)
                respond.append(rank)
            return respond

        if ProtocolCode.GET_AUTO_REPORT_MESSAGE.equal(genre):
            respond = []
            for index, data in enumerate(reply_data):
                if index < 3:
                    respond.append(round(data / 100, 2))
                elif index in (3, 4):
                    if data == 0:
                        rank = data
                    else:
                        rank = Utils.get_bits(data)
                    respond.append(rank)
                elif index == 5:
                    respond.append(round(data / 10, 1))
                elif index == 6:
                    respond.append(data)
            return respond

        if ProtocolCode.GET_MOTOR_ENABLE_STATUS.equal(genre):
            return list(reply_data[:4])

        if ProtocolCode.GET_INPUT_IO.equal(genre):
            if reply_data[0] == 255:
                return -1
            return reply_data[1]

        return reply_data[0]

    @classmethod
    def _combination(cls, genre, params):
        command_args = Utils.process_data_command(params)
        while len(command_args) < 8:
            command_args.append(0x00)
        command = bytearray([0xfe, 0xfe, 0x0b, genre.value])
        command.extend(command_args)
        crc16_check_code = Utils.crc16_check(command)

        command.extend(crc16_check_code)
        return command

    def _read_by_timeout(self, timeout=1.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            data = self.read(1)
            if len(data) == 0:
                continue

            yield data

            self._save_buffer_data(data)

    def _read_command_buffer(self, start_flag_caller, end_flag_caller, timeout=1.0):
        channel_buffer = b""
        real_command = b""
        is_record = False

        for data in self._read_by_timeout(timeout):
            channel_buffer += data

            start_flag = start_flag_caller()
            if channel_buffer.endswith(start_flag) and is_record is False:
                real_command = start_flag
                is_record = True
                continue

            if is_record is False:
                continue

            real_command += data
            end_flag = end_flag_caller(real_command)
            if real_command.endswith(end_flag):
                break
        else:
            real_command = b""
        return real_command


class MyAGVProCommandApi(MyAGVProCommandProtocolApi):

    def get_system_version(self):
        """Obtain the major firmware version number

        Returns:
            float: version
        """
        return self._merge(ProtocolCode.GET_SYSTEM_VERSION)

    def get_modify_version(self):
        """Obtain the minor firmware version number

        Returns:
            int: version
        """
        return self._merge(ProtocolCode.GET_MODIFY_VERSION)

    def power_on(self):
        """Turn on the robot

        Returns:
            int: Power-on result, 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.POWER_ON)

    def power_on_only(self):
        """Turn on the robot, but not start the control program

        Returns:
            int: Power-on result only, 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.POWER_ON_ONLY)

    def power_off(self):
        """Turn off the robot

        Returns:
            int: Power-off result only, 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.POWER_OFF)

    def is_power_on(self):
        """Check if the robot is powered on

        Returns:
            int: power state, 1: Power-on, 0: Power-off
        """
        return self._merge(ProtocolCode.GET_POWER_STATE)

    def move_forward(self, speed):
        """Pan the robot forward

        Args:
            speed(float): 0 ~ 1.5 m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0 <= speed <= 1.5:
            raise ValueError("Speed must be between 0 and 1.5")
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [int(speed * 100 * 1), 0x00])

    def move_backward(self, speed):
        """Pan the robot backward

        Args:
            speed(float): 0 ~ 1.5 m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0 <= speed <= 1.5:
            raise ValueError("Speed must be between 0 and 1.5")
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [int(speed * 100 * -1)])

    def move_left_lateral(self, speed):
        """Pan the robot left

        Args:
            speed(float): 0 ~ 1 m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0 <= speed <= 1:
            raise ValueError("Speed must be between 0 and 1")
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, int(speed * 100 * 1)])

    def move_right_lateral(self, speed):
        """Pan the robot right

        Args:
            speed(float): 0 ~ 1 m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0 <= speed <= 1:
            raise ValueError("Speed must be between 0 and 1")
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, int(speed * 100 * -1)])

    def turn_left(self, speed):
        """Rotate to the left

        Args:
            speed:

        Returns:
            int: 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, 0x00, int(speed * 100 * -1), 0x00])

    def turn_right(self, speed):
        """Rotate to the right

        Args:
            speed:

        Returns:
            int: 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, 0x00, int(speed * 100 * 1)])

    def stop(self):
        """Stop moving

        Returns:
            int: 1: Success, 0: Failed
        """
        return self._merge(ProtocolCode.AGV_STOP_MOVING)

    def set_auto_report_state(self, state):
        """Set the auto-report state

        Args:
            state(int): 0: Close, 1: Open

        Returns:
            int: 1: Success, 0: Failed
        """

        if state not in (0, 1):
            raise ValueError("State must be 0 or 1")

        return self._merge(ProtocolCode.SET_AUTO_REPORT_STATE, state)

    def get_auto_report_state(self):
        """Get the auto-report state

        Returns:
            int: 0: Close, 1: Open
        """
        return self._merge(ProtocolCode.GET_AUTO_REPORT_STATE)

    def get_auto_report_message(self):
        """Get the auto-report message
        Returns:
            list[int | list[int] | float]:
             0 - (float)rx
             1 - (float)ry
             2 - (float)rw
             3 - (list[int])Machine status
             4 - (list[int])Motor information
             5 - (float)Battery voltage
             6 - (int)Motor enable status 0: Enabled, 1: Disabled
        """
        return self._merge(ProtocolCode.GET_AUTO_REPORT_MESSAGE)

    def set_motor_enable(self, motor_id, state):
        """Enable or disable the motor

        Args:
            motor_id(int):
                1: Left upper motor
                2: Right upper motor
                3: Left lower motor
                4: Right lower motor
                254: All motors
            state(bool):
                0: Disable
                1: Enable
        Returns:
            int: 1: Success, 0: Failed
        """
        if motor_id not in (1, 2, 3, 4, 254):
            raise ValueError("Motor ID must be 0 or 1")

        if state not in (0, 1):
            raise ValueError("State must be 0 or 1")

        return self._merge(ProtocolCode.SET_MOTOR_ENABLED, motor_id, state)

    def get_motor_status(self):
        """Get the motor status

        Returns:
            list[int]: Motor status
                0: normal
                any: error code
        """
        return self._merge(ProtocolCode.GET_MOTOR_STATUS)

    def get_motor_temps(self):
        """Get the motor temperature

        Returns:
            list[float]: Motor temperature
        """
        return self._merge(ProtocolCode.GET_MOTOR_TEMPERATURE)

    def get_motor_speeds(self):
        """Get the motor speeds

        Returns:
            list[float]: Motor speeds
        """
        return self._merge(ProtocolCode.GET_MOTOR_SPEEDS)

    def get_motor_torques(self):
        """Get the motor torques

        Returns:
            list[float]: Motor torques
        """
        return self._merge(ProtocolCode.GET_MOTOR_TORQUES)

    def get_motor_enable_status(self):
        """Get the motor enabled status
        Returns:
            list[int]: Motor enabled status
                0: Disable
                1: Enable
        """
        return self._merge(ProtocolCode.GET_MOTOR_ENABLE_STATUS)

    def get_motor_loss_count(self):
        """Get the motor loss count

        Returns:
            list[int]: Motor loss count
        """
        return self._merge(ProtocolCode.GET_MOTOR_LOSS_COUNT)

    def set_communication_state(self, state):
        """Set the communication state

        Args:
            state(int):
                0: Serial communication (default)
                1: Socket communication
                2: Bluetooth communication (Write the MAC address to the file and the endpoint, and then return to the state)

        Returns:
            int: 1: Success, 0: Failed
        """
        if state not in (0, 1, 2):
            raise ValueError("State must be 0, 1 or 2")

        self._communication_mode = state
        return self._merge(ProtocolCode.SET_COMMUNICATION_MODE, state)

    def get_communication_state(self):
        """Get the communication state

        Returns:
            int: communication state
                0: Serial communication,
                1: Socket communication,
                2: Bluetooth communication
        """
        return self._merge(ProtocolCode.GET_COMMUNICATION_MODE)

    def set_led_color(self, position, brightness, color):
        """Set the LED color

        Args:
            position(int):
                0: Left LED
                1: Right LED
            brightness(int): 0 - 255
            color(tuple(int, int, int)): RGB color

        Returns:
            int: 1: Success, 0: Failed
        """
        if position not in (0, 1):
            raise ValueError("Position must be 0 or 1")

        if not 0 <= brightness <= 255:
            raise ValueError("Brightness must be between 0 and 255")

        if any(not 0 <= c <= 255 for c in color):
            raise ValueError("Color must be between 0 and 255")

        return self._merge(ProtocolCode.SET_LED_COLOR, position, brightness, *color)

    def set_led_mode(self, mode):
        """Set the LED mode

        Args:
            mode(int):
                0: Battery level(default)
                1: DIY

        Returns:
            int: 1: Success, 0: Failed
        """
        if mode not in (0, 1):
            raise ValueError("Mode must be 0 or 1")
        return self._merge(ProtocolCode.SET_LED_MODE, mode)

    def set_pin_output(self, pin, state):
        """Set the output IO

        Args:
            pin(int): 1 - 6
            state(int): 0: Low, 1: High

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 1 <= pin <= 6:
            raise ValueError("Pin must be between 1 and 6")

        if state not in (0, 1):
            raise ValueError("State must be 0 or 1")

        return self._merge(ProtocolCode.SET_OUTPUT_IO, pin, state)

    def get_pin_input(self, pin):
        """Get the input IO

        Args:
            pin(int): 1, 2, 3, 4, 5, 6, 7, 8, 254

        Returns:
            int: 0: Low, 1: High, -1: There is no such pin
        """
        supported_pins = [1, 2, 3, 4, 5, 6, 7, 8, 254]
        if pin not in supported_pins:
            raise ValueError(f"Pin must be in {supported_pins}")
        return self._merge(ProtocolCode.GET_INPUT_IO, pin)

    def get_wifi_account(self):
        """Get the wi-fi account

        Returns:
            tuple(str, str): wi-fi account, wi-fi password
        """
        return self._merge(ProtocolCode.GET_WIFI_ACCOUNT)

    def get_wifi_ip_port(self):
        """Get the wi-fi ip and port
        Returns:
            tuple(str, int): wi-fi ip, wi-fi port
        """
        return self._merge(ProtocolCode.GET_WIFI_IP_PORT)

    def get_bluetooth_uuid(self):
        """Get the bluetooth uuid

        Returns:
            tuple(str, str, str): bluetooth name, service uuid, characteristic uuid
        """

        return self._merge(ProtocolCode.GET_BLUETOOTH_UUID)

    def get_bluetooth_address(self):
        """Get the bluetooth MAC address

        Returns:
            str: bluetooth MAC address
        """

        return self._merge(ProtocolCode.GET_BLUETOOTH_ADDRESS)


class MyAGVPro(MyAGVProCommandApi):

    def __init__(self, port, baudrate=1000000, timeout=0.1, debug=False, save_serial_log=False):
        super().__init__(debug=debug, save_serial_log=save_serial_log)
        self._serial = setup_serial_connect(port=port, baudrate=baudrate, timeout=timeout)
        self._serial_filename = 'agvpro_serial_serial.log'

    def write(self, command):
        return self._serial.write(command)

    def read(self, size=1):
        return self._serial.read(size)

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def open(self):
        if not self._serial.is_open:
            self._serial.open()

    def is_open(self):
        return self._serial.is_open

    def clear(self):
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
