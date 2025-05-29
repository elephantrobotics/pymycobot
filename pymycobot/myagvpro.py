#!/usr/bin/env python
# -*- coding: UTF-8 -*-
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
    IS_POWER_ON = 0x12

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

    def equal(self, other):
        return self.value == other.value


READ_INTERFACE_INSTRUCTION_SET = (
        ProtocolCode.GET_AUTO_REPORT_MESSAGE,
        ProtocolCode.GET_SYSTEM_VERSION,
        ProtocolCode.IS_POWER_ON,
        ProtocolCode.GET_MOTOR_STATUS,
        ProtocolCode.GET_MOTOR_ENABLE_STATUS,
        ProtocolCode.GET_MOTOR_SPEEDS,
        ProtocolCode.GET_MOTOR_TORQUES,
        ProtocolCode.GET_MOTOR_TEMPERATURE,
        ProtocolCode.GET_COMMUNICATION_MODE,
        ProtocolCode.GET_AUTO_REPORT_STATE,
        ProtocolCode.GET_MODIFY_VERSION
)


class MyAGVProCommandProtocolApi(CommunicationProtocol):

    def __init__(self, debug=False, save_serial_log=False):
        self.__mutex = threading.Lock()
        self.__save_serial_log = save_serial_log
        self.__serial_buffer = []
        self.log = setup_logging(name=f"{__name__}.{self.__class__.__name__}", debug=debug)

    def _merge(self, genre, *args):
        with self.__mutex:
            self.clear()
            if not ProtocolCode.GET_AUTO_REPORT_MESSAGE.equal(genre):
                real_command = self._combination(genre, args)
                self.log.info(f"write: {' '.join(f'{x:02x}' for x in real_command)}")
                self.write(real_command)

            for _ in range(10):
                reply_data = self._read_command_buffer(
                    conditional=lambda x: len(x) == 14 and x[3] == genre.value, timeout=0.1
                )

                if len(reply_data) == 0:
                    continue

                self.log.info(f" read: {' '.join(f'{x:02x}' for x in reply_data)}")
                if reply_data[3] == genre.value:
                    break
            else:
                reply_data = None

            if self.__save_serial_log is True:
                with open("serial.log", "a+") as f:
                    f.write(" ".join(map(str, self.__serial_buffer)))
                    f.write("\n")
                self.__serial_buffer.clear()

            decode_respond = self._instruction_decoding(genre, reply_data)
            return self._parsing_data(genre, decode_respond)

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

        index = 4 if genre in READ_INTERFACE_INSTRUCTION_SET else 5
        respond_body = reply_data[index: -2]

        if genre in (
                ProtocolCode.GET_MOTOR_TEMPERATURE,
                ProtocolCode.GET_MOTOR_SPEEDS,
                ProtocolCode.GET_MOTOR_TORQUES,
                ProtocolCode.GET_MOTOR_STATUS,
        ):
            respond = []
            for i in range(0, len(respond_body), 2):
                data = Utils.decode_int16(respond_body[i:i + 2])
                respond.append(data)
            return respond
        return respond_body

    @classmethod
    def _parsing_data(cls, genre, reply_data: bytearray):
        if not reply_data:
            return None

        if ProtocolCode.GET_SYSTEM_VERSION.equal(genre):
            return reply_data[0] / 10

        if genre in (ProtocolCode.GET_MOTOR_TEMPERATURE, ProtocolCode.GET_MOTOR_SPEEDS, ProtocolCode.GET_MOTOR_TORQUES):
            return list(data / 10 for data in reply_data)

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

        if genre in (ProtocolCode.GET_MOTOR_ENABLE_STATUS, ):
            return list(cls._filter(reply_data, 0x00))

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

    def _read_command_buffer(self, timeout=1.0, conditional=None):
        previous_frame = b""
        is_record = False
        commands = b""

        start_time = time.time()
        while time.time() - start_time < timeout:
            current_frame = self.read()

            if self.__save_serial_log is True:
                print(current_frame)
                self.__serial_buffer.append(current_frame)

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
            if Utils.crc16_check(commands[:-2]) == list(commands[-2:]) and cond_res:
                break
        else:
            commands = b""
        return commands


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
        return self._merge(ProtocolCode.IS_POWER_ON)

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

    def set_communication_state(self, state):
        """Set the communication state

        Args:
            state(int):
                0: Serial communication
                1: Socket communication
                2: Bluetooth communication

        Returns:
            int: 1: Success, 0: Failed
        """
        if state not in (0, 1, 2):
            raise ValueError("State must be 0, 1 or 2")

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


class MyAGVPro(MyAGVProCommandApi):

    def __init__(self, port, baudrate=1000000, timeout=0.1, debug=False, save_serial_log=False):
        super().__init__(debug=debug, save_serial_log=save_serial_log)
        self._serial = setup_serial_connect(port=port, baudrate=baudrate, timeout=timeout)

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
