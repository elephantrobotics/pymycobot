#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import asyncio
from .myagvpro import MyAGVProCommandProtocolApi, ProtocolCode, PLAINTEXT_REPLY_PROTOCOL_CODE


class MyAGVProCommandApi(MyAGVProCommandProtocolApi):

    async def _match_protocol_data(self, genre, timeout=0.1):
        for _ in range(5):
            reply_data = await self.read()

            if len(reply_data) == 0:
                continue

            self._save_buffer_data(data=reply_data, to_local=True)

            if genre in PLAINTEXT_REPLY_PROTOCOL_CODE:
                if not reply_data.startswith(b"AGVPro:"):
                    continue
            else:
                if reply_data[3] != genre.value:
                    continue

            self.log.info(f" read: {' '.join(f'{x:02x}' for x in reply_data)}")
            break
        else:
            reply_data = None
        return reply_data

    async def _merge(self, genre, *args):
        with self._mutex:
            timeout = self.genre_timeout_table.get(genre, 0.1)
            if not ProtocolCode.GET_AUTO_REPORT_MESSAGE.equal(genre):
                real_command = self._combination(genre, args)
                self.log.info(f"write: {' '.join(f'{x:02x}' for x in real_command)}")
                await self.write(real_command)

            reply_data = await self._match_protocol_data(genre, timeout)
            decode_respond = self._instruction_decoding(genre, reply_data)
            return self._parsing_data(genre, decode_respond)

    async def get_system_version(self):
        """Obtain the major firmware version number

        Returns:
            float: version
        """
        return await self._merge(ProtocolCode.GET_SYSTEM_VERSION)

    async def get_modify_version(self):
        """Obtain the minor firmware version number

        Returns:
            int: version
        """
        return await self._merge(ProtocolCode.GET_MODIFY_VERSION)

    async def get_robot_status(self):
        """Obtain the machine status, and support the acquisition in the case of power failure

        Returns:
            tuple: (list[int] | int, float)
                0 - list[int] | int: machine status，Normally 0
                    0 - (int)Emergency stop status
                    1 - (int)Power status
                    2 - (int)Front bumper strip
                    3 - (int)Rear bumper strip
                    4 - (int)Motor No. 1 connection status
                    5 - (int)Motor No. 2 connection status
                    6 - (int)Motor No. 3 connection status
                    7 - (int)Motor No. 4 connection status
        """
        return await self._merge(ProtocolCode.GET_ROBOT_STATUS)

    async def power_on(self):
        """Turn on the robot

        Returns:
            int: Power-on result
                1 - Success
                2 - Emergency stop triggered
                3 - The battery is too low
                4 - CAN initialization is abnormal
                5 - Motor initialization exception
        """
        return await self._merge(ProtocolCode.POWER_ON)

    async def power_on_only(self):
        """Turn on the robot, but not start the control program

        Returns:
            int: Power-on result only, 1: Success, 0: Failed
        """
        return await self._merge(ProtocolCode.POWER_ON_ONLY)

    async def power_off(self):
        """Turn off the robot

        Returns:
            int: Power-off result only, 1: Success, 0: Failed
        """
        return await self._merge(ProtocolCode.POWER_OFF)

    async def is_power_on(self):
        """Check if the robot is powered on

        Returns:
            int: power state, 1: Power-on, 0: Power-off
        """
        return await self._merge(ProtocolCode.GET_POWER_STATE)

    async def move_forward(self, speed):
        """Pan the robot forward

        Args:
            speed(float): 0.01 ~ 1.50m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0.01 <= speed <= 1.50:
            raise ValueError("Speed must be between 0.01 and 1.50")

        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [int(speed * 100 * 1), 0x00])

    async def move_backward(self, speed):
        """Pan the robot backward

        Args:
            speed(float): 0.01 ~ 1.50m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0.01 <= speed <= 1.50:
            raise ValueError("Speed must be between 0.01 and 1.50")

        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [int(speed * 100 * -1)])

    async def move_left_lateral(self, speed):
        """Pan the robot left

        Args:
            speed(float): 0.01 ~ 1.00 m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0.01 <= speed <= 1.00:
            raise ValueError("Speed must be between 0.01 and 1.00")

        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, int(speed * 100 * -1)])

    async def move_right_lateral(self, speed):
        """Pan the robot right

        Args:
            speed(float): 0.01 ~ 1.00m/s

        Returns:
            int: 1: Success, 0: Failed
        """
        if not 0.01 <= speed <= 1.00:
            raise ValueError("Speed must be between 0.00 and 1.00")

        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, int(speed * 100 * 1)])

    async def turn_left(self, speed):
        """Rotate to the left

        Args:
            speed:

        Returns:
            int: 1: Success, 0: Failed
        """
        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, 0x00, int(speed * 100 * -1), 0x00])

    async def turn_right(self, speed):
        """Rotate to the right

        Args:
            speed:

        Returns:
            int: 1: Success, 0: Failed
        """
        if self.get_significant_bit(speed) > 2:
            raise ValueError(f"speed must be a number with 2 significant bits, but got {speed}")
        return await self._merge(ProtocolCode.AGV_MOTION_CONTROL, [0x00, 0x00, int(speed * 100 * 1), 0x00])

    async def stop(self):
        """Stop moving

        Returns:
            int: 1: Success, 0: Failed
        """
        return await self._merge(ProtocolCode.AGV_STOP_MOVING)

    async def set_auto_report_state(self, state):
        """Set the auto-report state

        Args:
            state(int): 0: Close, 1: Open

        Returns:
            int: 1: Success, 0: Failed
        """

        if state not in (0, 1):
            raise ValueError("State must be 0 or 1")

        return await self._merge(ProtocolCode.SET_AUTO_REPORT_STATE, state)

    async def get_auto_report_state(self):
        """Get the auto-report state

        Returns:
            int: 0: Close, 1: Open
        """
        return await self._merge(ProtocolCode.GET_AUTO_REPORT_STATE)

    async def get_auto_report_message(self):
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
        return await self._merge(ProtocolCode.GET_AUTO_REPORT_MESSAGE)

    async def set_motor_enable(self, motor_id, state):
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
            raise ValueError("Motor id must be 0 or 1")

        if state not in (0, 1):
            raise ValueError("State must be 0 or 1")

        return await self._merge(ProtocolCode.SET_MOTOR_ENABLED, motor_id, state)

    async def get_motor_status(self):
        """Get the motor status

        Returns:
            list[int]: Motor status
                0: normal
                any: error code
        """
        return await self._merge(ProtocolCode.GET_MOTOR_STATUS)

    async def get_motor_temps(self):
        """Get the motor temperature

        Returns:
            list[float]: Motor temperature
        """
        return await self._merge(ProtocolCode.GET_MOTOR_TEMPERATURE)

    async def get_motor_speeds(self):
        """Get the motor speeds

        Returns:
            list[float]: Motor speeds
        """
        return await self._merge(ProtocolCode.GET_MOTOR_SPEEDS)

    async def get_motor_torques(self):
        """Get the motor torques

        Returns:
            list[float]: Motor torques
        """
        return await self._merge(ProtocolCode.GET_MOTOR_TORQUES)

    async def get_motor_enable_status(self):
        """Get the motor enabled status
        Returns:
            list[int]: Motor enabled status
                0: Disable
                1: Enable
        """
        return await self._merge(ProtocolCode.GET_MOTOR_ENABLE_STATUS)

    async def get_motor_loss_count(self):
        """Get the motor loss count

        Returns:
            list[int]: Motor loss count
        """
        return await self._merge(ProtocolCode.GET_MOTOR_LOSS_COUNT)

    async def set_communication_state(self, state):
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
        return await self._merge(ProtocolCode.SET_COMMUNICATION_MODE, state)

    async def get_communication_state(self):
        """Get the communication state

        Returns:
            int: communication state
                0: Serial communication,
                1: Socket communication,
                2: Bluetooth communication
        """
        return await self._merge(ProtocolCode.GET_COMMUNICATION_MODE)

    async def set_led_color(self, position, color, brightness=255):
        """Set the LED color

        Args:
            position(int):
                0: Left LED
                1: Right LED
            color(tuple(int, int, int)): RGB color
            brightness(int): 0 - 255 (default: 255)

        Returns:
            int: 1: Success, 0: Failed
        """
        if position not in (0, 1):
            raise ValueError("Position must be 0 or 1")

        if not 0 <= brightness <= 255:
            raise ValueError("Brightness must be between 0 and 255")

        if len(color) != 3:
            raise ValueError("Color must be a tuple of 3 values")

        if any(map(lambda c: not 0 <= c <= 255, color)):
            raise ValueError("Color must be between 0 and 255")

        return await self._merge(ProtocolCode.SET_LED_COLOR, position, brightness, *color)

    async def set_led_mode(self, mode):
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
        return await self._merge(ProtocolCode.SET_LED_MODE, mode)

    async def set_pin_output(self, pin, state):
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

        return await self._merge(ProtocolCode.SET_OUTPUT_IO, pin, state)

    async def get_pin_input(self, pin):
        """Get the input IO

        Args:
            pin(int): 1 - 6

        Returns:
            int: 0: Low, 1: High, -1: There is no such pin
        """
        if not 1 <= pin <= 6:
            raise ValueError("Pin must be between 1 and 6")
        return await self._merge(ProtocolCode.GET_INPUT_IO, pin)

    def get_estop_state(self):
        """Get the emergency stop state

        Returns:
            int: 0: Release, 1: Press
        """
        return self._merge(ProtocolCode.GET_INPUT_IO, 254)

    async def get_wifi_account(self):
        """Get the wi-fi account

        Returns:
            tuple(str, str): wi-fi account, wi-fi password
        """
        return await self._merge(ProtocolCode.GET_WIFI_ACCOUNT)

    async def get_wifi_ip_port(self):
        """Get the wi-fi ip and port
        Returns:
            tuple(str, int): wi-fi ip, wi-fi port
        """
        return await self._merge(ProtocolCode.GET_WIFI_IP_PORT)

    async def get_bluetooth_uuid(self):
        """Get the bluetooth uuid

        Returns:
            tuple(str, str, str): bluetooth name, service uuid, characteristic uuid
        """

        return await self._merge(ProtocolCode.GET_BLUETOOTH_UUID)

    async def get_bluetooth_address(self):
        """Get the bluetooth MAC address

        Returns:
            str: bluetooth MAC address
        """

        return await self._merge(ProtocolCode.GET_BLUETOOTH_ADDRESS)

    async def set_handle_control_state(self, state):
        """Set the handle control switch status
        Args:
            state(int): 0: Disable, 1: Enable

        Returns:
            int: 1: Success, 0: Failed
        """
        if state not in (0, 1):
            raise ValueError("state must be 0 or 1")
        return await self._merge(ProtocolCode.SET_HANDLE_CONTROL_STATE, state)

    async def get_handle_control_state(self):
        """Get the handle control switch status

        Returns:
            int: 0: Disable, 1: Enable
        """
        return await self._merge(ProtocolCode.GET_HANDLE_CONTROL_STATE)


class MyAGVProBluetooth(MyAGVProCommandApi):
    def __init__(self, address, service_uuid, char_uuid, debug=False, save_serial_log=False):
        super().__init__(debug=debug, save_serial_log=save_serial_log)
        from bleak import BleakClient
        self._bluetooth = BleakClient(address, timeout=10)
        self._address = address
        self._char_uuid = char_uuid
        self._service_uuid = service_uuid
        self._serial_filename = 'agvpro_bluetooth_serial.log'
        self._communication_mode = 2
        self._queue = asyncio.Queue()

    # Async Context managers
    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *args, **kwargs):
        await self.disconnect()

    async def read(self, size=1):
        if not self._queue.empty():
            return self._queue.get_nowait()
        return await self._bluetooth.read_gatt_char(self._char_uuid, size=size)

    async def write(self, data):
        return await self._bluetooth.write_gatt_char(self._char_uuid, data)

    async def connect(self):
        await self._bluetooth.connect()
        await self._bluetooth.start_notify(self._char_uuid, self._handle_notification)

    async def _handle_notification(self, _, data):
        await self._queue.put(data)

    async def disconnect(self):
        await self._bluetooth.stop_notify(self._char_uuid)
        await self._bluetooth.disconnect()

    async def is_connected(self):
        return self._bluetooth.is_connected

    @classmethod
    async def connect_by_name(cls, name, service_uuid, char_uuid, debug=False, save_serial_log=False):
        from bleak import BleakScanner
        device = await BleakScanner.find_device_by_name(name=name)
        if device is None:
            return None

        return cls(
            address=device,
            service_uuid=service_uuid,
            char_uuid=char_uuid,
            debug=debug,
            save_serial_log=save_serial_log
        )
