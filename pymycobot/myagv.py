import enum
import serial
import time
import struct
import logging
from pymycobot.log import setup_logging
from pymycobot.common import DataProcessor
from pymycobot.error import calibration_parameters


class ProtocolCode(enum.Enum):
    HEADER = 0xFE
    RESTORE = [0x01, 0x00]
    SET_LED = [0x01, 0x02]
    SET_LED_MODE = [0x01, 0x0A]
    GET_FIRMWARE_VERSION = [0x01, 0x03]
    GET_MOTORS_CURRENT = [0x01, 0x04]
    GET_BATTERY_INFO = [0x01, 0x05]
    SET_GYRO_STATE = [0x01, 0x07]
    GET_GYRO_STATE = [0x01, 0x08]
    GET_MODIFIED_VERSION = [0x01, 0x09]


class MyAgv(DataProcessor):
    def __init__(self, port="/dev/ttyAMA0", baudrate="115200", timeout=0.1, debug=False):
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.__movement = False

    def _write(self, command):
        self._serial_port.reset_input_buffer()
        self.log.debug("_write: {}".format([hex(i) for i in command]))
        self._serial_port.write(command)
        self._serial_port.flush()

    def _read(self, command) -> bytes:
        datas = b''
        k = 0
        pre = 0
        end = 5
        t = time.time()
        if command[k - 1] == 29:
            end = 28
        elif command[k - 1] == 44:
            end = 43
        while time.time() - t < 0.2:
            data = self._serial_port.read()
            k += 1
            if len(datas) == 4:
                if datas[-2] == 0x01 and datas[-1] == 0x05:
                    end = 7
                datas += data

            elif len(datas) == end:
                datas += data
                break
            elif len(datas) > 4:
                datas += data

            elif len(datas) >= 2:
                data_len = struct.unpack("b", data)[0]
                if command[-1] == 29 or command[-1] == 44 or data_len == command[k - 1]:
                    datas += data
                else:
                    datas = b''
                    k = 0
                    pre = 0
            elif data == b"\xfe":
                if datas == b'':
                    datas += data
                    if k != 1:
                        k = 1
                    pre = k
                else:
                    if k - 1 == pre:
                        datas += data
                    else:
                        datas = b"\xfe"
                        k = 1
                        pre = 0
        else:
            datas = b''
        self.log.debug("_read: {}".format([hex(data) for data in datas]))
        return datas

    def _mesg(self, genre, *args, **kwargs):
        """

        Args:
            genre: command type (Command)
            *args: other data.
                   It is converted to octal by default.
                   If the data needs to be encapsulated into hexadecimal,
                   the array is used to include them. (Data cannot be nested)
            **kwargs: support `has_reply`
                has_reply: Whether there is a return value to accept.
        """
        has_reply = kwargs.get("has_reply", None)
        real_command = self._process_data_command(genre, self.__class__.__name__, args)
        command = [
            ProtocolCode.HEADER.value,
            ProtocolCode.HEADER.value,
        ]
        if isinstance(genre, list):
            for data in genre:
                command.append(data)
        else:
            command.append(genre)
        command.append(real_command)
        command = self._flatten(command)
        if genre == ProtocolCode.SET_LED.value:
            command.append(sum(command[2:]) & 0xff)
        elif genre == ProtocolCode.GET_FIRMWARE_VERSION.value:
            command.append(4)
        elif genre == ProtocolCode.GET_MOTORS_CURRENT.value:
            command.append(5)
        elif genre == ProtocolCode.GET_BATTERY_INFO.value:
            command.append(6)
        else:
            command.append(sum(command[2:]) & 0xff)
        self._write(command)
        if has_reply:
            data = self._read(command)
            if data:
                if genre in [ProtocolCode.GET_FIRMWARE_VERSION.value]:
                    return self._int2coord(data[4])
                elif genre == ProtocolCode.GET_MOTORS_CURRENT.value:
                    return self._decode_int16(data[4:6])
                elif genre == ProtocolCode.GET_BATTERY_INFO.value:
                    byte_1 = bin(data[4])[2:]
                    res = []
                    while len(byte_1) != 6:
                        byte_1 = "0" + byte_1
                    res.append(byte_1)
                    res.append(self._int2coord(data[5]))
                    res.append(self._int2coord(data[6]))
                    if byte_1[0] == "0":
                        res[-1] = 0
                    elif byte_1[1] == "0":
                        res[1] = 0
                    return res
                return data[4]

        return None

    def set_led(self, mode, R, G, B):
        """Set up LED lights

        Args:
            mode (int): 1 - Set LED light color. 2 - Set the LED light to blink
            R (int): 0 ~ 255
            G (int): 0 ~ 255
            B (int): 0 ~ 255
        """
        calibration_parameters(class_name=self.__class__.__name__, rgb=[R, G, B], led_mode=mode)
        return self._mesg(ProtocolCode.SET_LED.value, mode, R, G, B)

    def set_led_mode(self, mode: int):
        """Set the LED light mode

        Args:
            mode (int): 0 - charging mode, 1 - DIY mode
        """
        if mode not in [0, 1]:
            raise ValueError("mode must be 0 or 1")
        return self._mesg(ProtocolCode.SET_LED_MODE.value, mode)

    def get_firmware_version(self):
        """Get firmware version number
        """
        return self._mesg(ProtocolCode.GET_FIRMWARE_VERSION.value, has_reply=True)

    def get_motors_current(self):
        """Get the total current of the motor
        """
        return self._mesg(ProtocolCode.GET_MOTORS_CURRENT.value, has_reply=True)

    def get_battery_info(self):
        """Read battery information
        
        Return:
            list : [battery_data, battery_1_voltage, battery_2_voltage].
                battery_data:
                    A string of length 6, represented from left to right: 
                    bit5, bit4, bit3, bit2, bit1, bit0.

                    bit5 : Battery 2 is inserted into the interface 1 means inserted, 0 is not inserted.
                    bit4 : Battery 1 is inserted into the interface, 1 means inserted, 0 is not inserted.
                    bit3 : The adapter is plugged into the interface 1 means plugged in, 0 not plugged in.
                    bit2 : The charging pile is inserted into the interface, 1 means plugged in, 0 is not plugged in.
                    bit1 : Battery 2 charging light 0 means off, 1 means on.
                    bit0 : Battery 1 charging light, 0 means off, 1 means on.
                battery_1_voltage : Battery 1 voltage in volts.
                battery_2_voltage : Battery 2 voltage in volts.
        """
        return self._mesg(ProtocolCode.GET_BATTERY_INFO.value, has_reply=True)

    def __basic_move_control(self, *genre, timeout: int = 5):
        t = time.time()
        self.__movement = True
        while time.time() - t < timeout:
            if self.__movement is False:
                break
            self._mesg(*genre)
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
        self.__movement = False
        self._mesg(128, 128, 128)

    def get_mcu_info(self, version: float = None) -> list:
        """
        Get MCU information
        Args:
            version (float): firmware version, default None, auto-detect
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
        if version is None:
            version = self.get_firmware_version()

        if version == 1.0:
            data_len = 29
        else:
            data_len = 44

        res = []
        index = 0
        datas = self._read([0xfe, 0xfe, data_len])
        if not datas:
            return res

        datas = datas[2:][:-1]  # header and footer frames are not counted
        while index < len(datas):
            if index in range(0, 3):
                res.append(datas[index])
                index += 1

            elif index in range(3, 15, 2):
                res.append(self._decode_int16(datas[index:index + 2]))
                index += 2

            elif index == 15:
                byte_1 = bin(datas[index])[2:]
                while len(byte_1) < 6:
                    byte_1 = "0" + byte_1
                res.append(byte_1)
                index += 1

            elif index in (16, 17):
                res.append(datas[index] / 10)
                index += 1

            elif index in range(18, 26, 2):
                res.append(self._int2angle(self._decode_int16(datas[index:index + 2])))
                index += 2

            elif index in range(26, 32, 2):
                res.append(self._int2angle(self._decode_int16(datas[index:index + 2])))
                index += 2

            elif index in range(32, 42, 1):
                res.append(datas[index])
                index += 1
            else:
                index += 1

        return res

    def restore(self):
        """Motor stall recovery"""
        self._mesg(ProtocolCode.RESTORE.value, 1)

    def set_gyro_state(self, state=1):
        """Set gyroscope calibration status (save after power failure)

        Args:
            state (int, optional): 1 - open. 0 - close. Defaults to 1.
        """
        if state not in (0, 1):
            raise ValueError("state must be 0 or 1")
        self._mesg(ProtocolCode.SET_GYRO_STATE.value, state)

    def get_gyro_state(self):
        """Get gyroscope calibration status

        Return:
            1 - open
            0 - close
        """
        return self._mesg(ProtocolCode.GET_GYRO_STATE.value, has_reply=True)

    def get_modified_version(self):
        return self._mesg(ProtocolCode.GET_MODIFIED_VERSION.value, has_reply=True)
