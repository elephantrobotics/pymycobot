import enum
import threading
import serial
import time
import struct
import logging
import logging.handlers


def setup_logging(name: str = __name__, debug: bool = False) -> logging.Logger:
    debug_formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s [%(name)s] - %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger(name)
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(debug_formatter)
    if debug is True:
        logger.addHandler(stream_handler)
        logger.setLevel(logging.INFO)  # 100M日志
        file_handler = logging.handlers.RotatingFileHandler(
            filename="python_debug.log", maxBytes=100 * 1024 * 1024, backupCount=1
        )
        file_handler.setFormatter(debug_formatter)
        logger.addHandler(file_handler)

    else:
        logger.setLevel(logging.DEBUG)

    return logger


class ProtocolCode(enum.Enum):
    HEADER = (0xFE, 0xFE)
    RESTORE = (0x01, 0x00)
    SET_LED = (0x01, 0x02)
    SET_LED_MODE = (0x01, 0x0A)
    GET_FIRMWARE_VERSION = (0x01, 0x03)
    GET_MODIFIED_VERSION = (0x01, 0x09)
    SET_GYRO_STATE = (0x01, 0x07)
    GET_GYRO_STATE = (0x01, 0x08)
    GET_MCU_INFO = (0x01, 0x0B)
    UNDEFINED = ()


class SerialStreamProtocol(object):

    def __init__(self, port="/dev/ttyAMA0", baudrate=115200, timeout=0.1):
        self._serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._serial_port.rts = False

    def open(self):
        if self._serial_port.is_open is False:
            self._serial_port.open()

    def close(self):
        if self._serial_port.is_open is True:
            self._serial_port.close()

    def flush(self):
        self._serial_port.flush()

    def write(self, data):
        self._serial_port.write(data)
        self._serial_port.flush()

    def read(self, size):
        return self._serial_port.read(size)

    def readall(self):
        return self._serial_port.read_all()


class CommandProtocol(object):

    def _process_data_command(self, args):
        if not args:
            return []

        processed_args = []
        for index in range(len(args)):
            if isinstance(args[index], list):
                data = self._encode_int16(args[index])
                processed_args.extend(data)
            else:
                processed_args.append(args[index])

        return processed_args

    def _flatten(self, datas):
        flat_list = []
        for item in datas:
            if not isinstance(item, list):
                flat_list.append(item)
            else:
                flat_list.extend(self._flatten(item))
        return flat_list

    @classmethod
    def _float(cls, number, decimal=2):
        return round(number / 10 ** decimal, 2)

    @classmethod
    def _encode_int16(cls, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            res = []
            for v in data:
                t = cls._encode_int16(v)
                res.extend(t)
            return res

    @classmethod
    def _decode_int16(cls, data):
        return struct.unpack(">h", data)[0]


class MyAgv(SerialStreamProtocol, CommandProtocol):

    def __init__(self, comport, baudrate=115200, timeout=0.1, debug=False):
        super().__init__(comport, baudrate, timeout)
        self.__movement = False
        self.__command_buffer_table = {}
        self.__mutex = threading.Lock()
        self.log = setup_logging(name=self.__class__.__name__, debug=debug)
        self._command_read_thread = threading.Thread(target=self._read_command_buffer_thread, daemon=True)
        self._command_read_thread.start()

    @classmethod
    def __is_complete_command(cls, command):
        return sum(command[2:-1]) & 0xff == command[-1] and len(command) > 5

    def _read_command_buffer(self):
        previous_frame = b""
        is_record = False
        commands = b"\xfe\xfe"

        while True:
            current_frame = self.read(1)

            if current_frame == b"\xfe" and previous_frame == b"\xfe" and is_record is False:
                is_record = True
                continue

            previous_frame = current_frame
            if is_record is False:
                continue

            commands += current_frame
            if sum(commands[2:-1]) & 0xff == commands[-1] and len(commands) > 5:
                break
        return commands

    def _read_command_buffer_thread(self):
        while True:
            command_buffers = self._read_command_buffer()
            if self.__is_complete_command(command_buffers):
                self.log.info(f"write: {' '.join(f'{x:02x}' for x in command_buffers)}")
                genre = tuple(command_buffers[2:4])
                if genre == (128, 128):
                    genre = ProtocolCode.GET_MCU_INFO.value
                self.__command_buffer_table[genre] = (time.perf_counter(), command_buffers)
            time.sleep(0.008)

    def _compose_complete_command(self, genre: ProtocolCode, params):  # packing command
        command_args = self._process_data_command(params)
        command_args = self._flatten(command_args)

        command = [*ProtocolCode.HEADER.value]
        if isinstance(genre.value, tuple):
            command.extend(genre.value)
        else:
            command.append(genre.value)

        command.extend(command_args)
        command.append(sum(command[2:]) & 0xff)
        return command

    def _parse_reply_instruction(self, genre: ProtocolCode):  # unpacking command
        timestamp, reply_data = self.__command_buffer_table.get(genre.value, (time.perf_counter(), []))
        if not reply_data:
            return None

        self.log.info(f"read : {' '.join(f'{x:02x}' for x in reply_data)}")
        if genre == ProtocolCode.GET_FIRMWARE_VERSION:
            return self._float(reply_data[4], 1)
        elif genre == ProtocolCode.GET_MCU_INFO:
            if len(reply_data) < 30:
                return None
            index = 0
            res = []
            datas = reply_data[2:][:-1]  # header and footer frames are not counted
            while index < len(datas):
                if index in range(0, 3):
                    res.append(datas[index])
                    index += 1

                elif index in range(3, 15, 2):
                    data = self._decode_int16(datas[index:index + 2])
                    res.append(self._float(data, 2))
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
                    res.append(self._float(self._decode_int16(datas[index:index + 2]), 3))
                    index += 2

                elif index in range(26, 32, 2):
                    res.append(self._float(self._decode_int16(datas[index:index + 2]), 3))
                    index += 2

                elif index in range(32, 42, 1):
                    res.append(datas[index])
                    index += 1
                else:
                    index += 1

            return res
        return reply_data[4]

    def _merge(self, genre: ProtocolCode, *args, has_reply=False, in_buffer=False):
        if in_buffer is False:
            real_command = self._compose_complete_command(genre, args)
            self.log.info(f"write: {' '.join(f'{x:02x}' for x in real_command)}")
            with self.__mutex:
                self.write(real_command)
                if has_reply is False:
                    return None
        time.sleep(0.1)
        return self._parse_reply_instruction(genre)

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
            self._merge(ProtocolCode.UNDEFINED, *genre)
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
        self._merge(ProtocolCode.UNDEFINED, 128, 128, 128)

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
        return self._merge(ProtocolCode.GET_MCU_INFO, has_reply=True, in_buffer=True)

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
