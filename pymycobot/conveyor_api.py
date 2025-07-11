import logging
import threading
import time
import struct
import serial


def setup_serial_connect(port, baudrate, timeout=None):
    serial_api = serial.Serial()
    serial_api.port = port
    serial_api.baudrate = baudrate
    serial_api.timeout = timeout
    serial_api.rts = False
    serial_api.dtr = False
    serial_api.open()
    return serial_api


class CommandGenre(object):
    SET_SERVO_DIRECTION = 0XA0
    GET_SERVO_DIRECTION = 0XA2

    SET_SERVO_SPEED = 0XA5
    GET_SERVO_SPEED = 0XA3

    # WRITE_SERVO_ANGLE = 0XA6
    # WRITE_SERVO_STEP = 0XA7

    READ_FIRMWARE_VERSION = 0XAA


class Command(object):
    HEADER = 0XFF

    def __init__(self, genre, address, check_digit, params, length=None):
        self.__header = [0xff, 0xff]
        self.__length = length or len(params)
        self.__address = address
        self.__params = params
        self.__genre = genre
        self.__check_digit = check_digit

    @property
    def genre(self):
        return self.__genre

    def to_bytes(self):
        return bytes([*self.__header, self.__address, self.__length, *self.__params, self.__genre, *self.__check_digit])

    def get_params(self):
        return self.__params[0] if self.__length == 1 else self.__params

    def __str__(self):
        return ' '.join(f'{x:02x}' for x in self.to_bytes())

    def __bytes__(self):
        return self.to_bytes()

    @classmethod
    def packing(cls, genre: int, addr: int, *params):
        check_code = cls.check_digit(genre, params)
        return cls(genre=genre, address=addr, check_digit=(check_code, ), params=(*params,))

    @classmethod
    def parsing(cls, buffer: bytes):
        # header, header, addr, length, params, genre, check_bits
        if len(buffer) < 6:
            return None
        if buffer[0] != cls.HEADER or buffer[1] != cls.HEADER:
            return None
        length = buffer[3]
        return cls(genre=buffer[-2], address=buffer[2], length=length, params=(*buffer[4:4+length], ), check_digit=buffer[4+length:4+length+1])

    @classmethod
    def unpack_args(cls, *parameters):
        bits_pack_list = []
        for param in parameters:
            pair = struct.pack('>h', param)
            if len(pair) == 2:
                bits_pack_list.extend(list(pair))
            else:
                bits_pack_list.clear()
        return bits_pack_list

    @classmethod
    def check_digit(cls, genre, params):
        """
        Calculate the check-code for the command.
        :param genre: int, function genre
        :param params: bytes, function parameters
        :return: int, check-code
        """
        return sum([genre, *params]) & 0xff

    @classmethod
    def has_header(cls, buffer: bytes):
        """
        Check if the buffer contains a header.
        :param buffer:
        :return:
        """
        if len(buffer) < 2:
            return False
        return buffer[0] == cls.HEADER and buffer[1] == cls.HEADER


class SerialProtocol(object):

    def __init__(self, comport, baudrate, timeout=0.5):
        self._comport = comport
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial_port = setup_serial_connect(port=comport, baudrate=baudrate, timeout=timeout)

    def open(self):
        if self._serial_port.is_open is False:
            self._serial_port.open()

    def close(self):
        if self._serial_port.is_open is True:
            self._serial_port.close()

    def write(self, data: bytes):
        self._serial_port.write(data)
        self._serial_port.flush()

    def read(self, size=None):
        return self._serial_port.read(size or self._serial_port.in_waiting)

    def flush(self):
        self._serial_port.flush()


class ConveyorAPI(SerialProtocol):
    class MotorModel:
        STEPPER_MOTOR_42 = 0x30
        STEPPER_MOTOR_57 = 0x31

    def __init__(self, comport, baudrate="115200", timeout=0.1, debug=False):
        super().__init__(comport, baudrate, timeout)
        self._debug = debug
        self.open()
        self._mutex = threading.Lock()
        self._log = logging.getLogger("conveyor_api")
        self._log.setLevel(logging.DEBUG if debug else logging.INFO)
        handler = logging.StreamHandler()
        handler.setLevel(logging.DEBUG if debug else logging.INFO)
        formatter = logging.Formatter(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        self._log.addHandler(handler)

    def _wait_for_reply(self, timeout=None):
        buffers = b""
        timeout = timeout or self._timeout
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < timeout:
            if self._serial_port.in_waiting <= 0:
                continue
            buffers = self.read(self._serial_port.in_waiting)
            if not Command.has_header(buffers):
                continue
            break

        return Command.parsing(buffers) if Command.has_header(buffers) else None

    def _merge(self, genre, address, *parameters, has_reply=False):
        command = Command.packing(genre, address, *parameters)
        self._log.debug(f"write > {command}")
        with self._mutex:
            self.write(command.to_bytes())

            if not has_reply:
                return

            reply_command = self._wait_for_reply(timeout=0.07)  # WaitForAReply
            self._log.debug(f"read  < {reply_command}")
            if not reply_command:
                return None

            result = reply_command.get_params()
            if reply_command.genre == CommandGenre.READ_FIRMWARE_VERSION:
                return result / 10
            return result

    def set_motor_direction(self, direction, motor_model=MotorModel.STEPPER_MOTOR_57):
        """Modify the direction of movement of the conveyor belt"""
        if direction not in (0, 1):
            raise ValueError("direction must be 0 or 1")
        self._merge(CommandGenre.SET_SERVO_DIRECTION, motor_model, direction)

    def get_motor_direction(self, motor_model=MotorModel.STEPPER_MOTOR_57):
        """Get the direction of movement of the conveyor belt"""
        return self._merge(CommandGenre.GET_SERVO_DIRECTION, motor_model, has_reply=True)

    def get_motor_speed(self, motor_model=MotorModel.STEPPER_MOTOR_57):
        """Get the speed of the conveyor belt"""
        return self._merge(CommandGenre.GET_SERVO_SPEED, motor_model, has_reply=True)

    def set_motor_speed(self, status, speed: int, motor_model=MotorModel.STEPPER_MOTOR_57):
        """Modify the speed of the conveyor belt"""
        if status not in (0, 1):
            raise ValueError("status must be 0 or 1")

        if not 0 <= speed <= 100:
            raise ValueError("speed must be in range [0, 100]")
        return self._merge(CommandGenre.SET_SERVO_SPEED, motor_model, status, speed)

    # def write_motor_angle(self, carry, angle: int, speed,  motor_model=MotorModel.STEPPER_MOTOR_57):
    #     return self._merge(CommandGenre.WRITE_SERVO_ANGLE, motor_model, carry, angle, speed)
    #
    # def write_motor_step(self, carry, step: int, speed, direction, motor_model=MotorModel.STEPPER_MOTOR_57):
    #     return self._merge(CommandGenre.WRITE_SERVO_STEP, motor_model, carry, step, speed, direction)

    def read_firmware_version(self, motor_model=MotorModel.STEPPER_MOTOR_57):
        """Get the firmware version of the conveyor belt"""
        return self._merge(CommandGenre.READ_FIRMWARE_VERSION, motor_model, has_reply=True)

