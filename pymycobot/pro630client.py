# coding=utf-8
import socket
import sys
import time
import traceback

from pymycobot.pro630 import Pro630Api
from pymycobot.common import ProtocolCode

SYS_VERSION_INFO = sys.version_info.major


def format_hex_log(data):
    if SYS_VERSION_INFO == 2:
        command_log = ""
        for d in data:
            command_log += hex(ord(d))[2:] + " "
    else:
        command_log = ""
        for d in data:
            command_log += hex(d)[2:] + " "

    return command_log


def connect_socket(addr, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((addr, port))
    return sock


class Pro630Client(Pro630Api):
    def __init__(self, host, port=9000, timeout=0.1, debug=False):
        """
        Args:
            host: host address
            port: port number
            timeout: socket timeout
            debug: debug mode
        """
        self.sock = connect_socket(host, port)
        self.timeout = timeout
        self.host = host
        self.port = port
        super().__init__(debug=debug, method='socket')

    def close(self):
        self.sock.close()

    def read_thread(self, method=None):
        self.sock.settimeout(self.timeout)
        while True:
            try:
                data = self.sock.recv(1024)
                print(data)
                result = self._process_received(data)
                command_log = format_hex_log(result)
                self.log.debug(f"_read :{command_log}")
                if not result:
                    continue

                with self.lock:
                    self.read_command.append([result, time.time()])
            except socket.timeout:
                print("socket timeout")
                time.sleep(0.1)
            except Exception as e:
                print(e)
                traceback.print_exc()

    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output.IO low-level output high-level, high-level output high resistance state

        Args:
            pin_no: pin port number. range 1 ~ 6
            pin_signal: 0 / 1
        """
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_basic_input(self, pin_no):
        """Get basic input.

        Args:
            pin_no: pin port number. range 1 ~ 6
            
        Return:
            1 - high
            0 - low
        """
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)

    def send_angles_sync(self, angles, speed):
        self.calibration_parameters(class_name = self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, no_return=True)

    def set_monitor_mode(self, mode):
        raise NotImplementedError("Pro630 does not support monitor mode")

    def get_monitor_mode(self):
        raise NotImplementedError("Pro630 does not support monitor mode")