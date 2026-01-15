#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import json
import struct
import socket
import sys
import threading
import time
import traceback
from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.error import calibration_parameters

"""
for mercury x1 robot arms (six-axis or seven-axis), using socket to communicate with the robot
"""

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


class MercuryArmsSocket(MercuryCommandGenerator):
    def __init__(self, arm, ip, netport=9000, debug=False):
        """
        Arm socket connection
        Args:
            arm: 'left_arm' or 'right_arm'
            ip: ip address
            netport: port
            debug: debug mode
        """
        super(MercuryArmsSocket, self).__init__(debug)
        self.crc_robot_class.append(self.__class__.__name__)
        self.arm = arm
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()

        self.read_threading = threading.Thread(target=self.read_thread, args=("socket",))
        self.read_threading.daemon = True
        self.read_threading.start()
        self.get_limit_switch()

    def read_thread(self, method=None):
        self.sock.settimeout(3)
        while True:
            try:
                data = self.sock.recv(1024)
                result = self._process_received(data)

                self.log.debug(f"_read :{format_hex_log(result)}")
                if not result:
                    continue

                with self.lock:
                    self.read_command.append([result, time.time()])

            except socket.timeout:
                time.sleep(0.1)

            except Exception as e:
                print(e)
                traceback.print_exc()

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

    def open(self):
        self.sock = self.connect_socket()

    def close(self):
        self.sock.close()
        self.sock = None

    def __format(self, command: str) -> bytes:
        send_data = {"command": command, "arm": self.arm}
        data_byter = json.dumps(send_data).encode('utf-8')
        date_length = struct.pack('!I', len(data_byter))
        return b''.join([date_length, data_byter])

    def _write(self, command, method=None):
        self.log.debug("_write: {}".format(format_hex_log(command)))
        self.sock.sendall(self.__format(command))

    def _send_command(self, genre, real_command):
        self.write_command.append(genre)
        self._write(self._flatten(real_command), method="socket")


if __name__ == '__main__':
    mercury_socket = MercuryArmsSocket("left_arm", ip="192.168.1.216", debug=True)
    mercury_socket.send_angle(1, 20, 10)
    # while True:
    #     print(mercury_socket.get_angles())
    #     time.sleep(0.1)
