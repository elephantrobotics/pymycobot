#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import json
import struct
import socket
import threading
from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.error import calibration_parameters

"""
for mercury x1 robot arms (six-axis or seven-axis), using socket to communicate with the robot
"""


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
        self.arm = arm
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()
        self.lock_out = threading.Lock()
        self.read_threading = threading.Thread(target=self.read_thread, args=("socket", ))
        self.read_threading.daemon = True
        self.read_threading.start()
        self.get_limit_switch()

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
        log_command = " ".join(map(lambda n: hex(n)[2:], command))
        self.log.debug("_write: {}".format(log_command))
        self.sock.sendall(self.__format(command))



























