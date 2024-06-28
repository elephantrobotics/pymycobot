# coding=utf-8

import time
import socket
import threading

from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.error import calibration_parameters


class MercurySocket(MercuryCommandGenerator):
    def __init__(self, ip, netport=9000, debug=False):
        """
        Args:
            ip: Server ip
            netport: Server port(default 9000)
        """
        super(MercurySocket, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()
        self.read_threading = threading.Thread(target=self.read_thread, daemon=True)
        self.read_threading.start()

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock
        
    def open(self):
        self.sock = self.connect_socket()
        
    def close(self):
        self.sock.close()
