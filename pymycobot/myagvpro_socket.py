#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
from pymycobot.myagvpro import MyAGVProCommandApi


def setup_socket_connect(host, port, timeout=0.1):
    socket_api = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_api.settimeout(timeout)
    socket_api.connect((host, port))
    return socket_api


class MyAGVProSocket(MyAGVProCommandApi):
    def __init__(self, host, port, debug=False, save_serial_log=False):
        super().__init__(debug=debug, save_serial_log=save_serial_log)
        self._socket = setup_socket_connect(host=host, port=port)
        self._serial_filename = 'agvpro_socket_serial.log'
        self._communication_mode = 1

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __enter__(self):
        return self

    def write(self, command):
        return self._socket.send(command)

    def read(self, size=1):
        try:
            return self._socket.recv(size)
        except socket.timeout:
            return b''

    def close(self):
        self._socket.close()

    def clear(self):
        pass
