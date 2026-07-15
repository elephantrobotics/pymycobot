#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ultraarm_p1_socket.py

Python socket interface for the ultraArmP1 robotic arm.

Author: weijian.wang
Date: 2026-03-11
"""
import socket
import time

import select

from pymycobot.common import ProtocolCode
from pymycobot.ultraarm_p1_base import UltraArmP1Base


class UltraArmP1Socket(UltraArmP1Base):
    """Socket communication interface for ultraArmP1."""

    REQUEST_TIMEOUT = 0.5
    ANGLE_COORD_TIMEOUT = 0.15
    QUEUE_TIMEOUT = 0.15
    SET_RESPONSE_TIMEOUT = 5

    def __init__(self, ip, netport=9000, timeout=0.05, debug=False, _internal_mode=False):
        """Initialize the ultraArmP1 robot communication.

        Args:
            ip     : Server IP address
            netport : Socket port number, default is 9000
            timeout (float, optional): Serial read timeout in seconds. Defaults to 0.05.
            debug (bool, optional): Whether to print debug information. Defaults to False.
        """
        super().__init__(debug, _internal_mode)
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.sock.settimeout(timeout)
        time.sleep(0.5)

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

    def _socket_in_waiting(self):

        ready = select.select([self.sock], [], [], 0)
        if ready[0]:
            return 1
        return 0

    def _read_available_bytes(self):

        if not self._socket_in_waiting():
            return b""
        try:
            data = self.sock.recv(1024)
            return data
        except Exception:
            return b""

    def _send_command(self, command: str, clear_input=True):
        """Send commands to serial port"""
        command = self._append_checksum(command)
        command += ProtocolCode.END
        self._debug_write(command)
        try:
            self.sock.sendall(command.encode())
        except Exception as e:
            self.log.exception(f"socket send error: {e}")
            raise

    def _send_raw_command(self, command: str):
        self.sock.sendall(command.encode())
        time.sleep(0.02)

    def close(self):
        """Close the socket connect."""
        with self.lock:
            try:
                self.sock.close()
            except Exception as e:
                self.log.error(f"socket close error: {e}")

    def open(self):
        """Open the socket connect."""
        with self.lock:
            try:
                self.sock = self.connect_socket()
            except Exception as e:
                self.log.error(f"socket open error: {e}")

