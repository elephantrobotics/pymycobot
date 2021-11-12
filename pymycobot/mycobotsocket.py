# coding=utf-8

from __future__ import division
import socket
import time
import sys
# import threading


class MyCobotSocket:
    """MyCobot Python API Socket communication class."""

    def __init__(self, ip, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        self.SERVER_IP = ip  # 输入正确的目标ip地址，请查看树莓派ip
        self.SERVER_PORT = 9000
        self.sock = self.connect()
        self.send_command(port)
        self.send_command(baudrate)

    def connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))  # 打开链接
        return sock

    def send_command(self, command):
        # 发送数据
        self.sock.sendall((command+"~").encode())
        # 接收数据
        data = self.sock.recv(1024)
        if data.decode() == "None":
            return ""
        return data.decode()
