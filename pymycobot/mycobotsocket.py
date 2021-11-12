# coding=utf-8

from __future__ import division
import socket
import time


class MyCobotSocket:
    """MyCobot Python API Socket communication class."""

    def __init__(self, ip, port, baudrate="1000000"):
        """
        Args:
            ip       : ip address
            port     : port string
            baudrate : baud rate string, default '1000000'
        """
        # 输入正确的目标ip地址，请查看树莓派ip
        self.SERVER_IP = ip
        # 端口号
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
        self.sock.sendall(command.encode())
        # 接收数据
        data = self.sock.recv(1024)
        res = data.decode()
        if res == "None":
            return ""
        elif 'ERROR' in res:
            raise Exception(command+"\n"+res)
        return res
