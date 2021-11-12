import time
import socketserver
from pymycobot.mycobot import MyCobot
import numpy as np


class TCPMessageHandler(socketserver.StreamRequestHandler):

    def handle(self):
        """
        Process socket data

        """
        # process received data.
        while True:
            try:
                # data=bytes.decode(self.rfile.readline().strip())
                data = bytes.decode(self.request.recv(1024))
                print(data)
            except Exception as ex:
                print(ex)
                return

            data = data.split(',')
            commend = []

            for dt in data:
                try:
                    if dt.isdigit():
                        commend.append(int(dt))
                    else:
                        commend.append(float(dt))
                except ValueError:
                    commend.append(None)

            if len(commend) > 0:
                print("Received:%s" % commend)
                self.request.sendall(str.encode(str(commend)))


class SocketServer(object):
    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port

    def start_server(self):
        while True:
            try:
                server = socketserver.TCPServer((self.host, self.port),
                                                TCPMessageHandler)
                break
            except:
                time.sleep(1)
                continue
        try:
            print("server open seccess")
            server.serve_forever()

        finally:
            print("server close")
            server.server_close()


class Robot(object):
    def __init__(self, serial, baud):
        super().__init__()
        self.serial = serial
        self.baud = baud

    def start_Robot(self):
        try:
            mc = MyCobot(self.serial, self.baud)  # 树莓派版本打开机械臂，串口固定，无需USB连接
            print(mc)  # 打印机械臂端口信息
        except:
            print("can not find cobot")
            exit(0)


if __name__ == "__main__":
    host = '192.168.10.191'  # 输入本机IP地址
    port = 9000
    serial = ''
    baud = 1000000
    server = SocketServer(host, port)  # 声明服务器端口
    mycobot = Robot(serial, baud)  # 声明机械臂接口
    server.start_server()  # 永久打开服务器
