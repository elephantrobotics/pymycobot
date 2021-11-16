#!/usr/bin/env python3
# coding:utf-8
import socket
import serial
import time
import re


class MycobotServer:

    def __init__(self, host, port):
        self.mc = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        print("Binding succeeded!")
        self.s.listen(1)
        self.connect()

    def connect(self):
        try:
            while True:
                conn, addr = self.s.accept()
                port_baud = []
                while True:
                    try:
                        data = conn.recv(1024)
                        command = data.decode('utf-8')
                        if data.decode('utf-8') == "":
                            print("client dsiconnect!")
                            break
                        res = b'None'
                        command = command.replace(" ", "")

                        if len(port_baud) < 3:

                            port_baud.append(command)
                            if len(port_baud) == 3:
                                print(port_baud)
                                self.mc = serial.Serial(
                                    port_baud[0], port_baud[1], timeout=float(port_baud[1]))
                                port_baud.append(1)
                        else:
                            command = self.re_data_2(command)
                            self.write(command)
                            res = self.read()

                        conn.sendall(res)
                    except Exception as e:
                        conn.sendall(str.encode("ERROR:"+str(e)))
                        break
        except:
            conn.close()

    def write(self, command):
        self.mc.write(command)
        self.mc.flush()
        time.sleep(0.05)

    def read(self):
        data = None
        if self.mc.inWaiting() > 0:
            data = self.mc.read(self.mc.inWaiting())
        return data

    def re_data_2(self, command):
        r2 = re.compile(r'[[](.*?)[]]')
        data_str = re.findall(r2, command)[0]
        data_list = [int(i) for i in data_str.split(',')]
        return data_list


if __name__ == "__main__":
    HOST = socket.gethostbyname(socket.gethostname())
    PORT = 9000
    MycobotServer(HOST, PORT)
