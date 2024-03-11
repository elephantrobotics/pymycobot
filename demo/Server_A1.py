#!/usr/bin/env python3
# coding:utf-8
import socket
import serial
import time
import logging
import logging.handlers
import re
import fcntl
import struct
import traceback

"""
Instructions for use:

Please update pymycobot to the latest version before use.

`pip install pymycobot --upgrade`

Please change the parameters passed in the last line of the Server.py file, MercuryServer, based on your model.


"""

has_return = [0x02, 0x03, 0x04, 0x09, 0x10, 0x11, 0x12, 0x13, 0x1c, 0x18, 0x19, 0x20, 0x23, 0x27, 0x29, 0x2A, 0x2B, 0x35, 0x4A, 0x4B,0x4C, 0x4D,
              0x50, 0x51, 0x56,0x57, 0x59,0x5A,0x62, 0x82, 0x84, 0x86, 0x88, 0x8A, 0xA1, 0xA2, 0xB2, 0xB3, 0xB4, 0xB5, 0xB7, 0xD6, 0xe1, 0xe2, 0xe4]


def get_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    # DATE_FORMAT = "%m/%d/%Y %H:%M:%S %p"

    formatter = logging.Formatter(LOG_FORMAT)
    console = logging.StreamHandler()
    console.setFormatter(formatter)

    save = logging.handlers.RotatingFileHandler(
        "server.log", maxBytes=10485760, backupCount=1)
    save.setFormatter(formatter)

    logger.addHandler(save)
    logger.addHandler(console)
    return logger


class MercuryServer(object):

    def __init__(self, host, port, serial_num="/dev/ttyAMA1", baud=115200):
        """Server class

        Args:
            host: server ip address.
            port: server port.
            serial_num: serial number of the robot.The default is /dev/ttyAMA1.
            baud: baud rate of the serial port.The default is 115200.

        """
        self.logger = get_logger("AS")
        self.mc = None
        self.serial_num = serial_num
        self.baud = baud
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        print("Binding succeeded!")
        self.s.listen(1)
        self.mc = serial.Serial(self.serial_num, self.baud, timeout=0.1)
        self.connect()

    def connect(self):
        while True:
            try:
                print("waiting connect!------------------")
                conn, addr = self.s.accept()
                while True:
                    try:
                        print("waiting data--------")
                        data = conn.recv(1024)
                        command = []
                        for v in data:
                            command.append(v)
                        if command == []:
                            print("close disconnect!")
                            break
                        if self.mc.isOpen() == False:
                            self.mc.open()
                        else:
                            self.logger.info("get command: {}".format(
                                [hex(v) for v in command]))
                            # command = self.re_data_2(command)

                            self.write(command)
                            if command[3] in has_return:
                                res = self.read(command)
                                self.logger.info(
                                    "return datas: {}".format([hex(v) for v in res]))

                                conn.sendall(res)
                    except Exception as e:
                        self.logger.error(traceback.format_exc())
                        conn.sendall(str.encode(traceback.format_exc()))
                        break
            except Exception as e:
                self.logger.error(traceback.format_exc())
                conn.close()
                self.mc.close()
                
    def _encode_int16(self, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            res = []
            for v in data:
                t = self._encode_int16(v)
                res.extend(t)
        return res
              
    @classmethod  
    def crc_check(cls, command):
        crc = 0xffff
        for index in range(len(command)):
            crc ^= command[index]
            for _ in range(8):
                if crc & 1 == 1:
                    crc >>=  1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        if crc > 0x7FFF:
            return list(struct.pack(">H", crc))
        return cls._encode_int16(_, crc)

    def write(self, command):
        self.mc.write(command)
        self.mc.flush()

    def read(self, command):
        datas = b""
        data_len = -1
        k = 0
        pre = 0
        t = time.time()
        wait_time = 0.1
        if command[3] == 0x10:
            wait_time = 8
        elif command[3] in [0x11, 0x13, 0x18, 0x56, 0x57, 0x29]:
            wait_time = 3
        while True and time.time() - t < wait_time:
            data = self.mc.read()
            k += 1
            if data_len == 3:
                datas += data
                crc = self.mc.read(2)
                if self.crc_check(datas) == [v for v in crc]:
                    datas+=crc
                    break
            if data_len == 1 and data == b"\xfa":
                datas += data
                if [i for i in datas] == command:
                    datas = b''
                    data_len = -1
                    k = 0
                    pre = 0
                    continue
                break
            elif len(datas) == 2:
                data_len = struct.unpack("b", data)[0]
                datas += data
            elif len(datas) > 2 and data_len > 0:
                datas += data
                data_len -= 1
                if len(datas) == 4:
                    if datas[-1] != command[3]:
                        datas = b''
                        data_len = -1
                        k = 0
                        pre = 0
                        continue
            elif data == b"\xfe":
                if datas == b"":
                    datas += data
                    pre = k
                else:
                    if k - 1 == pre:
                        datas += data
                    else:
                        datas = b"\xfe"
                        pre = k
        else:
            datas = b''
        return datas

    def re_data_2(self, command):
        r2 = re.compile(r'[[](.*?)[]]')
        data_str = re.findall(r2, command)[0]
        data_list = data_str.split(",")
        data_list = [int(i) for i in data_list]
        return data_list


if __name__ == "__main__":
    ifname = "wlan0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack(
        '256s', bytes(ifname, encoding="utf8")))[20:24])
    PORT = 9000
    print("ip: {} port: {}".format(HOST, PORT))
    MercuryServer(HOST, PORT, "/dev/ttyAMA1", 115200)
