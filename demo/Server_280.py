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
import RPi.GPIO as GPIO

"""
Instructions for use:

Please update pymycobot to the latest version before use.

`pip install pymycobot --upgrade`

Please change the parameters passed in the last line of the Server.py file, MycobotServer, based on your model.


The default model is the 280PI.

    The default parameters are: 

        serial_num: /dev/ttyAMA0

        baud: 1000000


"""

has_return = [0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1A, 0x1B,
              0x20, 0x21, 0x22,
              0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2D, 0x30, 0x31, 0x32, 0x33, 0x35, 0x36, 0x37,
              0x38, 0x39, 0x3A,
              0x3B, 0x3C, 0x3D, 0X3E, 0x4A, 0x4B, 0x4C, 0x4D, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x60,
              0x61, 0x62, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0xA0, 0xA1, 0xB0, 0xB1, 0xB2,
              0xC0, 0xC1, 0xC2, 0xC3, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0xD4, 0xD5, 0xD6,
              0xD7, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xB0]


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


class MycobotServer(object):

    def __init__(self, host, port, serial_num="/dev/ttyAMA0", baud=1000000):
        """Server class

        Args:
            host: server ip address.
            port: server port.
            serial_num: serial number of the robot.The default is /dev/ttyAMA0.
            baud: baud rate of the serial port.The default is 1000000.

        """
        try:
            GPIO.setwarnings(False)
        except:
            pass
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
                            self.logger.info("get command: {}".format([hex(v) for v in command]))
                            # command = self.re_data_2(command)
                            if command[3] == 170:
                                if command[4] == 0:
                                    GPIO.setmode(GPIO.BCM)
                                else:
                                    GPIO.setmode(GPIO.BOARD)
                            elif command[3] == 171:
                                if command[5]:
                                    GPIO.setup(command[4], GPIO.OUT)
                                else:
                                    GPIO.setup(command[4], GPIO.IN)

                            elif command[3] == 172:
                                GPIO.output(command[4], command[5])

                            elif command[3] == 173:
                                res = bytes(GPIO.input(command[4]))

                            self.write(command)
                            # if command[3] in has_return:
                            res = self.read(command)
                            self.logger.info("return datas: {}".format([hex(v) for v in res]))

                            conn.sendall(res)
                    except Exception as e:
                        self.logger.error(traceback.format_exc())
                        conn.sendall(str.encode(traceback.format_exc()))
                        break
            except Exception as e:
                self.logger.error(traceback.format_exc())
                conn.close()
                self.mc.close()

    def write(self, command):
        self.mc.write(command)
        self.mc.flush()

    def read(self, command):
        datas = b""
        data_len = -1
        k = 0
        pre = 0
        t = time.time()
        wait_time = 0.5
        if command[3] in [0x87, 0x54]:
            wait_time = 1
        while True and time.time() - t < wait_time:
            data = self.mc.read()
            k += 1
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
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', bytes(ifname, encoding="utf8")))[20:24])
    PORT = 9000
    print("ip: {} port: {}".format(HOST, PORT))
    MycobotServer(HOST, PORT, "/dev/ttyAMA0", 1000000)
