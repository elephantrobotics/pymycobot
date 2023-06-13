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
import RPi.GPIO as GPIO

"""
Instructions for use:

Please update pymycobot to the latest version before use.

`pip install pymycobot --upgrade`

Please change the parameters passed in MycobotServer in line 141 according to your model.


The default model is the 280PI.

    The default parameters are: 

        serial_num: /dev/ttyAMA0

        baud: 1000000


"""

def get_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    #DATE_FORMAT = "%m/%d/%Y %H:%M:%S %p"

    formatter = logging.Formatter(LOG_FORMAT)
    # console = logging.StreamHandler()
    # console.setFormatter(formatter)

    save = logging.handlers.RotatingFileHandler(
        "server.log", maxBytes=10485760, backupCount=1)
    save.setFormatter(formatter)

    logger.addHandler(save)
    # logger.addHandler(console)
    return logger


class MycobotServer(object):

    def __init__(self, host, port, serial_num = "/dev/ttyAMA0", baud = 1000000):
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
                        command = data.decode('utf-8')
                        if data.decode('utf-8') == "":
                            print("close disconnect!")
                            break
                        res = b'1'
                        command = command.replace(" ", "")
                        if self.mc.closed() == True:
                            self.mc.open()
                        else:
                            self.logger.info(command)
                            command = self.re_data_2(command)
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
                            res = self.read()
                            if res == None:
                                res = b'1'
                        conn.sendall(res)
                    except Exception as e:
                        self.logger.error(str(e))
                        conn.sendall(str.encode("ERROR:"+str(e)))
                        break
            except Exception as e:
                self.logger.error(str(e))
                conn.close()
                self.mc.close()

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
        data_list = data_str.split(",")
        data_list = [int(i) for i in data_list]
        return data_list


if __name__ == "__main__":
    ifname = "wlan0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', bytes(ifname,encoding="utf8")))[20:24])
    PORT = 9000
    print("ip: {} port: {}".format(HOST, PORT))
    MycobotServer(HOST, PORT, "/dev/ttyAMA0", 1000000)
