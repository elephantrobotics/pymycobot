#!/usr/bin/env python2
# coding:utf-8
import socket
import serial
import time
import logging
import logging.handlers
import re
import RPi.GPIO as GPIO


def get_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    LOG_FORMAT = "%(asctime)s - %(levelname)s - %(message)s"
    #DATE_FORMAT = "%m/%d/%Y %H:%M:%S %p"

    formatter = logging.Formatter(LOG_FORMAT)
    # console = logging.StreamHandler()
    # console.setFormatter(formatter)

    save = logging.handlers.RotatingFileHandler(
        "/home/ubuntu/mycobot_server.log", maxBytes=10485760, backupCount=1)
    save.setFormatter(formatter)

    logger.addHandler(save)
    # logger.addHandler(console)
    return logger


class MycobotServer(object):

    def __init__(self, host, port):
        try:
            GPIO.setwarnings(False)
        except:
            pass
        self.logger = get_logger("AS")
        self.mc = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        print "Binding succeeded!"
        self.s.listen(1)
        self.connect()

    def connect(self):
        while True:
            try:
                print "waiting connect!------------------"
                conn, addr = self.s.accept()
                port_baud = []
                while True:
                    try:
                        print "waiting data--------"
                        data = conn.recv(1024)
                        command = data.decode('utf-8')
                        if data.decode('utf-8') == "":
                            print("close dsiconnect!")
                            break
                        res = b'1'
                        command = command.replace(" ", "")
                        if len(port_baud) < 3:

                            port_baud.append(command)
                            if len(port_baud) == 3:
                                self.mc = serial.Serial(
                                    port_baud[0], port_baud[1], timeout=float(port_baud[1]))
                                port_baud.append(1)
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
    HOST = socket.gethostbyname(socket.gethostname())
    PORT = 9000
    MycobotServer(HOST, PORT)
