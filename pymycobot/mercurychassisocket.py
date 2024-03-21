#!/usr/bin/python
# -*- coding:utf-8 -*-
import json
import socket
import threading
import time
import traceback


class MercuryChassisSocket(object):
    """
    Mercury X1 mobile chassis car socket class
    """

    def __init__(self, server_host, server_port, debug=False):
        """
        Args:
            server_host: Server ip
            server_port: Server port
        """
        super(MercuryChassisSocket, self).__init__()
        self.SERVER_HOST = server_host
        self.SERVER_PORT = server_port
        self.sock = None
        self.debug = debug
        self.lock = threading.Lock()
        self.connect_to_server_b()

    def connect_to_server_b(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.SERVER_HOST, self.SERVER_PORT))

    def send_command(self, command, data=None):
        with self.lock:
            try:
                payload = {command: data}
                serialized_data = json.dumps(payload).encode('utf-8')
                self.sock.sendall(serialized_data)

                if self.debug:
                    current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                    print("[{}] Sent command: {}, Data: {}".format(current_time, command, data))
            except Exception as e:
                print("Error sending command:", str(traceback.format_exc()))

    def goStraight(self, time, speed):
        command = 'goStraight'
        data = {'time': time, 'speed': speed}
        self.send_command(command, data)

    def goBack(self, time, speed):
        command = 'goBack'
        data = {'time': time, 'speed': speed}
        self.send_command(command, data)

    def turnRight(self, time, speed):
        command = 'turnRight'
        data = {'time': time, 'speed': speed}
        self.send_command(command, data)

    def turnLeft(self, time, speed):
        command = 'turnLeft'
        data = {'time': time, 'speed': speed}
        self.send_command(command, data)

    def stop(self):
        command = 'stop'
        data = {'stop': True, 'speed': 0}
        self.send_command(command, data)
