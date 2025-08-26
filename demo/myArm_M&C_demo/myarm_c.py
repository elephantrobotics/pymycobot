#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import threading
import traceback
import socket
import typing as T
from serial import SerialException
from pymycobot import MyArmC
import utils


class Config:
    host = "0.0.0.0"            # The host address to listen on
    port = 8001                 # The port to listen on
    maximum_connections = 5     # The maximum number of connections allowed


def setup_robotic_connect(comport: str) -> T.Optional[MyArmC]:
    robotic_api = None
    try:
        robotic_api = MyArmC(comport)
    except Exception as e:
        print(f" # (Error) Error while connecting robotic: {e}")
    return robotic_api


class SocketTransportServer(threading.Thread):
    def __init__(self, host="0.0.0.0", port=8001, listen_size=5):
        super().__init__(daemon=True)
        self.port = port
        self.host = host
        self.running = True
        self.listen_size = listen_size
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(listen_size)
        self.socket_client_table = {}
        if host == "0.0.0.0":
            host = utils.get_local_host()
        print(f" # (Info) Start listening on {host}:{port}")

    def accept(self):
        while self.running is True:
            try:
                yield self.socket.accept()
            except OSError:
                print(f" # (Error) Error while accepting socket")
                continue

    def context(self, address, connection):
        while self.running is True:
            try:
                data_buffer = connection.recv()
                if len(data_buffer) == 0:
                    print(f" # (Warn) Close connection from {address}")
                    del self.socket_client_table[address]
                    connection.close()
                    break
                yield data_buffer
            except Exception as e:
                print(f" # (Error) Error while reading socket: {e}")
                traceback.print_exc()
                break

    def run(self):
        for conn, addr in self.accept():
            print(f" # (Info) Accept connection from {addr}")
            self.socket_client_table[addr] = conn

    def sendall(self, data):
        exit_addr_list = []
        for address, connection in self.socket_client_table.items():
            print(f" # (Info) {address} => {data}")
            try:
                connection.send(data)
            except WindowsError:
                print(f" # (Warn) Close connection from {address}")
                exit_addr_list.append(address)

        for exit_addr in exit_addr_list:
            del self.socket_client_table[exit_addr]

    def close(self):
        print(f" # (Info) close socket on {self.host}:{self.port}")
        self.running = False
        self.socket.close()


def main():
    serial_ports = utils.get_local_serial_port()
    if len(serial_ports) == 0:
        print(" # (Warn) No serial ports found. exit")
        return

    serial_comport = utils.select("# (Info) Please select a serial port to connect robotic arm:", serial_ports, 1)

    print(f" # (Info) Selected {serial_comport} to connect robotic arm")
    robotic_api = setup_robotic_connect(serial_comport)
    if robotic_api is None:
        print(" # (Error) Failed to connect robotic arm. exit")
        return

    print(" # (Info) Start listening for changes in the angle of the robotic arm")
    print(" # (Info) Press 【Ctrl+C】 to exit")
    transport_server = SocketTransportServer(
        host=Config.host,
        port=Config.port,
        listen_size=Config.maximum_connections
    )
    transport_server.start()
    while True:
        try:
            angles = robotic_api.get_joints_angle()
            if angles is None:
                continue

            print(f" # (Info) Current angle: {angles}")
            if max(angles) > 200 or min(angles) < -200:
                print(" # (Warn) There is no communication between the joints, please check the connection")
                continue

            transport_server.sendall(f"\n{angles}".encode('utf-8'))
        except KeyboardInterrupt:
            print(" # (Info) Exit")
            break

        except SerialException as e:
            print(f" # (Error) Serial port error: {e}")
            break

        except Exception as e:
            print(f" # (Error) {e}")

    transport_server.close()
    robotic_api._serial_port.close()
    print(" # (Info) Close socket and serial port")
    print(" # (Info) Exit")


if __name__ == "__main__":
    main()
