#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import fcntl
import logging
import traceback
import socket
import struct
import serial
import Hobot.GPIO as GPIO

GPIO.setwarnings(False)


class GPIOProtocolCode:
    SETUP_GPIO_MODE = 0xAA
    SETUP_GPIO_STATE = 0xAB
    SET_GPIO_OUTPUT = 0xAC
    GET_GPIO_INPUT = 0xAD


def to_string(data: bytes):
    return ' '.join(map(lambda x: f'{x:02x}', data))


def get_local_host(name: str):
    host = None
    dgram_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pack_res = struct.pack('256s', bytes(name, encoding="utf8"))
        host = socket.inet_ntoa(fcntl.ioctl(dgram_socket.fileno(), 0x8915, pack_res)[20:24])
    except Exception as e:
        print(e)
    finally:
        dgram_socket.close()
        return host


localhost = get_local_host("wlan0")


class SocketTransport(object):
    def __init__(self, host="0.0.0.0", port=30002):
        self.port = port
        self.host = host
        self.running = True
        self.log = logging.getLogger("socket")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(5)
        if host == "0.0.0.0":
            host = localhost
        self.log.info(f"start listening on {host}:{port}")

    def accept(self):
        while self.running is True:
            yield self.socket.accept()

    def context(self, conn, buffer_size=1024):
        while self.running is True:
            try:
                data_buffer = conn.recv(buffer_size)
                if len(data_buffer) == 0:
                    break
                yield data_buffer
            except Exception as e:
                self.log.error(f"error while reading socket: {e}")
                traceback.print_exc()
                break

    def close(self):
        self.log.info(f"close socket on {self.host}:{self.port}")
        self.running = False
        self.socket.close()


class SerialTransport(object):
    def __init__(self, comport="/dev/ttyS1", baudrate=100_0000, timeout=None):
        self.serial = serial.Serial(port=comport, baudrate=baudrate, timeout=timeout)
        self.log = logging.getLogger("serial")
        self.baudrate = baudrate
        self.comport = comport
        self.open()
        self.log.info(f"start serial on [{self.comport}] with baudrate [{self.baudrate}]")

    def send(self, data):
        self.serial.write(data)

    def recv(self, size=1024):
        return self.serial.read(size)

    @property
    def is_open(self):
        return self.serial.is_open

    def close(self):
        self.serial.close()

    def open(self):
        if not self.serial.is_open:
            self.serial.open()


class MyCobot280RDKX5Server(object):
    """
    Server for 280 RDK-X5

    1. System GPIO operating protocol adheres to protocol MyCobot 280 RDK X5.
    2. This server only does the work of forwarding protocol and does not participate in the analysis of instructions.
    3. The server is only responsible for forwarding the data received from the socket to the serial port and vice versa.
    4. Instruction parsing is done entirely by the client
    5. The server is responsible for setting the GPIO mode
    """
    def __init__(self, socket_transport, serial_transport, debug=True):
        self.debug = debug
        self.socket_transport = socket_transport
        self.serial_transport = serial_transport
        self.log = logging.getLogger("server")

    def mainloop(self):
        try:
            self.log.info("tcp server started.")
            for conn, addr in self.socket_transport.accept():
                self.log.info(f"{addr} accepted!")
                for data in self.socket_transport.context(conn, buffer_size=1024):
                    self.serial_transport.log.info(f"{addr} recv << {to_string(data)}")
                    if data[3] == GPIOProtocolCode.SETUP_GPIO_MODE:
                        try:
                            mode = GPIO.BCM if data[4] == 0 else GPIO.BOARD
                            GPIO.setmode(mode)
                            self.log.debug(f"{addr} setup gpio mode => {mode}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0x01, 0xfa])
                        except Exception as e:
                            self.log.error(f"{addr} setup gpio mode error: {e}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0xff, 0xfa])

                    elif data[3] == GPIOProtocolCode.SETUP_GPIO_STATE:
                        try:
                            mode = GPIO.OUT if data[5] == 1 else GPIO.IN
                            level = GPIO.HIGH if data[6] == 1 else GPIO.LOW
                            self.log.debug(f"{addr} setup gpio state, mode => {mode}, level => {level}")
                            GPIO.setup(data[4], mode, initial=level)
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0x01, 0xfa])
                        except Exception as e:
                            self.log.error(f"{addr} setup gpio state error: {e}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0xff, 0xfa])

                    elif data[3] == GPIOProtocolCode.SET_GPIO_OUTPUT:
                        try:
                            level = GPIO.HIGH if data[5] == 1 else GPIO.LOW
                            self.log.debug(f"{addr} set gpio output, level => {level}")
                            GPIO.output(data[4], level)
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0x01, 0xfa])
                        except Exception as e:
                            self.log.error(f"{addr} set gpio output error: {e}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0xff, 0xfa])

                    elif data[3] == GPIOProtocolCode.GET_GPIO_INPUT:
                        try:
                            self.log.debug(f"{addr} get gpio input, channel => {data[4]}")
                            level = GPIO.input(data[4])
                            self.log.debug(f"{addr} get gpio input, level => {level}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], level, 0xfa])
                        except Exception as e:
                            self.log.error(f"{addr} get gpio input error: {e}")
                            serial_data = bytes([0xfe, 0xfe, 0x03, data[3], 0xff, 0xfa])
                    else:
                        self.serial_transport.send(data)
                        serial_data = self.serial_transport.recv()

                    self.serial_transport.log.info(f"{addr} send >> {to_string(serial_data)}")
                    conn.send(serial_data)
                else:
                    self.log.info(f"{addr} closed!")
        except Exception as e:
            self.log.error(f"server error: {e}")
            self.log.exception(traceback.format_exc())
        finally:
            self.socket_transport.close()
            self.serial_transport.close()
            self.log.info("server closed")


def main(debug=False):
    logging.basicConfig(
        level=logging.DEBUG if debug else logging.INFO,
        format="%(asctime)s - [%(name)s] %(levelname)7s - %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("server.log")
        ]
    )

    socket_transport = SocketTransport(host="0.0.0.0", port=30002)

    serial_transport = SerialTransport(comport="/dev/ttyS1", baudrate=100_0000, timeout=0.1)

    MyCobot280RDKX5Server(socket_transport, serial_transport).mainloop()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
