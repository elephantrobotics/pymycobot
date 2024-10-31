#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import dataclasses
import json
import struct
import fcntl
import threading
import traceback
from typing import Union, Optional, List, Generator
import serial
import socket
import time
import logging
from logging.handlers import RotatingFileHandler
from concurrent.futures import ThreadPoolExecutor, Future
"""
This is a demo server for Mercury X1 robot arm. It can be used to control the robot arm via TCP/IP.
"""


def get_local_host(name: str = "eth0") -> Optional[str]:
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


def init_logging(name: str, level: int = logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level=level)
    handle = RotatingFileHandler(f"{name}.log", maxBytes=1024 * 1024 * 5, backupCount=5, encoding="utf-8")
    handle.setLevel(level=level)
    logger.addHandler(handle)
    return logger


class Arm:
    LEFT_ARM = "left_arm"
    RIGHT_ARM = "right_arm"
    UNKNOWN = "unknown"


@dataclasses.dataclass
class Client:
    arm: str
    address: str
    socket: socket.socket

    def receive(self) -> Optional[dict]:
        message = None
        buffer_prefix = self.socket.recv(4)  # Let's assume that each message length prefix occupies 4 bytes
        if buffer_prefix:
            buffer_size = struct.unpack('!I', buffer_prefix)[0]
            data_bytes = self.socket.recv(buffer_size)
            message = json.loads(data_bytes.decode('utf-8'))
        return message

    def send(self, message: dict):
        message_bytes = json.dumps(message).encode()
        buffer_size = struct.pack('!I', len(message_bytes))
        self.socket.sendall(buffer_size + message_bytes)

    def raw_send(self, data: bytes):
        self.socket.sendto(data, self.address)

    @property
    def host(self):
        return "{}:{}".format(*self.address)


class MercurySerialApi(object):

    def __init__(self, comport: str, baudrate: int, timeout: float = 1, debug: bool = False):
        level = logging.DEBUG if debug else logging.INFO
        self.comport = comport
        self.baudrate = baudrate
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.log = init_logging("server.serial", level)
        self.serial = serial.Serial(self.comport, self.baudrate, timeout=self.timeout)

    def open(self):
        if self.serial.is_open:
            self.log.debug(f"Serial port {self.comport} is already opened")
            return
        self.serial.open()
        self.log.debug(f"Open serial port {self.comport} with baudrate {self.baudrate}")

    def close(self):
        self.serial.close()
        self.log.debug(f"Close serial port {self.comport}")

    def send(self, data: Union[bytes, List[int]]):
        self.serial.write(data)
        self.log.info(f"[{self.comport}] write: {data}")

    def _encode_int16(self, data):
        if isinstance(data, int):
            return [ord(i) if isinstance(i, str) else i for i in list(struct.pack(">h", data))]

        res = []
        for v in data:
            res.extend(self._encode_int16(v))
        return res

    @classmethod
    def _crc_check(cls, commands: bytes) -> Optional[List[int]]:
        crc = 0xffff
        for command in commands:
            crc ^= command
            for _ in range(8):
                if crc & 1 == 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        if crc > 0x7FFF:
            return list(struct.pack(">H", crc))

    @classmethod
    def _decode_int16(cls, data):
        return struct.unpack(">h", data)[0]

    def read(self, timeout: float = 0.5) -> bytes:
        data_len = -1
        k = 0
        pre = 0
        datas = b""
        stime = time.time()
        while time.time() - stime < timeout:

            if self.serial.is_open and self.serial.in_waiting > 0:
                data = self.serial.read()
                k += 1
                if data_len == 3:
                    datas += data
                    crc = self.serial.read(2)
                    if self._crc_check(datas) == [v for v in crc]:
                        datas += crc
                        break

                if data_len == 1 and data == b"\xfa":
                    datas += data
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
                time.sleep(0.001)
        return datas


class MercurySerialManager(object):

    def __init__(self):
        self.__mian_arm = Arm.RIGHT_ARM
        self.__arm_serial_map: Union[str:Optional[MercurySerialApi]] = {}

    def __setitem__(self, key: str, value: MercurySerialApi):
        self.__arm_serial_map[key] = value

    def __getitem__(self, key: str) -> Optional[MercurySerialApi]:
        return self.__arm_serial_map[key]

    @property
    def arms(self) -> List[str]:
        return list(self.__arm_serial_map.keys())

    def add_arm(self, arm_name: str, serial_api: MercurySerialApi):
        self.__arm_serial_map[arm_name] = serial_api

    def get_arm(self, arm_name: str) -> Optional[MercurySerialApi]:
        return self.__arm_serial_map.get(arm_name, None)

    @property
    def main_arm(self) -> Optional[MercurySerialApi]:
        return self.get_arm(self.__mian_arm)

    @main_arm.setter
    def main_arm(self, arm_name: str):
        assert arm_name in self.__arm_serial_map, f"Arm {arm_name} is not exist"
        self.__mian_arm = arm_name


class MercurySocketInlet(object):

    def __init__(self, host: str, port: int, debug: bool = False):
        level = logging.DEBUG if debug else logging.INFO
        self.host = host
        self.port = port
        self.log = init_logging("server.socket", level)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        self.log.info(f"Start listening on {self.host}:{self.port}")

    def accept(self):
        while True:
            yield self.socket.accept()

    def close(self):
        self.socket.close()
        self.log.info(f"Close socket on {self.host}:{self.port}")


class MercuryCommandServer(object):

    def __init__(self, serial_manager: MercurySerialManager, socket_inlet: MercurySocketInlet, debug: bool = False):
        level = logging.DEBUG if debug else logging.INFO
        self.log = init_logging("server.command", level)
        self.serial_manager = serial_manager
        self.socket_inlet = socket_inlet
        self.connected_client_list: List[Client] = []
        self.respond_client_threads = []
        self.max_thread_pool = 5

    def robot_serial_data_reader(self, arm: str, serial_api: MercurySerialApi):
        serial_api.open()
        while True:
            data = serial_api.read()
            if not data:
                time.sleep(0.001)
                continue

            self.log.info(f"[{serial_api.comport}] read : {list(data)}")
            for client in self.connected_client_list:
                if client.arm != arm:
                    continue
                self.log.info(" * send data to client")
                client.raw_send(data)

    def accept_client_connection(self) -> Generator[Client, None, None]:
        """Accept client connection and return a client object"""
        for client_socket, client_address in self.socket_inlet.accept():
            if len(self.connected_client_list) >= self.max_thread_pool:
                self.log.warning(f"too many clients, reject connection from {client_address}")
                client_socket.close()
                continue

            self.log.info(f" * connection from {client_address}")
            yield Client(arm=Arm.UNKNOWN, address=client_address, socket=client_socket)

    def __on_future_done(self, future: Future):
        client: Client = future.result()
        if client in self.connected_client_list:
            client.socket.close()
            self.log.info(f" * disconnect from {client.address}")
            self.connected_client_list.remove(client)

    def __client_request_handle(self, client: Client) -> Client:
        while True:
            try:
                message = client.receive()
                if message is not None:
                    arm = message.get("arm", Arm.UNKNOWN)
                    commands: List[int] = message.get("command", [])
                    if client.arm == Arm.UNKNOWN and arm != Arm.UNKNOWN:
                        client.arm = arm

                    self.log.info(f" [{client.address}] -> {commands}")
                    self.serial_manager[arm].send(commands)
            except ConnectionResetError:
                break

            except Exception as e:
                self.log.error(f"Error occurred while handling client {client.host}: {e}")
                self.log.error(traceback.format_exc())
        return client   # must be returned

    def enter_mian_loop(self):
        try:
            self.log.info("Start command server")
            for arm_name in self.serial_manager.arms:
                serial_api = self.serial_manager.get_arm(arm_name)
                if serial_api is None:
                    self.log.error(f"Arm {arm_name} not found")
                    continue

                thread = threading.Thread(target=self.robot_serial_data_reader, args=(arm_name, serial_api),
                                          daemon=True)
                self.respond_client_threads.append(thread)

            for thread in self.respond_client_threads:
                thread.start()

            with ThreadPoolExecutor(max_workers=self.max_thread_pool) as executor:
                for client in self.accept_client_connection():
                    self.log.info(f" * accept connection from {client.address}")
                    future: Future = executor.submit(self.__client_request_handle, client)
                    future.add_done_callback(self.__on_future_done)
                    self.connected_client_list.append(client)

            for thread in self.respond_client_threads:
                thread.join()
        except KeyboardInterrupt:
            pass
        finally:
            self.log.info("Stop command server")
            self.socket_inlet.close()


def main(host: str, port: int, debug: bool = False):
    basic_level = logging.DEBUG if debug else logging.INFO
    basic_format = '[%(levelname)8s] | [%(name)14s] %(asctime)s - %(message)s'
    basic_date_format = '%Y-%m-%d %H:%M:%S'
    logging.basicConfig(level=basic_level, format=basic_format, datefmt=basic_date_format)

    serial_manager = MercurySerialManager()

    left_arm = MercurySerialApi("/dev/left_arm", 115200, debug=debug)
    right_arm = MercurySerialApi("/dev/right_arm", 115200, debug=debug)

    serial_manager.add_arm(Arm.LEFT_ARM, left_arm)
    serial_manager.add_arm(Arm.RIGHT_ARM, right_arm)

    socket_inlet = MercurySocketInlet(host=host, port=port, debug=debug)

    mercury_command = MercuryCommandServer(serial_manager=serial_manager, socket_inlet=socket_inlet, debug=debug)
    mercury_command.enter_mian_loop()


if __name__ == '__main__':
    main(host="0.0.0.0", port=9000, debug=True)


