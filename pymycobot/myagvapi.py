#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import serial
import struct
import logging
import logging.handlers


def setup_logging(name: str = __name__, debug: bool = False) -> logging.Logger:
    debug_formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)06d %(levelname).4s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger(name)
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(debug_formatter)
    if debug is True:
        logger.addHandler(stream_handler)
        logger.setLevel(logging.INFO)  # 100M日志
        file_handler = logging.handlers.RotatingFileHandler(
            filename="python_debug.log", maxBytes=100 * 1024 * 1024, backupCount=1
        )
        file_handler.setFormatter(debug_formatter)
        logger.addHandler(file_handler)

    else:
        logger.setLevel(logging.DEBUG)

    return logger


def setup_serial_connect(port, baudrate, timeout=None):
    serial_api = serial.Serial()
    serial_api.port = port
    serial_api.baudrate = baudrate
    serial_api.timeout = timeout
    serial_api.rts = False
    serial_api.dtr = False
    serial_api.open()
    return serial_api


class CommunicationProtocol(object):
    def write(self, command):
        raise NotImplementedError

    def read(self, size=1):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    def open(self):
        raise NotImplementedError

    def is_open(self):
        raise NotImplementedError

    def clear(self):
        raise NotImplementedError


class Utils:
    @classmethod
    def process_data_command(cls, args):
        if not args:
            return []

        processed_args = []
        for index in range(len(args)):
            if isinstance(args[index], list):
                data = cls.encode_int16(args[index])
                processed_args.extend(data)
            else:
                processed_args.append(args[index])

        return cls.flatten(processed_args)

    @classmethod
    def flatten(cls, datas):
        flat_list = []
        for item in datas:
            if not isinstance(item, list):
                flat_list.append(item)
            else:
                flat_list.extend(cls.flatten(item))
        return flat_list

    @classmethod
    def float(cls, number, decimal=2):
        return round(number / 10 ** decimal, 2)

    @classmethod
    def encode_int16(cls, data):
        if isinstance(data, int):
            return [
                ord(i) if isinstance(i, str) else i
                for i in list(struct.pack(">h", data))
            ]
        else:
            res = []
            for v in data:
                t = cls.encode_int16(v)
                res.extend(t)
            return res

    @classmethod
    def decode_int16(cls, data):
        return struct.unpack(">h", data)[0]

    @classmethod
    def crc16_check(cls, command):
        crc = 0xffff
        for index in range(len(command)):
            crc ^= command[index]
            for _ in range(8):
                if crc & 1 == 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        if crc > 0x7FFF:
            crc_res = list(struct.pack(">H", crc))
        else:
            crc_res = cls.encode_int16(crc)

        for i in range(2):
            if isinstance(crc_res[i], str):
                crc_res[i] = ord(crc_res[i])
        return crc_res

    @classmethod
    def crc16_check_bytes(cls, command):
        data = cls.crc16_check(command)
        return bytes(bytearray(data))

    @classmethod
    def get_bits(cls, data):
        reverse_bins = reversed(bin(data)[2:])
        rank = [i for i, e in enumerate(reverse_bins) if e != '0']
        return rank
