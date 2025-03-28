#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import typing as T
from serial.tools import list_ports
_T = T.TypeVar("_T")


def select(
        message: str,
        options: T.List[_T],
        default: int = None,
        level: int = 1,
        echo: T.Callable = lambda msg: msg,
        start: int = 1
) -> T.Optional[_T]:
    print(f"{message}\r\n")
    p = "    " * level
    for ordinal, option in enumerate(options, start=start):
        print(f"{p}{ordinal}. {echo(option)}")
    else:
        print()

    tips = f" # (Info) Select ({echo(options[default - 1])}) >" if default is not None else f" # (Info) Select: "

    while True:
        try:
            user_input = input(f"{tips}").lower().strip()
            if len(user_input) == 0 and (default is not None or len(options) == 1):
                return options[default - 1]

            if user_input.isdigit():
                return options[int(user_input) - 1]

            if user_input in options:
                return user_input

            if user_input in ("q", "quit"):
                return None

            raise ValueError
        except ValueError:
            print(f"  * Invalid input, please enter a number.")

        except IndexError:
            print(f"  * Invalid input, please enter a valid number.")


def get_local_serial_port():  # 获取所有串口号
    return [comport.device for comport in list_ports.comports()]


def get_local_host():
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    return ip_address
