#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import socket


class Number(object):
    def __init__(self, maximum: float, minimum: float, step: float):
        self.maximum = maximum
        self.minimum = minimum
        self.step = step
        self.value = minimum

    def increase(self, digits: int = 2) -> float:
        self.value = round(self.value + self.step, digits)
        if self.value > self.maximum:
            self.value = self.maximum
        return self.value

    def decrease(self, digits: int = 2) -> float:
        self.value = round(self.value - self.step, digits)
        if self.value < self.minimum:
            self.value = self.minimum
        return self.value

    def reset(self) -> float:
        self.value = self.minimum
        return self.value


def get_local_host() -> str:
    return socket.gethostbyname(socket.gethostname())


def find_esp32_drive():
    paths = []
    for i in os.listdir('/dev'):
        if not i.startswith("ttyACM"):
            continue
        paths.append(os.path.join('/dev', i))

    paths.sort(key=lambda p: int(p[-1]))
    return paths

