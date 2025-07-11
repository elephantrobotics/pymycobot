#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import dataclasses
import typing as T
import pymycobot


@dataclasses.dataclass
class AutoReportMessage:
    rx: float
    ry: float
    rw: float
    emergency_stop: bool
    is_power_on: bool
    error_motor: list
    left_bumper_strip: bool
    right_bumper_strip: bool
    power_voltage: float
    motor_enable: bool


class AGVDriveAPI(object):
    __version__ = pymycobot.__version__

    def __init__(self, comport: str, debug: bool = False):
        self._agv_pro = pymycobot.MyAGVPro(comport, debug=debug)

    def set_strip_light_color(self, locations: T.List[int], color: T.Tuple[int, int, int], brightness: int = 255):
        for location in locations:
            print(f" # Set strip light color {location} to {color} with brightness {brightness}")
            self._agv_pro.set_led_color(location, color=color, brightness=brightness)

    def open_strip_light_diy_mode(self):
        self._agv_pro.set_led_mode(1)

    def close_strip_light_diy_mode(self):
        self._agv_pro.set_led_mode(0)

    def get_firmware_version(self):
        system_version = self._agv_pro.get_system_version()
        modify_version = self._agv_pro.get_modify_version()

        if system_version is None or modify_version is None:
            return None

        return f"{system_version}.{modify_version}"

    def get_communication_state(self):
        return self._agv_pro.get_communication_state()

    def get_motor_enable_status(self):
        return self._agv_pro.get_motor_enable_status()

    def get_motor_temps(self):
        return self._agv_pro.get_motor_temps()

    def get_motor_torques(self):
        return self._agv_pro.get_motor_torques()

    def get_motor_speeds(self):
        return self._agv_pro.get_motor_speeds()

    def get_auto_report_state(self) -> bool:
        return self._agv_pro.get_auto_report_state() == 1

    def get_auto_report_message(self) -> T.Optional[AutoReportMessage]:
        report_message = self._agv_pro.get_auto_report_message()
        if report_message is None:
            return None

        if report_message[3] == 0:
            report_message[3] = [0, 0, 0, 0]

        print(f" # Get auto report message {report_message}")
        return AutoReportMessage(
            rx=report_message[0],
            ry=report_message[1],
            rw=report_message[2],
            emergency_stop=report_message[3][0] == 1,
            is_power_on=report_message[3][1] == 1,
            left_bumper_strip=report_message[3][2] == 1,
            right_bumper_strip=report_message[3][3] == 1,
            error_motor=report_message[4],
            power_voltage=report_message[5],
            motor_enable=report_message[6] == 1
        )

    def get_emergency_stop_state(self) -> bool:
        return self._agv_pro.get_estop_state() == 1

    def set_auto_report_state(self, state: bool):
        return self._agv_pro.set_auto_report_state(int(state))

    def forward(self, speed: float):
        self._agv_pro.move_forward(speed)

    def backward(self, speed: float):
        self._agv_pro.move_backward(speed)

    def pan_left(self, speed: float):
        self._agv_pro.move_left_lateral(speed)

    def pan_right(self, speed: float):
        self._agv_pro.move_right_lateral(speed)

    def rotate_left(self, speed: float):
        self._agv_pro.turn_left(speed)

    def rotate_right(self, speed: float):
        self._agv_pro.turn_right(speed)

    def power_on(self) -> bool:
        return self._agv_pro.power_on() == 1

    def is_power_on(self) -> bool:
        is_power_on = self._agv_pro.is_power_on()
        return is_power_on == 1

    def power_off(self) -> bool:
        return self._agv_pro.power_off() == 1

    def stop(self):
        self._agv_pro.stop()

    def close(self):
        self._agv_pro.close()

    def open(self):
        self._agv_pro.open()
