#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from pymycobot import MyArmMControl, MyCobot320, ElephantRobot


class BaseControllerApi:
    def open_suction_pump(self):
        raise NotImplementedError("Not implemented")

    def close_suction_pump(self):
        raise NotImplementedError("Not implemented")

    def set_end_rotate(self, direction: int, speed: float):
        raise NotImplementedError("Not implemented")

    def open_gripper(self, speed: int = 1):
        raise NotImplementedError("Not implemented")

    def close_gripper(self, speed: int = 1):
        raise NotImplementedError("Not implemented")

    def coordinate(self, axis: int, direction: int):
        raise NotImplementedError("Not implemented")

    def go_home(self):
        raise NotImplementedError("Not implemented")

    def stop(self):
        raise NotImplementedError("Not implemented")


class M750Controller(BaseControllerApi):
    def __init__(self, port: str, debug: bool = False):
        self._arm = MyArmMControl(port, baudrate=1000000, debug=debug)

    def go_home(self):
        self._arm.write_angles([0, 0, 0, 0, 90, 0], 10)

    def open_suction_pump(self):
        print(" # (Warning) M750 not support suction pump")

    def close_suction_pump(self):
        print(" # (Warning) M750 not support suction pump")

    def stop(self):
        self._arm.stop()

    def set_end_rotate(self, direction: int, speed: float):
        self._arm.write_angle(6, 180 * direction, speed)

    def open_gripper(self, speed: int = 1):
        self._arm.set_gripper_value(100, speed)

    def close_gripper(self, speed: int = 1):
        self._arm.set_gripper_value(2, speed)

    def coordinate(self, axis: int, direction: int):
        self._arm.jog_coord(axis, direction, 10)


class MyCobot320Controller(BaseControllerApi):
    def __init__(self, port: str, debug: bool = False):
        self._cobot = MyCobot320(port, baudrate=115200, debug=debug)

    def open_suction_pump(self):
        try:
            self._cobot.set_basic_output(1, 0)
        except Exception as e:
            print(f" # (Error) MyCobot320 open suction pump: {e}")

    def close_suction_pump(self):
        try:
            self._cobot.set_basic_output(1, 1)
        except Exception as e:
            print(f" # (Error) MyCobot320 close suction pump: {e}")

    def set_end_rotate(self, direction: int, speed: float):
        self._cobot.send_angle(6, 160 * direction, speed)

    def open_gripper(self, speed: int = 1):
        self._cobot.set_pro_gripper_angle(14, 100)

    def close_gripper(self, speed: int = 1):
        self._cobot.set_pro_gripper_angle(14, 0)

    def coordinate(self, axis: int, direction: int):
        self._cobot.jog_coord(axis, direction, 10)

    def go_home(self):
        self._cobot.send_angles([-90, 0, -90, 0, 90, 0], 10)

    def stop(self):
        self._cobot.stop()


class MyCobot630Controller(BaseControllerApi):
    def __init__(self, host: str, port: int = 5001, debug: bool = False):
        self._cobot = ElephantRobot(host=host, port=port, debug=debug)
        self._cobot.start_client()

    def open_suction_pump(self):
        print(" # (Warning) MyCobot360 not support suction pump")

    def close_suction_pump(self):
        print(" # (Warning) MyCobot360 not support suction pump")

    def set_end_rotate(self, direction: int, speed: float):
        self._cobot.write_angle(4, 180 * direction, 1000)

    def open_gripper(self, speed: int = 1):
        self._cobot.force_gripper_full_open()

    def close_gripper(self, speed: int = 1):
        self._cobot.force_gripper_full_close()

    def coordinate(self, axis: int, direction: int):
        axis_dir_table = ["x", "y", "z"]
        if direction == 0:
            direction = -1
        self._cobot.jog_coord(axis_dir_table[axis - 1], direction, 3000)

    def go_home(self):
        self._cobot.write_angles([0, -97.509, 39.049, -120.34, -95.609, 66.717], 3000)

    def stop(self):
        self._cobot.task_stop()


class UndefinedController(BaseControllerApi):

    def open_suction_pump(self):
        print(" # UndefinedController not support suction pump")

    def close_suction_pump(self):
        print(" # UndefinedController not support suction pump")

    def set_end_rotate(self, direction: int, speed: float):
        print(" # UndefinedController not support set_end_rotate")

    def open_gripper(self, speed: int = 1):
        print(" # UndefinedController not support open_gripper")

    def close_gripper(self, speed: int = 1):
        print(" # UndefinedController not support close_gripper")

    def coordinate(self, axis: int, direction: int):
        print(" # UndefinedController not support coordinate")

    def go_home(self):
        print(" # UndefinedController not support go_home")

    def stop(self):
        print(" # UndefinedController not support stop")
