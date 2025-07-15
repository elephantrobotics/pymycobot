#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import time
from core import Number
from core.joystick import InputJoystick, Hotkey
from core.drive import AGVDriveAPI
from core.controller import BaseControllerApi


class AGVProCompositeKit(object):
    agvHorizontalSpeed: Number = Number(0.5, 0.1, 0.1)
    agvVerticalSpeed: Number = Number(0.75, 0.15, 0.15)
    agvRotationSpeed: Number = Number(1.5, 0.2, 0.1)

    joystick = InputJoystick(raw_mapping=False)

    def __init__(self, agv_pro_driver: AGVDriveAPI, arm_controller: BaseControllerApi):
        self.arm_controller = arm_controller
        self.agv_pro = agv_pro_driver

        if self.agv_pro.is_power_on() is False:
            self.agv_pro.power_on()
            print(f" # Start the AGVPRO power supply.")
        else:
            print(f" # AGVPRO is already powered on.")

        self.joystick.inject_caller(self)

        self._agv_motion_control = False
        self._arm_motion_control = False

    def init_joystick(self) -> bool:
        device = InputJoystick.get_gamepad(0)
        if device is not None:
            self.joystick.set_device(device)
        return device is not None

    @joystick.register(hotkey=Hotkey.L1)
    def L1(self, value: int):
        if value == 1 and not self._agv_motion_control:
            self.agv_pro.rotate_right(self.agvRotationSpeed.value)
            self._agv_motion_control = True
            print(f" # Rotate right.")
            return

        if value == 0 and self._agv_motion_control:
            self.agv_pro.stop()
            self._agv_motion_control = False
            print(f" # Stop rotating.")
            return

    @joystick.register(hotkey=Hotkey.R1)
    def R1(self, value: int):
        if value == 1 and not self._agv_motion_control:
            self.agv_pro.rotate_left(self.agvRotationSpeed.value)
            self._agv_motion_control = True
            print(f" # Rotate left.")
            return

        if value == 0 and self._agv_motion_control:
            self._agv_motion_control = False
            self.agv_pro.stop()
            print(f" # Stop rotating.")
            return

    @joystick.register(hotkey=Hotkey.L2, value_filter=lambda value: value == 0)
    def L2(self, _: int):
        hs = self.agvHorizontalSpeed.decrease()
        vs = self.agvVerticalSpeed.decrease()
        rs = self.agvRotationSpeed.decrease()
        print(f" # Speed decreases {hs = :.2f} {vs = :.2f} {rs = :.2f}")

    @joystick.register(hotkey=Hotkey.R2, value_filter=lambda value: value == 0)
    def R2(self, _: int):
        hs = self.agvHorizontalSpeed.increase()
        vs = self.agvVerticalSpeed.increase()
        rs = self.agvRotationSpeed.increase()
        print(f" # Speed increases {hs = :.2f} {vs = :.2f} {rs = :.2f}")

    @joystick.register(hotkey=Hotkey.RIGHT_Y_AXIS)
    def RIGHT_X_AXIS(self, value: int):
        if value < 128 and not self._agv_motion_control:
            self.agv_pro.forward(self.agvVerticalSpeed.value)
            self._agv_motion_control = True
            print(f" # Move forward.")

        elif value > 128 and not self._agv_motion_control:
            self.agv_pro.backward(self.agvVerticalSpeed.value)
            self._agv_motion_control = True
            print(f" # Move backward.")

        elif value == 128 and self._agv_motion_control:
            self.agv_pro.stop()
            self._agv_motion_control = False
            print(f" # Stop moving.")

    @joystick.register(hotkey=Hotkey.RIGHT_X_AXIS)
    def RIGHT_Y_AXIS(self, value: int):
        if value < 128 and not self._agv_motion_control:
            self._agv_motion_control = True
            self.agv_pro.pan_left(self.agvHorizontalSpeed.value)
            print(f" # Pan left.")

        elif value > 128 and not self._agv_motion_control:
            self._agv_motion_control = True
            self.agv_pro.pan_right(self.agvHorizontalSpeed.value)
            print(f" # Pan right.")

        elif value == 128 and self._agv_motion_control:
            self._agv_motion_control = False
            print(f" # Stop panning.")
            self.agv_pro.stop()

    @joystick.register(hotkey=Hotkey.STARTUP, value_filter=lambda value: value == 0)
    def STARTUP(self, _):
        hs = self.agvHorizontalSpeed.reset()
        vs = self.agvVerticalSpeed.reset()
        rs = self.agvRotationSpeed.reset()

        print(f" # Speed reset {hs = :.2f} {vs = :.2f} {rs = :.2f}")

        if self.agv_pro.is_power_on() is False:
            print(f" # Abnormal power-off of AGVPRO has been detected, and it is trying to power on...")
            self.agv_pro.power_on()
            time.sleep(1)
            self.agv_pro.open_strip_light_diy_mode()
            self.agv_pro.set_strip_light_color([0, 1], (0, 255, 0), 255)
        else:
            self.agv_pro.stop()
            print(f" # Stop moving.")

        self.arm_controller.go_home()
        self.arm_controller.close_suction_pump()

    @joystick.register(hotkey=Hotkey.Y)
    def Y(self, value: int):
        print(f"{Hotkey.Y} {value}")
        if value == 1:
            print(f" # Open gripper.")
            self.arm_controller.open_gripper(1)
        else:
            self.arm_controller.stop()
            print(f" # Stop opening the gripper.")

    @joystick.register(hotkey=Hotkey.A)
    def A(self, value: int):
        if value == 1:
            print(f" # Close gripper.")
            self.arm_controller.close_gripper(1)
        else:
            self.arm_controller.stop()
            print(f" # Stop closing the gripper.")

    @joystick.register(hotkey=Hotkey.X, value_filter=lambda value: value == 0)
    def X(self, _: int):
        try:
            self.arm_controller.open_suction_pump()
            print(f"Open suction pump.")
        except NotImplementedError as e:
            print(e)

    @joystick.register(hotkey=Hotkey.B, value_filter=lambda value: value == 0)
    def B(self, _: int):
        try:
            self.arm_controller.close_suction_pump()
            print(f"Close suction pump.")
        except NotImplementedError as e:
            print(e)

    @joystick.register(hotkey=Hotkey.HORIZONTAL)
    def HORIZONTAL(self, value: int):
        if value == 1:
            print(f" # The end is rotated counterclockwise.")
            self.arm_controller.set_end_rotate(direction=-1, speed=1)

        elif value == -1:
            print(f" # The end is rotate clockwise.")
            self.arm_controller.set_end_rotate(direction=1, speed=1)

        else:
            print(" # Stop spinning.")
            self.arm_controller.stop()

    @joystick.register(hotkey=Hotkey.VERTICAL)
    def VERTICAL(self, value: int):
        if value == -1:
            print(f"axis z +.")
            self.arm_controller.coordinate(axis=3, direction=1)
        elif value == 1:
            print(f"axis Z -.")
            self.arm_controller.coordinate(axis=3, direction=0)
        else:
            print("axis z stop.")
            self.arm_controller.stop()

    @joystick.register(hotkey=Hotkey.LEFT_X_AXIS)
    def LEFT_X_AXIS(self, value: int):
        if value > 128 and not self._arm_motion_control:
            print(f"axis y -.")  # Y+
            self.arm_controller.coordinate(axis=2, direction=0)
            self._arm_motion_control = True

        elif value < 128 and not self._arm_motion_control:
            print(f"axis y +.")  # Y-
            self.arm_controller.coordinate(axis=2, direction=1)
            self._arm_motion_control = True

        elif value == 128 and self._arm_motion_control:
            self._arm_motion_control = False
            print(f"axis y stop.")
            self.arm_controller.stop()

    @joystick.register(hotkey=Hotkey.LEFT_Y_AXIS)
    def LEFT_Y_AXIS(self, value: int):
        if value < 128 and not self._arm_motion_control:
            print(f"axis x +.")  # X+
            self.arm_controller.coordinate(axis=1, direction=1)
            self._arm_motion_control = True

        elif value > 128 and not self._arm_motion_control:
            print(f"axis x -.")  # X-
            self.arm_controller.coordinate(axis=1, direction=0)
            self._arm_motion_control = True

        elif value == 128 and self._arm_motion_control:
            self._arm_motion_control = False
            print(f"axis x stop.")
            self.arm_controller.stop()

    def mainloop(self):
        self.joystick.run_with_loop()




