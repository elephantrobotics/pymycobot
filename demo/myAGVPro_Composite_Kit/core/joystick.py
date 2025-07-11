#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import logging
import queue
import threading
import time
import enum
import typing as T
from inputs import GamePad, DeviceManager


class Hotkey(enum.Flag):
    # hat
    HORIZONTAL = enum.auto()
    VERTICAL = enum.auto()
    CENTER = enum.auto()

    # button
    A = enum.auto()
    B = enum.auto()
    X = enum.auto()
    Y = enum.auto()
    L1 = enum.auto()
    R1 = enum.auto()
    SELECT = enum.auto()
    STARTUP = enum.auto()
    LEFT_STICK = enum.auto()
    RIGHT_STICK = enum.auto()
    MODEL = enum.auto()

    # axis
    LEFT_X_AXIS = enum.auto()
    LEFT_Y_AXIS = enum.auto()
    RIGHT_X_AXIS = enum.auto()
    RIGHT_Y_AXIS = enum.auto()
    L2 = enum.auto()
    R2 = enum.auto()


BindingCallbackT = T.Callable[[int], None]
HotkeyT = T.Union[Hotkey, int]


class InputJoystick(object):
    """InputJoystick"""
    hotkey_map = {
        "Key::BTN_TL": Hotkey.L1,
        "Absolute::ABS_BRAKE": Hotkey.L2,
        "Key::BTN_TR": Hotkey.R1,
        "Absolute::ABS_GAS": Hotkey.R2,
        "Absolute::ABS_HAT0Y": Hotkey.VERTICAL,
        "Absolute::ABS_HAT0X": Hotkey.HORIZONTAL,
        "Key::BTN_SELECT": Hotkey.SELECT,
        "Key::BTN_START": Hotkey.STARTUP,
        "Key::BTN_NORTH": Hotkey.X,
        "Key::BTN_SOUTH": Hotkey.A,
        "Key::BTN_WEST": Hotkey.Y,
        "Key::BTN_EAST": Hotkey.B,
        "Key::BTN_THUMBL": Hotkey.LEFT_STICK,
        "Key::BTN_THUMBR": Hotkey.RIGHT_STICK,
        "Key::BTN_MODE": Hotkey.MODEL,
        "Absolute::ABS_X": Hotkey.LEFT_X_AXIS,
        "Absolute::ABS_Y": Hotkey.LEFT_Y_AXIS,
        "Absolute::ABS_Z": Hotkey.RIGHT_X_AXIS,
        "Absolute::ABS_RZ": Hotkey.RIGHT_Y_AXIS
    }

    def __init__(self, device: T.Optional[GamePad] = None, ignore_types: T.List[str] = None, raw_mapping: bool = True):
        self._device = device
        self._raw_mapping = raw_mapping
        self._ignore_types = ignore_types or ["Sync", "Misc"]
        self._bindings: T.Dict[HotkeyT, BindingCallbackT] = {}
        self._value_filter_table: T.Dict[HotkeyT, T.Callable[[int], bool]] = {}
        self._logger = logging.getLogger("joystick")
        self._caller: T.Optional[object] = None

    def __repr__(self) -> str:
        return f"InputJoystick({self._device.name})"

    @property
    def name(self):
        return self._device.name

    @property
    def device(self) -> GamePad:
        return self._device

    def set_device(self, device: GamePad) -> None:
        self._device = device
        self._logger.debug(f"Set device {device.name}")

    def binding(self, hotkey: HotkeyT, callback: BindingCallbackT) -> None:
        self._logger.debug(f"Bind {hotkey} -> {callback.__name__}")
        self._bindings[hotkey] = callback

    def register(self, hotkey: HotkeyT, value_filter: T.Callable[[int], bool] = None):
        def wrapper(callback: BindingCallbackT):
            self.binding(hotkey, callback)

            if value_filter is None:
                return

            self._value_filter_table[hotkey] = value_filter

        return wrapper

    # 添加调用者
    def inject_caller(self, caller: object) -> None:
        self._logger.debug(f"Inject caller {caller.__class__.__name__}")
        self._caller = caller

    def read(self) -> T.Iterator[T.Tuple[HotkeyT, int]]:
        for event in self._device.read():
            if event.ev_type in self._ignore_types:
                continue

            hotkey = f"{event.ev_type}::{event.code}"
            if self._raw_mapping is False:
                hotkey = self.hotkey_map.get(hotkey, hotkey)
            yield hotkey, event.state

    def _trigger_hotkey_callback(self, hotkey: HotkeyT, value: int) -> None:
        self._logger.debug(f"{hotkey} -> {value}")
        if hotkey not in self._bindings.keys():
            self._logger.debug(f"Unbind {hotkey} -> {value}")
            return

        if hotkey in self._value_filter_table.keys() and not self._value_filter_table[hotkey](value):
            self._logger.debug(f"Filter {hotkey} -> {value}")
            return

        self._logger.debug(f"Bind {hotkey} -> {value}")

        if self._caller is not None:
            self._bindings[hotkey](self._caller, value)
        else:
            self._bindings[hotkey](value)

    def monitoring(self):
        for hotkey, value in self.read():
            self._trigger_hotkey_callback(hotkey, value)

    def run_with_loop(self, delay: float = 0.01) -> None:
        try:
            while True:
                self.monitoring()
                time.sleep(delay)
        except KeyboardInterrupt:
            return

    def _thread_loop(self, hotkey_queue: queue.Queue) -> None:
        while True:
            try:
                hotkey, value = hotkey_queue.get()
                if all([hotkey, value]) is False:
                    break
                self._trigger_hotkey_callback(hotkey, value)
            except queue.Empty:
                continue

            except KeyboardInterrupt:
                print(" # exit")
                break
            except Exception as e:
                self._logger.error(e)

    def run_with_thread(self, loop_caller: T.Callable[[], bool] = None, delay: float = 0.01) -> None:
        hotkey_queue = queue.Queue()
        thread = threading.Thread(target=self._thread_loop, args=(hotkey_queue,))
        thread.start()
        while loop_caller() if loop_caller is not None else True:
            for hotkey, value in self.read():
                hotkey_queue.put((hotkey, value))
            time.sleep(delay)
        hotkey_queue.put((None, None))
        thread.join()

    @staticmethod
    def get_gamepad(index: int = 0, _filter: T.Callable[[GamePad], bool] = None) -> T.Optional[GamePad]:
        device_manager = DeviceManager()
        gamepads = device_manager.gamepads
        if _filter is not None:
            gamepads = list(filter(_filter, gamepads))

        if index >= len(gamepads):
            return None
        return gamepads[index]

    @classmethod
    def create_by_index(cls, index: int, ignore_types: T.List[str] = None, raw_mapping: bool = True) -> "InputJoystick":
        ignore_types = ignore_types or []
        joystick = cls.get_gamepad(index=index)
        if joystick is None:
            raise FileNotFoundError("No joystick found")
        return cls(joystick, ignore_types=ignore_types, raw_mapping=raw_mapping)

    @classmethod
    def create_by_name(cls, name: str, ignore_types: T.List[str] = None, raw_mapping: bool = True) -> "InputJoystick":
        joystick = cls.get_gamepad(_filter=lambda j: j.name == name)
        if joystick is not None:
            return cls(joystick, ignore_types=ignore_types, raw_mapping=raw_mapping)
        raise FileNotFoundError(f"No joystick named {name}")
