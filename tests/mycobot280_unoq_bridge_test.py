import sys
import types

import pytest

import pymycobot.mycobot280 as mycobot280_module
from pymycobot.mycobot280 import MyCobot280


class FakeBridge:
    responses = {}
    calls = []

    @classmethod
    def reset(cls, responses):
        cls.responses = responses
        cls.calls = []

    @classmethod
    def call(cls, name, frame_hex, timeout_ms=None, baudrate=None):
        cls.calls.append((name, frame_hex, timeout_ms, baudrate))
        return cls.responses.get(frame_hex)


def install_fake_bridge(monkeypatch, responses):
    arduino = types.ModuleType("arduino")
    app_utils = types.ModuleType("arduino.app_utils")
    app_utils.Bridge = FakeBridge
    arduino.app_utils = app_utils
    FakeBridge.reset(responses)
    monkeypatch.setitem(sys.modules, "arduino", arduino)
    monkeypatch.setitem(sys.modules, "arduino.app_utils", app_utils)


def make_bridge_robot(monkeypatch, responses):
    install_fake_bridge(monkeypatch, responses)
    return MyCobot280(timeout=0.5, unoq_bridge=True)


def test_unoq_bridge_get_angles(monkeypatch):
    mc = make_bridge_robot(
        monkeypatch,
        {
            "FEFE0220FA": (
                "FEFE0E20"
                "0000"
                "2328"
                "DCD8"
                "3039"
                "CFC7"
                "0001"
                "FA"
            )
        },
    )

    assert mc.get_angles() == [0.0, 90.0, -90.0, 123.45, -123.45, 0.01]
    assert FakeBridge.calls == [("XferBridgeMsg", "FEFE0220FA", 500, 1000000)]


def test_unoq_bridge_ignores_port_and_uses_timing_response(monkeypatch):
    mc = make_bridge_robot(
        monkeypatch,
        {
            "FEFE062101000020FA": "FEFE032101FA|1000|20|30",
        },
    )

    assert mc.send_angle(1, 0, 32) == 1
    assert FakeBridge.calls == [
        ("XferBridgeMsg", "FEFE062101000020FA", 500, 1000000)
    ]


def test_unoq_bridge_accepts_legacy_port_and_baudrate(monkeypatch):
    install_fake_bridge(monkeypatch, {"FEFE0220FA": "FEFE0E20000000000000000000000000FA"})

    mc = MyCobot280("/dev/mycobot", 1000000, timeout=0.1, unoq_bridge=True)

    assert mc.get_angles() == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert FakeBridge.calls == [("XferBridgeMsg", "FEFE0220FA", 100, 1000000)]


def test_unoq_bridge_send_angles_ack(monkeypatch):
    mc = make_bridge_robot(
        monkeypatch,
        {
            "FEFE0F2200000000000000000000000020FA": "FEFE032201FA",
        },
    )

    assert mc.send_angles([0, 0, 0, 0, 0, 0], 32) == 1


@pytest.mark.parametrize("response", ["FEFE035B01FA", "FEFE035B02FA"])
def test_unoq_bridge_error_frames_return_minus_one(monkeypatch, response):
    mc = make_bridge_robot(monkeypatch, {"FEFE0220FA": response})

    assert mc.get_angles() == -1


def test_unoq_bridge_method_unavailable_returns_minus_one(monkeypatch):
    class UnavailableBridge:
        @classmethod
        def call(cls, name, frame_hex, timeout_ms=None, baudrate=None):
            raise ValueError(
                "Request 'XferBridgeMsg' failed: "
                "method XferBridgeMsg not available (2)"
            )

    arduino = types.ModuleType("arduino")
    app_utils = types.ModuleType("arduino.app_utils")
    app_utils.Bridge = UnavailableBridge
    arduino.app_utils = app_utils
    monkeypatch.setitem(sys.modules, "arduino", arduino)
    monkeypatch.setitem(sys.modules, "arduino.app_utils", app_utils)

    mc = MyCobot280(unoq_bridge=True)

    assert mc.get_angles() == -1


def test_unoq_bridge_requires_arduino_bridge(monkeypatch):
    monkeypatch.delitem(sys.modules, "arduino", raising=False)
    monkeypatch.delitem(sys.modules, "arduino.app_utils", raising=False)

    with pytest.raises(RuntimeError, match="unoq_bridge=True requires"):
        MyCobot280(unoq_bridge=True)


def test_default_serial_mode_still_opens_serial(monkeypatch):
    import serial

    instances = []

    class FakeSerial:
        def __init__(self):
            self.port = None
            self.baudrate = None
            self.timeout = None
            self.rts = None
            self.open_called = False

        def open(self):
            self.open_called = True

    def serial_factory():
        instance = FakeSerial()
        instances.append(instance)
        return instance

    monkeypatch.setattr(serial, "Serial", serial_factory)
    monkeypatch.setattr(mycobot280_module.time, "sleep", lambda _seconds: None)

    mc = MyCobot280("/dev/ttyUSB0", 1000000, timeout=0.25)

    assert mc.unoq_bridge is False
    assert instances[0].port == "/dev/ttyUSB0"
    assert instances[0].baudrate == 1000000
    assert instances[0].timeout == 0.25
    assert instances[0].rts is False
    assert instances[0].open_called is True
