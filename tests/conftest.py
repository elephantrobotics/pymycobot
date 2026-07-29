import sys
import types


bleak_stub = types.ModuleType("bleak")
bleak_stub.BleakClient = object
sys.modules.setdefault("bleak", bleak_stub)
