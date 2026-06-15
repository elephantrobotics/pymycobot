"""
ultraarm_p1_bluetooth.py
This module controls the robotic arm movements.

Author: Wang Weijian
Date: 2026-03-12
"""
import asyncio
import logging
import threading
import time

from bleak import BleakClient

from pymycobot.common import ProtocolCode
from pymycobot.ultraarm_p1_base import UltraArmP1Base


class UltraArmP1Bluetooth(UltraArmP1Base):
    """Bluetooth communication interface for ultraArmP1."""

    REQUEST_TIMEOUT = 1
    ANGLE_COORD_TIMEOUT = 0.1
    QUEUE_TIMEOUT = 0.15
    SET_RESPONSE_TIMEOUT = 7

    def __init__(self, address, timeout=0.05, debug=False, _internal_mode=False):

        super().__init__(debug, _internal_mode)
        self.address = address
        self.timeout = timeout

        logging.getLogger("bleak").setLevel(logging.WARNING)
        logging.getLogger("asyncio").setLevel(logging.WARNING)

        self.recv_lock = threading.Lock()

        self.recv_buffer = bytearray()

        # BLE event loop thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self._connect_ble()
        time.sleep(0.5)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _connect_ble(self):
        future = asyncio.run_coroutine_threadsafe(
            self._connect_async(), self.loop)
        future.result()

    async def _connect_async(self):
        if self.debug:
            self.log.info("BLE Connecting")

        self.client = BleakClient(self.address)

        await self.client.connect()

        if self.debug:
            self.log.info("BLE connected")

        # Automatic search characteristic
        for service in self.client.services:
            for char in service.characteristics:
                if "write" in char.properties and "notify" in char.properties:
                    self.handle = char.handle
                    self.char_uuid = char.uuid

        await self.client.start_notify(self.handle, self._notification_handler)

    def _notification_handler(self, sender, data):

        with self.recv_lock:
            self.recv_buffer += data

    def _read_available_bytes(self):

        if len(self.recv_buffer) == 0:
            return b""

        data = bytes(self.recv_buffer)
        self.recv_buffer.clear()
        return data

    def _clear_recv_buffer(self):
        with self.lock:
            self.recv_buffer.clear()

    def _send_command(self, command: str):
        """Send commands to bluetooth server"""
        self.recv_buffer.clear()
        command = self._append_checksum(command)
        command += ProtocolCode.END
        self._debug_write(command)
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(self.handle,command.encode(), response=False), self.loop)
            future.result()
        except Exception as e:
            self.log.exception(f"bluetooth send error: {e}")
            raise

    def _send_raw_command(self, command: str):
        self.recv_buffer.clear()
        future = asyncio.run_coroutine_threadsafe(
            self.client.write_gatt_char(self.handle, command.encode(), response=False), self.loop)
        future.result()
        time.sleep(0.02)

    def open(self):
        """Open BLE connection."""
        with self.lock:
            try:
                if self.client and self.client.is_connected:
                    return
                future = asyncio.run_coroutine_threadsafe(self._connect_async(), self.loop)
                future.result()
            except Exception as e:
                self.log.error(f"bluetooth open error: {e}")

    def close(self):
        """Close BLE connection."""
        with self.lock:
            try:
                if self.client and self.client.is_connected:
                    future = asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop)
                    future.result()

            except Exception as e:
                self.log.error(f"bluetooth close error: {e}")
