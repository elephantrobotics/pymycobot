#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ultraArmP1.py

Python interface for the ultraArmP1 robotic arm.

Author: weijian.wang
Date: 2025-11-25
Description: None
"""
import os
import time

import serial

from pymycobot.common import ProtocolCode
from pymycobot.ultraarm_p1_base import UltraArmP1Base


class UltraArmP1(UltraArmP1Base):
    """Class for controlling the ultraArmP1 robotic arm via serial communication.

    """

    REQUEST_TIMEOUT = 0.3
    ANGLE_COORD_TIMEOUT = 0.02
    QUEUE_TIMEOUT = 0.02
    SET_RESPONSE_TIMEOUT = 5

    def __init__(self, port, baudrate=1000000, timeout=0.05, debug=False, _internal_mode=False):
        """Initialize the ultraArmP1 robot communication.

        Args:
            port (str): Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int, optional): Communication baud rate. Defaults to 1000000.
            timeout (float, optional): Serial read timeout in seconds. Defaults to 0.05.
            debug (bool, optional): Whether to print debug information. Defaults to False.
        """
        super().__init__(debug, _internal_mode)
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.dtr = True
        self._serial_port.open()
        time.sleep(0.5)

    def _serial_in_waiting(self):
        try:
            return int(self._serial_port.in_waiting)
        except Exception:
            try:
                return int(self._serial_port.inWaiting())
            except Exception:
                return 0

    def _read_available_bytes(self):
        n = self._serial_in_waiting()
        if n <= 0:
            return b""
        try:
            return self._serial_port.read(n)
        except Exception:
            return b""

    def _send_command(self, command: str, clear_input=True):
        """Send commands to serial port"""
        if clear_input:
            self._clear_serial_buffer()
        # append checksum
        command = self._append_checksum(command)
        command += ProtocolCode.END
        self._debug_write(command)
        try:
            self._serial_port.write(command.encode())
            self._serial_port.flush()
        except serial.SerialException as e:
            self.log.error(
                f"Serial write failed. "
                f"port={self._serial_port.port}, "
                f"is_open={self._serial_port.is_open}, "
                f"cmd={command}, "
                f"error={e}"
            )
            raise

    def _send_raw_command(self, command: str):
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        time.sleep(0.02)

    def _clear_serial_buffer(self):
        """Clear the serial port buffer before sending commands."""
        try:
            if hasattr(self._serial_port, "reset_input_buffer"):
                self._serial_port.reset_input_buffer()
        except Exception:
            pass

    def _fw_calc_crc(self, payload: bytes):
        """
        CRC = sum(CMD + IDX_H + IDX_L + LEN_H + LEN_L + DATA) & 0xFF
        """
        return sum(payload) & 0xFF

    def _fw_build_packet(self, idx: int, data: bytes):
        """Build data packets"""
        frame = bytearray()
        frame += b'\xA5\x5A'  # Frame header
        frame += b'\x01'  # CMD: PC send data
        frame += idx.to_bytes(2, 'big')  # Packet index
        frame += len(data).to_bytes(2, 'big')
        frame += data

        crc = self._fw_calc_crc(frame[2:])  # exclude header
        frame.append(crc)
        return bytes(frame)

    def _fw_read_ack(self, timeout=1.0):
        """Read screen response data"""
        start = time.time()
        buf = bytearray()

        while time.time() - start < timeout:
            n = self._serial_in_waiting()
            if n > 0:
                buf += self._serial_port.read(n)

                while True:
                    # At least 8 bytes
                    if len(buf) < 8:
                        break

                    # Search for Frame Header
                    if buf[0] != 0xA5 or buf[1] != 0x5A:
                        buf.pop(0)
                        continue

                    frame = bytes(buf[:8])

                    # ✅ CRC Check
                    payload = frame[2:7]  # CMD + IDX + LEN?
                    crc = frame[7]
                    calc_crc = sum(payload) & 0xFF

                    if crc != calc_crc:
                        # ❌ CRC error: Discarding 1 byte and continuing the search.
                        buf.pop(0)
                        continue

                    # ✅ Valid Frame
                    buf[:] = buf[8:]
                    self._debug_read(frame.hex(' ').upper())

                    cmd = frame[2]
                    idx = int.from_bytes(frame[3:5], 'big')

                    # ✅ Legal Range Filtering (Very Important)
                    if cmd not in (2, 3):
                        continue

                    return cmd, idx

            time.sleep(0.002)
        return None

    def _fw_enter_upgrade(self, filename: str):
        """Start downloading"""
        command = ProtocolCode.START_DOWNLOAD_FIRMWARE
        command += f" {filename}"
        self._send_command(command)
        return self._response(_async=True, is_set=True)

    def _download_progress(self, percent):
        print(f"Download progress: {percent}%")
        if self.debug:
            self.log.info(f"Download progress: {percent}%")

    def is_open(self):
        return self._serial_port is not None and self._serial_port.is_open

    def close(self):
        """Close the serial port."""
        with self.lock:
            try:
                if self._serial_port and self._serial_port.is_open:
                    self._serial_port.close()
            except Exception as e:
                self.log.error(f"Failed to close serial port: {e}")

    def open(self):
        """Open the serial port."""
        with self.lock:
            try:
                self._serial_port.open()
            except Exception as e:
                self.log.error(f"Failed to open serial port: {e}")

    def finish_firmware_upgrade(self):
        """Download complete"""
        command = ProtocolCode.FINISH_DOWNLOAD_FIRMWARE
        self._send_command(command)
        res = self._response(_async=True, is_set=True)
        if res == "ok":
            self.log.debug("Waiting 3 seconds for controller restart...")
            time.sleep(3)
        return res

    def download_firmware_sd(self, filename, show_progress=True):
        """
        Download firmware to the SD card via M450/M451 commands.

        Args:
            filename (str): name of the firmware file, and must be a .bin file
            show_progress (bool): whether to show download progress
        """
        self.calibration_parameters(class_name=self.__class__.__name__, download_filename=filename)

        local_path = filename  # For local use

        fw_name = os.path.basename(filename)  # For protocol use (M450)

        if show_progress:
            # callback(percent:int) to report progress
            progress_cb = self._download_progress
        else:
            progress_cb = None
        with self.lock:
            self._clear_serial_buffer()
            self.finish_firmware_upgrade()

            # Entering upgrade mode.
            res = self._fw_enter_upgrade(fw_name)
            time.sleep(0.2)
            if res != 'ok':
                return res

            # read bin
            with open(local_path, "rb") as f:
                bin_data = f.read()

            chunk_size = 512
            total_packets = (len(bin_data) + chunk_size - 1) // chunk_size

            idx = 1
            while idx <= total_packets:
                offset = (idx - 1) * chunk_size
                data = bin_data[offset: offset + chunk_size]

                pkt = self._fw_build_packet(idx, data)
                self._debug_write(pkt.hex(' ').upper())
                self._serial_port.write(pkt)
                self._serial_port.flush()

                ack = self._fw_read_ack(timeout=1.0)
                if ack is None:
                    continue  # timeout -> resend
                cmd, next_idx = ack

                if cmd == 2:  # success
                    if next_idx < 1 or next_idx > total_packets + 1:
                        continue
                    idx = next_idx
                    if progress_cb:
                        progress_cb(int((idx - 1) * 100 / total_packets))

                elif cmd == 3:  # resend
                    idx = next_idx
                else:
                    self.finish_firmware_upgrade()
                    msg = f"Unknown ACK CMD: {cmd}"
                    self.log.error(msg)
                    raise RuntimeError(msg)
            # Finish
            return self.finish_firmware_upgrade()

    def upgrade_restart(self):
        """Upgrade and restart"""
        with self.lock:
            self._send_command(ProtocolCode.UPGRADE_RESTART)
            return self._response(_async=True, is_set=True, timeout=15)
