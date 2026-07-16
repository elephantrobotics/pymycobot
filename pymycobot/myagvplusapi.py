# coding=utf-8
import atexit
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import struct
import logging
import serial
import json

from pymycobot.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type
from pymycobot.log import setup_logging
from pymycobot.error import calibration_parameters
import functools
from pymycobot.common import MyagvPlusCommand

# MyAGV Plus mechanical constants
WHEEL_RADIUS = 0.04       # Wheel radius in meters
WHEEL_BASE_SUM = 0.221    # a + b (half-length + half-width) in meters


def motor_api(func):
    """Decorator to unify comm state check and return values (-1 / 1).
    Checks _power_on_state BEFORE executing the method body,
    Checks _power_on_state AFTER executing the method body,
    allowing _write debug logs to be generated.
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self._check_comm_state():
            return -1

        if getattr(self, 'motor_driver', None) is None:
            if hasattr(self, 'log'):
                self.log.error("Execution blocked: Motor driver is not initialized.")
            return -1

        if not getattr(self, '_power_on_state', False):
            msg = "Error: Currently in power-off state."
            if hasattr(self, 'log'):
                self.log.error(msg)
            if hasattr(self, 'last_serial_logs') and "error" in self.last_serial_logs:
                self.last_serial_logs["error"].append(msg)
            return -1

        res = func(self, *args, **kwargs)

        if res is None:
            res = 1
        return res
    return wrapper


def _crc16_modbus(data):
    """Calculate Modbus CRC-16

    Args:
        data (bytes): Input byte array

    Returns:
        bytes: 2-byte CRC array (high byte first)
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([(crc >> 8) & 0xFF, crc & 0xFF])


class MyAGVPlusApi(object):
    """Base class for MyAGV Plus communication protocol handling.

    This class handles the low-level serial communication with ESP32 and DM motors,
    providing the communication infrastructure for the MyAGVPlus API class.
    """

    _instance_created = False

    def __init__(self, motor_port='/dev/myagvplus_controller', baudrate=921600,
                 esp32_port='/dev/ttyUSB0', esp32_baud=115200, timeout=0.5,
                 debug=False, socket_server=False):
        """Initialize MyAGVPlus controller base instance.

        Args:
            motor_port (str): Serial port path for DM motors.
            baudrate (int): Baud rate for motor serial. Default 921600.
            esp32_port (str): Serial port path for ESP32.
            esp32_baud (int): Baud rate for ESP32 serial. Default 115200.
            timeout (float): Serial port read timeout in seconds. Default 0.5.
            debug (bool): Enable verbose debug logging. Default False.
            socket_server (bool): Flag indicating if instance runs inside socket server. Default False.
        """
        if MyAGVPlusApi._instance_created:
            raise Exception("Error: Please do not instantiate multiple times.")
            
        self._singleton_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._singleton_socket.bind(('127.0.0.1', 48123))
        except socket.error:
            raise Exception("Error: Another instance of MyAGVPlus is already running in a different terminal/process. Please do not instantiate multiple times.")
            
        MyAGVPlusApi._instance_created = True

        self._debug = debug
        setup_logging(self._debug)
        self.log = logging.getLogger(__name__)
        self._esp32_log = logging.getLogger("pymycobot.common")
        # trigger setter to update log levels
        self.debug = debug
        self.calibration_parameters = calibration_parameters
        self._lock = threading.Lock()
        self._dm_lock = threading.RLock()

        # Detect if running as socket server
        is_server = False
        if sys.argv:
            main_script = os.path.basename(sys.argv[0]) if hasattr(os, 'path') else ''
            if "server" in main_script.lower():
                is_server = True
        self.socket_server = socket_server or is_server
        self._power_on_state = False
        self._state_file = "/tmp/myagvplus_error_state.json"
        self._undervoltage_locked_motors = [False, False, False, False]
        self._stall_locked_motors = [False, False, False, False]
        self._load_error_state()
        self.last_serial_logs = {"write": [], "read": [], "error": []}

        # Initialize DM Motor Control
        self.motor_serial = serial.Serial(motor_port, baudrate, timeout=timeout)
        self.motor_driver = MotorControl(self.motor_serial, debug=self.debug)

        # Motor 1 — Left-Front Wheel 2
        self.rl_wheel_joint = Motor(DM_Motor_Type.DM2325, 0x02, 0x12)
        # Motor 2 — Right-Front Wheel 3
        self.fl_wheel_joint = Motor(DM_Motor_Type.DM2325, 0x03, 0x13)
        # Motor 3 — Left-Rear Wheel 1
        self.fr_wheel_joint = Motor(DM_Motor_Type.DM2325, 0x01, 0x11)
        # Motor 4 — Right-Rear Wheel
        self.rr_wheel_joint = Motor(DM_Motor_Type.DM2325, 0x04, 0x14)
        self.motors = [self.rl_wheel_joint, self.fl_wheel_joint, self.fr_wheel_joint, self.rr_wheel_joint]

        for m in self.motors:
            self.motor_driver.addMotor(m)
        for m in self.motors:
            self.motor_driver.change_motor_param(m, 0, 19.0)  # 0 is DM_variable.UV_Value (欠压)
        if self._debug:
            self.log.info("Successfully configured undervoltage protection for all four wheels.")
        for m in self.motors:
            self.motor_driver.change_motor_param(m, 3, 0.08)  # 3 is DM_variable.OC_Value (过流)
        if self._debug:
            self.log.info("Successfully configured maximum overcurrent protection for all four wheels.")

        with self._dm_lock:
            for m in self.motors:
                self.motor_driver.enable(m)

        # Dynamic Socket Lock to replace stale file cache
        if self.socket_server:
            self._socket_lock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self._socket_lock.bind(('127.0.0.1', 20235))
            except Exception:
                self.log.error("Warning: Socket lock 20235 is already bound.")

        self._latest_auto_report = None
        self._auto_report_buffer = b""

        self.esp32_serial = None
        if esp32_port:
            try:
                self.esp32_serial = serial.Serial()
                self.esp32_serial.port = esp32_port
                self.esp32_serial.baudrate = esp32_baud
                self.esp32_serial.timeout = timeout
                self.esp32_serial.rts = False
                self.esp32_serial.dtr = False
                self.esp32_serial.open()
                # The low-level driver might still trigger an ESP32 reset on open.
                # Wait 5.0 seconds for it to boot, then clear the bootloader spam.
                time.sleep(5.0)
                self.esp32_serial.reset_input_buffer()
            except Exception as e:
                raise IOError("Failed to open ESP32 port: {}".format(e))

        atexit.register(self._cleanup)

        # Determine initial communication state
        # Prefer persisted state saved in temp file, so if user manually set it to 1, it is preserved.
        # But if no cache exists (e.g. after a fresh boot), always default to 0 (Serial Mode).
        try:
            cached_state = self.get_communication_state(from_cache=True)
        except Exception:
            cached_state = None
            
        init_state = 0 if cached_state is None else cached_state

        if not self.socket_server and self._is_socket_server_active():
            self.log.warning("Socket Server is actively running. Skipping automatic reset to Serial Mode.")
        else:
            self.set_communication_state(init_state, force=True)
        
        # Bug 3: Detect actual power state on init (handles re-instantiation while powered on)
        try:
            resp = self._merge(MyagvPlusCommand.IS_POWER_ON)
            if resp == 1:
                self._power_on_state = True
                if self._debug:
                    self.log.info("Power on completed.")
                # If powered on, re-enable motors in case they were disabled by a previous session's exit cleanup
                if getattr(self, 'motor_driver', None) and getattr(self, 'motor_serial', None) and getattr(self.motor_serial, 'is_open', False):
                    for m in self.motors:
                        self.motor_driver.enable(m)
                        self.motor_driver.switchControlMode(m, Control_Type.VEL)
                    if self._debug:
                        self.log.info("Motor enable completed.")
        except Exception:
            pass

        def _quiet_keyboard_interrupt(exc_type, exc_value, exc_traceback):
            if issubclass(exc_type, KeyboardInterrupt):
                return
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
        sys.excepthook = _quiet_keyboard_interrupt

        # Automatically start stall protection in the background by default
        self.set_stall_protection(True)
        
        if self._debug:
            self.log.info("MyAGVPlus initialized successfully. Max current protection enabled.")

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, val):
        if str(val).lower() in ("false", "0", "none"):
            val = False
        self._debug = bool(val)
        level = logging.DEBUG if self._debug else logging.WARNING
        if hasattr(self, 'log'):
            self.log.setLevel(level)
        if hasattr(self, '_esp32_log'):
            self._esp32_log.setLevel(level)
        if hasattr(self, 'motor_driver') and self.motor_driver:
            self.motor_driver.debug = val

    def _load_error_state(self):
        try:
            if os.path.exists(self._state_file):
                with open(self._state_file, 'r') as f:
                    data = json.load(f)
                    self._stall_locked_motors = data.get("stall_locked_motors", [False, False, False, False])
                    self._undervoltage_locked_motors = data.get("undervoltage_locked_motors", [False, False, False, False])
        except Exception:
            pass

    def _save_error_state(self):
        try:
            with open(self._state_file, 'w') as f:
                json.dump({
                    "stall_locked_motors": self._stall_locked_motors,
                    "undervoltage_locked_motors": getattr(self, '_undervoltage_locked_motors', [False, False, False, False])
                }, f)
        except Exception:
            pass

    def _cleanup(self):
        """Cleanup on Python exit: stop motors and disable."""
        MyAGVPlusApi._instance_created = False
        self._stall_protection_active = False
        if getattr(self, '_stall_thread', None) and self._stall_thread.is_alive():
            try:
                self._stall_thread.join(timeout=0.2)
            except Exception:
                pass
        try:
            if self.motor_driver:
                saved_debug = self.motor_driver.debug
                self.motor_driver.debug = False
                self.motor_driver.control_Vel(self.rl_wheel_joint, 0)
                self.motor_driver.control_Vel(self.fl_wheel_joint, 0)
                self.motor_driver.control_Vel(self.fr_wheel_joint, 0)
                self.motor_driver.control_Vel(self.rr_wheel_joint, 0)
                time.sleep(0.05)
                for m in self.motors:
                    self.motor_driver.disable(m)
                self.motor_driver.debug = saved_debug
        except Exception:
            pass
        if getattr(self, '_socket_lock', None):
            try:
                self._socket_lock.close()
            except Exception:
                pass

    def _is_socket_server_active(self):
        """Dynamically check if the socket server process is currently running."""
        if getattr(self, 'socket_server', False):
            return True
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('127.0.0.1', 20235))
            s.close()
            return False
        except Exception:
            return True

    def _check_comm_state(self):
        """Check serial-socket communication mutual exclusion status.

        Returns:
            bool: True if allowed to proceed, False if blocked by state.
        """
        if getattr(self, 'socket_server', False):
            return True

        if self._is_socket_server_active():
            if hasattr(self, 'log') and not getattr(self, '_suppress_esp_log', False):
                self.log.error("Communication error: Serial ports are disabled because Socket Server is actively running.")
            return False

        if getattr(self, '_comm_state', 0) == 1:
            if hasattr(self, 'log') and not getattr(self, '_suppress_esp_log', False):
                self.log.error("Communication error: Serial ports are disabled because communication state is set to Socket (1).")
            return False

        return True

    def _check_motor_ready(self):
        """Check if motors are ready for commands (comm state).

        Returns:
            -1 if not ready (interface layer can return this directly),
            True if ready.
        """
        if not self._check_comm_state():
            return -1
        return True

    def _merge(self, cmd, *args):
        """Unified ESP32 command send/receive/parse entry point.

        Args:
            cmd (int): MyagvPlusCommand code.
            *args: Payload data values (auto-padded to 8 bytes).

        Returns:
            Parsed data from _parsing_data, or None if communication failed.
        """
        data = list(args) + [0] * (8 - len(args))
        resp = self._send_esp32_command(cmd, data)
        return self._parsing_data(cmd, resp, args)

    def _parsing_data(self, cmd, resp, args=()):
        """Parse ESP32 response based on command type.

        Args:
            cmd (int): MyagvPlusCommand code.
            resp (bytes): Raw response payload from ESP32, or None.
            args (tuple): Original arguments passed to _merge.

        Returns:
            Parsed data, or None if resp is None.
        """
        if resp is None:
            return -1

        if cmd == MyagvPlusCommand.GET_SYSTEM_VERSION:
            return resp[0] / 10.0

        if cmd == MyagvPlusCommand.GET_MODIFY_VERSION:
            return int(resp[0])

        if cmd in (MyagvPlusCommand.GET_DEBUG_STATE,
                   MyagvPlusCommand.GET_AUTO_REPORT_STATE):
            return resp[0]

        if cmd == MyagvPlusCommand.IS_POWER_ON:
            self._power_on_state = (resp[0] == 1)
            return resp[0]

        if cmd == MyagvPlusCommand.GET_ROBOT_STATUS:
            return [resp[0], resp[1], resp[2]]

        if cmd == MyagvPlusCommand.GET_ALL_MSG:
            return self._parse_all_msg(resp)

        if cmd == MyagvPlusCommand.GET_PIN_INPUT:
            pin = args[0] if args else 0
            if pin == 0:
                return [-1 if val == 255 else val for val in resp[1:7]]
            return -1 if resp[1] == 255 else resp[1]

        if cmd == MyagvPlusCommand.GET_COMMUNICATION_STATE:
            return resp[0]

        # Default for set-commands: return 1 indicating success
        return 1

    def _send_esp32_command(self, cmd, data, read_plaintext=False, suppress_log=False):
        """Send formatted command frame to ESP32 board and parse reply.

        Args:
            cmd (int): ESP32 Protocol Command byte.
            data (list): 8-element payload data list.
            read_plaintext (bool): If True, reads plaintext newline-terminated line.

        Returns:
            bytes: Parsed payload response bytes, or None if timeout/error.
        """
        if cmd not in (MyagvPlusCommand.SET_COMMUNICATION_STATE, MyagvPlusCommand.GET_COMMUNICATION_STATE):
            if not self._check_comm_state():
                return None
        if not self.esp32_serial or not self.esp32_serial.is_open:
            return None

        with self._lock:
            frame = bytearray([0xFE, 0xFE, 0x0B, cmd])
            frame.extend(data[:8])
            while len(frame) < 12:
                frame.append(0)

            crc_bytes = _crc16_modbus(frame)
            frame.extend(crc_bytes)

            self.esp32_serial.write(frame)
            self.esp32_serial.flush()
            if not getattr(self, '_suppress_esp_log', False) and not suppress_log:
                if hasattr(self, 'last_serial_logs'):
                    self.last_serial_logs["write"].append(" ".join("{:02x}".format(b) for b in frame))
                self._esp32_log.debug("_write: {}".format(" ".join("{:02x}".format(b) for b in frame)))

            if read_plaintext:
                start_time = time.time()
                buf = b""
                while time.time() - start_time < 0.5:
                    if self.esp32_serial.in_waiting:
                        buf += self.esp32_serial.read(1)
                        if buf.endswith(b"\n"):
                            if hasattr(self, 'last_serial_logs'):
                                self.last_serial_logs["read"].append(buf.decode('utf-8', errors='ignore').strip())
                            self._esp32_log.debug("_read : {}".format(buf))
                            return buf.decode('utf-8', errors='ignore')
                self._esp32_log.debug("_read timeout (plaintext)")
                return None

            start_time = time.time()
            buf = b""
            while time.time() - start_time < 2.0:
                if self.esp32_serial.in_waiting:
                    buf += self.esp32_serial.read(self.esp32_serial.in_waiting)

                while b'\xfe\xfe' in buf:
                    idx = buf.find(b'\xfe\xfe')
                    buf = buf[idx:]

                    if len(buf) >= 3:
                        length = buf[2]
                        frame_len = 3 + length

                        if len(buf) >= frame_len:
                            full_frame = buf[:frame_len]
                            buf = buf[frame_len:]

                            recv_cmd = full_frame[3]
                            recv_crc = full_frame[-2:]
                            calc_crc = _crc16_modbus(full_frame[:-2])

                            if recv_crc == calc_crc:
                                if recv_cmd == cmd:
                                    if not getattr(self, '_suppress_esp_log', False) and not suppress_log:
                                        if hasattr(self, 'last_serial_logs'):
                                            self.last_serial_logs["read"].append(" ".join("{:02x}".format(b) for b in full_frame))
                                        self._esp32_log.debug("_read : {}".format(" ".join("{:02x}".format(b) for b in full_frame)))
                                    return full_frame[4:-2]
                                elif recv_cmd == 0x25:
                                    self._latest_auto_report = full_frame
                                    continue
                            else:
                                buf = buf[2:]
                        else:
                            break
                    else:
                        break

                time.sleep(0.005)

            if not getattr(self, '_suppress_esp_log', False):
                self._esp32_log.debug("_read timeout for cmd 0x{:02X}".format(cmd))
            return None

    def _read_auto_report(self):
        """Read and parse the latest auto-report frame from ESP32 buffer.

        Blocks up to 100ms waiting for a fresh auto-report frame (firmware sends
        every 50ms), matching the blocking pattern used by MyAGVPro._merge.

        Returns:
            list: Parsed auto-report data, or None if no valid data within timeout.
        """
        if not self._check_comm_state():
            return -1

        timeout = 0.1  # 100ms, enough to catch one 50ms auto-report cycle
        start_time = time.time()

        with self._lock:
            # Clear old cache so we only return freshly received data
            self._auto_report_buffer = b""
            self._latest_auto_report = None

            # Block and wait for a complete new frame, like MyAGVPro._read_by_timeout
            while time.time() - start_time < timeout:
                if self.esp32_serial and self.esp32_serial.is_open:
                    if self.esp32_serial.in_waiting > 0:
                        self._auto_report_buffer += self.esp32_serial.read_all()

                # Try to parse a complete frame from the buffer
                buf = self._auto_report_buffer
                i = 0
                latest_valid_frame = None
                last_processed_idx = 0
                while i <= len(buf) - 4:
                    if buf[i] == 0xFE and buf[i + 1] == 0xFE:
                        length = buf[i + 2]
                        cmd = buf[i + 3]
                        frame_len = 3 + length
                        if i + frame_len <= len(buf):
                            full_frame = buf[i: i + frame_len]
                            recv_crc = full_frame[-2:]
                            calc_crc = _crc16_modbus(full_frame[:-2])
                            if recv_crc == calc_crc:
                                if cmd == 0x25:
                                    latest_valid_frame = full_frame
                                i += frame_len
                                last_processed_idx = i
                            else:
                                i += 2
                        else:
                            break
                    else:
                        i += 1

                if last_processed_idx > 0:
                    self._auto_report_buffer = buf[last_processed_idx:]

                # Got a valid frame — return immediately
                if latest_valid_frame:
                    self._esp32_log.debug("_read_auto_report: {}".format(
                        " ".join("{:02x}".format(b) for b in latest_valid_frame)
                    ))
                    if hasattr(self, 'last_serial_logs'):
                        self.last_serial_logs["read"].append(
                            " ".join("{:02x}".format(b) for b in latest_valid_frame)
                        )
                    resp = latest_valid_frame[4:-2]
                    if len(resp) >= 24:
                        return self._parse_all_msg(resp)

                # No complete frame yet, wait a bit before retrying
                time.sleep(0.005)

        # Timed out without receiving a valid auto-report frame
        self._esp32_log.debug("_read_auto_report: no valid auto-report data available.")
        return None

    def _power_on(self):
        """Execute full power-on sequence: relay + motor enable.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        # Guard: motor serial must be open (closed in socket mode)
        if not (self.motor_serial and self.motor_serial.is_open):
            self.log.error("Communication error: Motor serial port is closed (socket mode), cannot power on.")
            return -1

        for m in self.motors:
            m.req_count = 0
            m.recv_count = 0
        resp = self._merge(MyagvPlusCommand.POWER_CONTROL, 1)
        if resp == -1:
            return -1
        self._power_on_state = True
        time.sleep(6)

        self.motor_serial.reset_input_buffer()

        with self._dm_lock:
            for m in self.motors:
                self.motor_driver.enable(m)
                self.motor_driver.switchControlMode(m, Control_Type.VEL)
        return 1

    def _power_off(self):
        """Execute full power-off sequence: motor disable + relay off.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        for m in self.motors:
            m.req_count = 0
            m.recv_count = 0

        # Guard: motor serial might be closed in socket mode
        if self.motor_serial and self.motor_serial.is_open:
            if getattr(self, '_power_on_state', False):
                self._set_velocity(0, 0, 0, 0)
                with self._dm_lock:
                    for m in self.motors:
                        self.motor_driver.disable(m)
                time.sleep(0.1)

        resp = self._merge(MyagvPlusCommand.POWER_CONTROL, 0)
        self._power_on_state = False
        if resp == -1:
            return -1
        return 1

    def _set_motor_velocity_raw(self, w1, w2, w3, w4):
        """Dispatch raw motor angular velocities (rad/s) directly.

        Handles comm state check, undervoltage lock, deadzone and dispatch.
        Used by both linear (_set_velocity) and rotational (turn) commands.

        Args:
            w1..w4 (float): Motor angular velocities in rad/s.

        Returns:
            int: 1 if dispatched, -1 if blocked.
        """
        if not self._check_comm_state():
            return -1

        if not getattr(self, '_power_on_state', False):
            msg = "Error: Currently in power-off state."
            if hasattr(self, 'log'):
                self.log.error(msg)
            if hasattr(self, 'last_serial_logs') and "error" in self.last_serial_logs:
                self.last_serial_logs["error"].append(msg)
            return -1

        is_motion = any(abs(w) > 0 for w in (w1, w2, w3, w4))

        if is_motion and any(getattr(self, '_undervoltage_locked_motors', [False, False, False, False])):
            msg = "Warning: Undervoltage lock active. Please charge."
            self.log.warning(msg)
            return msg

        if is_motion and any(getattr(self, '_stall_locked_motors', [False, False, False, False])):
            msg = "Warning: Stall lock active (overcurrent/torque overload). Run clear_motor_error to reset."
            self.log.warning(msg)
            return msg

        w1 = 0 if abs(w1) < 0.01 else w1
        w2 = 0 if abs(w2) < 0.01 else w2
        w3 = 0 if abs(w3) < 0.01 else w3
        w4 = 0 if abs(w4) < 0.01 else w4

        if w1 == 0 or w2 == 0 or w3 == 0 or w4 == 0:
            w1 = w2 = w3 = w4 = 0
            self._is_moving = False
        else:
            self._is_moving = True

        self._expected_w = [w1, w2, w3, w4]

        with self._dm_lock:
            self.motor_driver.control_Vel(self.rl_wheel_joint, w1)
            self.motor_driver.control_Vel(self.fl_wheel_joint, w2)
            self.motor_driver.control_Vel(self.fr_wheel_joint, w3)
            self.motor_driver.control_Vel(self.rr_wheel_joint, w4)

        # Detect undervoltage (err==9) from motor feedback and lock
        for idx, m in enumerate(self.motors):
            if m.getError() == 9:
                self._undervoltage_locked_motors[idx] = True
                self.log.warning(
                    "Motor {} undervoltage error (err=9), all motion locked.".format(idx + 1)
                )
                break

        return 1 if self._power_on_state else -1

    def set_stall_protection(self, enable=True):
        """Enable or disable active motor stall protection.
        
        Args:
            enable (bool): If True, background thread will monitor for hardware overcurrent (Error 10/14)
                           and trigger emergency stop for all motors.
        """
        if enable:
            self._stall_protection_active = True
            if not hasattr(self, '_stall_thread') or not self._stall_thread.is_alive():
                import threading
                self._stall_thread = threading.Thread(target=self._stall_monitor_loop, daemon=True)
                self._stall_thread.start()
        else:
            self._stall_protection_active = False

    def _stall_monitor_loop(self):
        """Background thread polling motor error codes for emergency stall stop."""
        import time
        last_vol_check = 0
        while getattr(self, '_stall_protection_active', False):
            try:
                # 1. Background Voltage Check (Every 2 seconds, regardless of power state)
                current_time = time.time()
                if current_time - last_vol_check > 2.0:
                    last_vol_check = current_time
                    if not all(getattr(self, '_undervoltage_locked_motors', [False, False, False, False])) and hasattr(self, 'get_all_msg'):
                        try:
                            self._suppress_esp_log = True
                            msg = self.get_all_msg()
                        finally:
                            self._suppress_esp_log = False
                        
                        try:
                            if msg != -1 and isinstance(msg, list) and len(msg) >= 6:
                                vol1, vol2 = msg[4], msg[5]
                                is_v1_dead, is_v1_ok = (0 < vol1 < 19.0), (vol1 >= 19.0)
                                is_v2_dead, is_v2_ok = (0 < vol2 < 19.0), (vol2 >= 19.0)
                                
                                if not is_v1_ok and not is_v2_ok and (is_v1_dead or is_v2_dead):
                                    self.log.warning(f"BACKGROUND VOLTAGE DETECTED! Battery low (V1:{vol1}V, V2:{vol2}V). Emergency Lock!")
                                    self._undervoltage_locked_motors = [True, True, True, True]
                                    self._save_error_state()
                                    self._set_velocity(0, 0, 0, 0)
                                    import sys
                                    if hasattr(sys, 'ps1'):
                                        sys.stdout.write('\n' + sys.ps1)
                                        sys.stdout.flush()
                        except Exception:
                            pass

                # 2. Stall Check (Only if powered on and moving)
                if getattr(self, '_power_on_state', False):
                    if getattr(self, '_is_moving', False):
                        # Thread-safe motor read through the lock
                        stall_detected = False
                        
                        with self._dm_lock:
                            original_debug = getattr(self.motor_driver, 'debug', False)
                            self.motor_driver.debug = False
                            for i, m in enumerate(self.motors):
                                self.motor_driver.refresh_motor_status(m)
                                err = m.getError()
                                # Error 9 is Undervoltage
                                if err == 9:
                                    self.log.warning(f"UNDERVOLTAGE DETECTED! Motor {i+1} (err={err}). Emergency Stop!")
                                    stall_detected = True
                                    self._undervoltage_locked_motors[i] = True
                                    self._save_error_state()
                                    break
                                
                                # Torque overload stall detection (> 0.55 N.m)
                                actual_t = m.getTorque()
                                if abs(actual_t) > 0.55:
                                    self.log.warning(f"STALL DETECTED! Motor {i+1} torque overload ({actual_t:.2f} > 0.55). Emergency Stop!")
                                    stall_detected = True
                                    self._stall_locked_motors[i] = True
                                    self._save_error_state()
                                    break
                            self.motor_driver.debug = original_debug
                        
                        if stall_detected:
                            # Emergency stop: just set velocity to 0 (stop), do not power off
                            self._set_velocity(0, 0, 0, 0)
                            import sys
                            if hasattr(sys, 'ps1'):
                                sys.stdout.write('\n' + sys.ps1)
                                sys.stdout.flush()
            except Exception:
                pass
            time.sleep(0.05)  # 20 Hz

    def _set_velocity(self, v1, v2, v3, v4):
        """Map linear speeds (m/s) to motor angular speeds (rad/s) and dispatch.

        Args:
            v1 (float): Left-Front wheel linear velocity.
            v2 (float): Right-Front wheel linear velocity.
            v3 (float): Left-Rear wheel linear velocity.
            v4 (float): Right-Rear wheel linear velocity.

        Returns:
            int: 1 if dispatched, -1 if offline, blocked or speed out of range.
        """
        # Validate speed range: 0 (stop) or |v| in [0.01, 1.60] m/s
        for v in (v1, v2, v3, v4):
            mag = abs(v)
            if mag != 0 and not (0.01 <= mag <= 1.60):
                self.log.warning(
                    "Speed out of range: |{}| must be 0 or within [0.01, 1.60] m/s.".format(v)
                )
                return -1

        w1 = v1 / WHEEL_RADIUS
        w2 = v2 / WHEEL_RADIUS
        w3 = v3 / WHEEL_RADIUS
        w4 = v4 / WHEEL_RADIUS
        return self._set_motor_velocity_raw(w1, w2, w3, w4)

    def _parse_gyro_data(self, resp, offset=6, count=18):
        """Parse raw gyroscope data from ESP32 response.

        Args:
            resp (bytes): Raw response payload.
            offset (int): Start offset of gyro data.
            count (int): Byte length of gyro data.

        Returns:
            list[float]: Parsed gyro values.
        """
        raw_gyro = bytes(resp[offset:offset + count])
        gyro_parsed = []
        for i in range(0, count, 2):
            val = struct.unpack('>h', raw_gyro[i:i + 2])[0]
            gyro_parsed.append(round(val / 100.0, 2))
        return gyro_parsed

    def _parse_all_msg(self, resp):
        """Unified parsing for GET_ALL_MSG and auto-report responses.

        Args:
            resp (bytes): Raw response payload (24 or 28+ bytes).

        Returns:
            list: Parsed status data list, or -1 if response too short.
        """
        if not resp or len(resp) < 24:
            return -1

        battery_state = resp[0]
        gyro_state = resp[1]
        power_level = resp[2]
        charge_state = format(resp[3], '08b')
        battery_voltage1 = resp[4] / 10.0
        battery_voltage2 = resp[5] / 10.0
        gyro_data = self._parse_gyro_data(resp)

        if len(resp) >= 28:
            fine_vol1 = struct.unpack('>H', bytes(resp[24:26]))[0] / 100.0
            fine_vol2 = struct.unpack('>H', bytes(resp[26:28]))[0] / 100.0
            return [battery_state, gyro_state, power_level, charge_state,
                    battery_voltage1, battery_voltage2, gyro_data, fine_vol1, fine_vol2]

        return [battery_state, gyro_state, power_level, charge_state,
                battery_voltage1, battery_voltage2, gyro_data]

    def set_communication_state(self, state, force=False):
        """Toggle hardware communication interface state.

        Args:
            state (int): 0 for Serial, 1 for Socket
            force (bool): If True, bypasses the socket mode lock to force the state change.

        Returns:
            int: 1 if successful, -1 otherwise.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, communication_state=state)

        # Bug 5: Prevent serial scripts from forcing the state to Serial if Socket is active
        if not getattr(self, 'socket_server', False) and not force:
            if self._is_socket_server_active():
                self.log.error("Cannot change state: Socket Server is actively running. Use force=True to override.")
                return -1

        if state in [0, 1]:
            if getattr(self, 'esp32_serial', None) and not self.esp32_serial.is_open:
                try:
                    self.esp32_serial.open()
                    import time
                    time.sleep(3.0)
                    self.esp32_serial.reset_input_buffer()
                except Exception:
                    pass
        if state == 0:
            if getattr(self, 'motor_driver', None) and getattr(self.motor_driver, 'serial_', None) and not self.motor_driver.serial_.is_open:
                try:
                    self.motor_driver.serial_.open()
                except Exception:
                    pass

        if state == 2:
            mac = self.get_bluetooth_address()
            if mac:
                try:
                    info = "MyAGVPlus Bluetooth MAC Address: {}".format(mac)
                    with open("AGVPlus_BLUETOOTH_MAC_ADDR", "w") as f:
                        f.write(info + "\n")
                except Exception:
                    pass

        self._comm_state = state
        try:
            file_path = '/dev/shm/.agv_comm_state' if os.path.exists('/dev/shm') else os.path.join(tempfile.gettempdir(), '.agv_comm_state')
            with open(file_path, "w") as f:
                f.write(str(state))
        except Exception:
            pass

        resp = self._send_esp32_command(MyagvPlusCommand.SET_COMMUNICATION_STATE, [state] + [0] * 7)

        if state == 1 and not self.socket_server:
            try:
                if self.motor_driver and self.motor_driver.serial_:
                    self.motor_driver.serial_.close()
                if self.esp32_serial:
                    self.esp32_serial.close()
            except Exception:
                pass

        if resp:
            return 1
        return -1

    def get_communication_state(self, *, from_cache=False, simulate=False):
        """Read currently active communication interface setting.

        Returns:
            int: 0 (Serial), 1 (Socket), 2 (Bluetooth), or -1 if failed.
        """
        cached_state = None
        try:
            file_path = '/dev/shm/.agv_comm_state' if os.path.exists('/dev/shm') else os.path.join(tempfile.gettempdir(), '.agv_comm_state')
            if os.path.exists(file_path):
                with open(file_path, "r") as f:
                    cached_state = int(f.read().strip())
        except Exception:
            pass

        if from_cache:
            return cached_state

        if cached_state is not None:
            if simulate and not getattr(self, '_suppress_esp_log', False):
                # Simulate debug output to show the state query
                frame = bytearray([0xFE, 0xFE, 0x0B, MyagvPlusCommand.GET_COMMUNICATION_STATE, 0,0,0,0,0,0,0,0])
                frame.extend(_crc16_modbus(frame))
                if hasattr(self, 'last_serial_logs'):
                    self.last_serial_logs["write"].append(" ".join("{:02x}".format(b) for b in frame))
                self._esp32_log.debug("_write: {}".format(" ".join("{:02x}".format(b) for b in frame)))
                
                resp_frame = bytearray([0xFE, 0xFE, 0x0B, MyagvPlusCommand.GET_COMMUNICATION_STATE, cached_state,0,0,0,0,0,0,0])
                resp_frame.extend(_crc16_modbus(resp_frame))
                if hasattr(self, 'last_serial_logs'):
                    self.last_serial_logs["read"].append(" ".join("{:02x}".format(b) for b in resp_frame))
                self._esp32_log.debug("_read : {}".format(" ".join("{:02x}".format(b) for b in resp_frame)))
            return cached_state

        resp = self._send_esp32_command(MyagvPlusCommand.GET_COMMUNICATION_STATE, [0] * 8, suppress_log=False)

        if resp:
            return resp[0]
        return -1

    def get_bluetooth_address(self):
        """Read adapter Bluetooth MAC address from system.

        Returns:
            str: Bluetooth MAC address string, or empty string if unavailable.
        """
        mac = ""
        try:
            import subprocess, re
            result = subprocess.check_output(['hciconfig'], stderr=subprocess.DEVNULL).decode('utf-8')
            match = re.search(r'BD Address:\s*([0-9A-Fa-f:]+)', result)
            if match:
                mac = match.group(1)
        except Exception:
            pass
        return mac

    def get_last_serial_logs(self):
        """Hidden API to fetch raw serial bytes captured during the last RPC call.
        
        Returns:
            dict: Buffered write and read hex strings from ESP32 and DM_CAN interfaces.
        """
        logs = {
            "esp32": getattr(self, 'last_serial_logs', {"write": [], "read": [], "error": []}),
            "dm_can": getattr(getattr(self, 'motor_driver', None), 'last_serial_logs', {"write": [], "read": []})
        }
        # Clear logs after fetching
        if hasattr(self, 'last_serial_logs'):
            self.last_serial_logs = {"write": [], "read": [], "error": []}
        if hasattr(self, 'motor_driver') and hasattr(self.motor_driver, 'last_serial_logs'):
            self.motor_driver.last_serial_logs = {"write": [], "read": []}
        return logs

    def get_wifi_ip(self):
        """Get connected Wi-Fi local network IP.

        Returns:
            str: Local IP address string, or empty string if unavailable.
        """
        ip = ""
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass
        return ip

    def get_wifi_account(self):
        """Get connected Wi-Fi SSID network account name.

        Returns:
            str: SSID name string, or empty string if unavailable.
        """
        ssid = ""
        try:
            import subprocess
            result = subprocess.check_output(['iwgetid', '-r'], stderr=subprocess.DEVNULL)
            ssid = result.decode('utf-8').strip()
        except Exception:
            pass
        return ssid
