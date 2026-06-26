# coding=utf-8
import socket
import json
import struct
import logging
import time

from pymycobot.error import calibration_parameters
from pymycobot.log import setup_logging
from pymycobot.myagvplusapi import _crc16_modbus
from pymycobot.common import MyagvPlusCommand

# ============== Constants ==============
WHEEL_RADIUS = 0.04
WHEEL_BASE_SUM = 0.221

# ============== ESP32 frame reconstruction ==============

# Map RPC function names -> (ESP32 command code, fixed_data or None).
_ESP32_CMD_MAP = {
    "get_system_version":      (MyagvPlusCommand.GET_SYSTEM_VERSION, None),
    "get_modify_version":      (MyagvPlusCommand.GET_MODIFY_VERSION, None),
    "set_debug_state":         (MyagvPlusCommand.SET_DEBUG_STATE, None),
    "get_debug_state":         (MyagvPlusCommand.GET_DEBUG_STATE, None),
    "get_robot_status":        (MyagvPlusCommand.GET_ROBOT_STATUS, None),
    "power_on":                (MyagvPlusCommand.POWER_CONTROL, [1]),
    "power_off":               (MyagvPlusCommand.POWER_CONTROL, [0]),
    "is_power_on":             (MyagvPlusCommand.IS_POWER_ON, None),
    "get_all_msg":             (MyagvPlusCommand.GET_ALL_MSG, None),
    "set_auto_report_state":   (MyagvPlusCommand.SET_AUTO_REPORT_STATE, None),
    "get_auto_report_state":   (MyagvPlusCommand.GET_AUTO_REPORT_STATE, None),
    "set_communication_state": (MyagvPlusCommand.SET_COMMUNICATION_STATE, None),
    "get_communication_state": (MyagvPlusCommand.GET_COMMUNICATION_STATE, None),
    "set_led_color":           (MyagvPlusCommand.SET_LED_COLOR, None),
    "set_led_mode":            (MyagvPlusCommand.SET_LED_MODE, None),
    "set_pin_output":          (MyagvPlusCommand.SET_PIN_OUTPUT, None),
    "get_pin_input":           (MyagvPlusCommand.GET_PIN_INPUT, None),
    "set_fan_state":           (MyagvPlusCommand.SET_FAN_STATE, None),
    "get_auto_report_message": (0x25, None),
}


def _build_esp32_frame(cmd, data_args):
    """Build ESP32 protocol frame: FE FE 0B [cmd] [data x8] [crc16]."""
    frame = bytearray([0xFE, 0xFE, 0x0B, cmd])
    d = list(data_args) + [0] * (8 - len(data_args))
    frame.extend(d[:8])
    crc_bytes = _crc16_modbus(frame)
    frame.extend(crc_bytes)
    return frame


# ============== DM CAN frame reconstruction ==============

# 30-byte DM CAN frame template (from MotorControl.send_data_frame)
_DM_FRAME_TEMPLATE = bytearray([
    0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
])

# Movement: func_name -> wheel velocity signs [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint]
_MOVE_VEL_SIGNS = {
    "move_forward":       [-1,  1, -1,  1],
    "move_backward":      [ 1, -1,  1, -1],
    "move_left_lateral":  [ 1,  1, -1, -1],
    "move_right_lateral": [-1, -1,  1,  1],
    "stop":               [ 0,  0,  0,  0],
}

# Motor read commands that refresh all 4 motors
_MOTOR_READ_ALL = {
    "get_motor_enable_status", "get_motor_status", "get_motor_temps",
    "get_motor_positions", "get_motor_move_speeds", "get_motor_turn_speeds", "get_motor_torques",
}

# Motor read commands that refresh a single motor (first arg = motor_id)
_MOTOR_READ_SINGLE = {
    "get_motor_velocity", "get_motor_torque", "get_motor_position",
    "get_motor_param_cache",
}


def _build_dm_vel_frame(slave_id, velocity):
    """Build DM CAN velocity control frame: 55 AA 1E ... [motor_id] ... [vel_float] ..."""
    frame = bytearray(_DM_FRAME_TEMPLATE)
    motor_can_id = 0x200 + slave_id
    frame[13] = motor_can_id & 0xFF
    frame[14] = (motor_can_id >> 8) & 0xFF
    frame[21:25] = struct.pack('f', float(velocity))
    return frame


def _build_dm_cmd_frame(slave_id, cmd_byte):
    """Build DM CAN control command frame (enable=0xFC, disable=0xFD, set_zero=0xFE)."""
    frame = bytearray(_DM_FRAME_TEMPLATE)
    frame[13] = slave_id & 0xFF
    frame[14] = (slave_id >> 8) & 0xFF
    frame[21:29] = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte])
    return frame


def _fmt_dm(frame):
    """Format DM CAN frame as uppercase hex string (matching serial debug)."""
    return " ".join("{:02X}".format(b) for b in frame)


def _fmt_esp(frame):
    """Format ESP32 frame as lowercase hex string (matching serial debug)."""
    return " ".join("{:02x}".format(b) for b in frame)


class MyAGVPlusSocket(object):
    """MyAGV Plus Socket Client API

    This class is used to remotely control MyAGV Plus over TCP Socket.
    """
    
    _instance_created = False

    def __init__(self, ip, port=9000, timeout=15.0, debug=False):
        """Initialize TCP Socket connection to MyAGV Plus server daemon.

        Args:
            ip (str): Server IP address of the AGV Jetson motherboard.
            port (int): Server port, default is 9000.
            timeout (float): Socket connection timeout in seconds.
            debug (bool): Enable verbose communication packet logs.
        """
        if MyAGVPlusSocket._instance_created:
            raise Exception("Error: Please do not instantiate multiple times.")
            
        self._singleton_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._singleton_socket.bind(('127.0.0.1', 48124))
        except socket.error:
            raise Exception("Error: Another instance of MyAGVPlusSocket is already running in a different terminal/process. Please do not instantiate multiple times.")

        MyAGVPlusSocket._instance_created = True
        self.calibration_parameters = calibration_parameters

        self.SERVER_IP = ip
        self.SERVER_PORT = port
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._esp32_log = logging.getLogger("pymycobot.common")
        self._dm_log = logging.getLogger("pymycobot.dm_can")
        if not debug:
            self.log.setLevel(logging.WARNING)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((self.SERVER_IP, self.SERVER_PORT))
    def _log_write_frames(self, func_name, args):
        """Reconstruct and log protocol write frames for debug."""
        # --- ESP32 commands ---
        if func_name in _ESP32_CMD_MAP:
            cmd, fixed_data = _ESP32_CMD_MAP[func_name]
            data = fixed_data if fixed_data is not None else [int(a) for a in args if isinstance(a, (int, float))]
            self._esp32_log.debug("[Simulated] _write: {}".format(_fmt_esp(_build_esp32_frame(cmd, data))))
            return

        # --- DM CAN: linear movement ---
        if func_name in _MOVE_VEL_SIGNS:
            signs = _MOVE_VEL_SIGNS[func_name]
            speed = float(args[0]) if args and func_name != "stop" else 0.0
            for i, sign in enumerate(signs):
                w = (sign * speed) / WHEEL_RADIUS if sign != 0 else 0.0
                self._dm_log.debug("[Simulated] _write: {}".format(_fmt_dm(_build_dm_vel_frame(i + 1, w))))
            return

        # --- DM CAN: rotation ---
        if func_name == "turn_left":
            w_motor = (float(args[0]) if args else 0.0) * WHEEL_BASE_SUM / WHEEL_RADIUS
            for i in range(4):
                self._dm_log.debug("[Simulated] _write: {}".format(_fmt_dm(_build_dm_vel_frame(i + 1, w_motor))))
            return
        if func_name == "turn_right":
            w_motor = (float(args[0]) if args else 0.0) * WHEEL_BASE_SUM / WHEEL_RADIUS
            for i in range(4):
                self._dm_log.debug("[Simulated] _write: {}".format(_fmt_dm(_build_dm_vel_frame(i + 1, -w_motor))))
            return

        # --- DM CAN: read all motors ---
        if func_name in _MOTOR_READ_ALL:
            for i in range(4):
                self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_vel_frame(i + 1, 0.0))))
            return

        # --- DM CAN: read single motor ---
        if func_name in _MOTOR_READ_SINGLE:
            mid = int(args[0]) if args else 1
            self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_vel_frame(mid, 0.0))))
            return

        # --- DM CAN: enable / disable ---
        if func_name == "set_motor_enable":
            mid = int(args[0]) if args else 1
            state = int(args[1]) if len(args) > 1 else 1
            cmd = 0xFC if state else 0xFD
            ids = range(1, 5) if mid == 254 else [mid]
            for sid in ids:
                self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_cmd_frame(sid, cmd))))
            return

        # --- DM CAN: clear error (disable + enable) ---
        if func_name == "clear_motor_error":
            mid = int(args[0]) if args else 1
            ids = range(1, 5) if mid == 254 else [mid]
            for sid in ids:
                self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_cmd_frame(sid, 0xFD))))
            for sid in ids:
                self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_cmd_frame(sid, 0xFC))))
            return

        # --- DM CAN: set zero position ---
        if func_name == "set_motor_zero_position":
            mid = int(args[0]) if args else 1
            ids = range(1, 5) if mid == 254 else [mid]
            for sid in ids:
                self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_cmd_frame(sid, 0xFE))))
            return

        # --- DM CAN: set single motor velocity ---
        if func_name == "set_motor_velocity":
            mid = int(args[0]) if args else 1
            speed = float(args[1]) if len(args) > 1 else 0.0
            w = speed / WHEEL_RADIUS
            self._dm_log.debug("_write: {}".format(_fmt_dm(_build_dm_vel_frame(mid, w))))
            return

    def _log_read_frames(self, func_name, result):
        """Reconstruct and log protocol read frames for debug."""
        if func_name in _ESP32_CMD_MAP:
            cmd, _ = _ESP32_CMD_MAP[func_name]
            resp_data = []
            if isinstance(result, int):
                resp_data = [result & 0xFF]
            elif isinstance(result, float):
                resp_data = [int(result * 10) & 0xFF]
            elif isinstance(result, list):
                for x in result[:8]:
                    if isinstance(x, (int, float)):
                        resp_data.append(int(x) & 0xFF)
                    else:
                        resp_data.append(0)
            self._esp32_log.debug("_read : {}".format(_fmt_esp(_build_esp32_frame(cmd, resp_data))))
        elif func_name in _MOTOR_READ_ALL or func_name in _MOTOR_READ_SINGLE:
            if isinstance(result, list):
                self._dm_log.debug("_read : {}".format(result))
            else:
                self._dm_log.debug("_read : [{}]".format(result))

    def _rpc_call(self, func_name, *args):
        """Execute RPC call on socket server.

        Args:
            func_name (str): Target function name to execute.
            *args: Arguments payload to pass.

        Returns:
            any: Execution result returned from server or raises error.
        """
        # Clear server-side background logs before executing the target command to prevent debug pollution
        if self.debug and func_name not in ("get_last_serial_logs", "get_bluetooth_address", "get_wifi_account"):
            try:
                clear_payload = json.dumps({"func": "get_last_serial_logs", "args": []}) + "\n"
                self.sock.sendall(clear_payload.encode('utf-8'))
                clear_buffer = ""
                while "\n" not in clear_buffer:
                    clear_data = self.sock.recv(1024).decode('utf-8')
                    if not clear_data:
                        break
                    clear_buffer += clear_data
            except Exception:
                pass

        payload = json.dumps({"func": func_name, "args": args}) + "\n"

        try:
            self.sock.sendall(payload.encode('utf-8'))
        except Exception as e:
            raise e

        try:
            buffer = ""
            while "\n" not in buffer:
                data = self.sock.recv(1024).decode('utf-8')
                if not data:
                    raise socket.error("Connection closed by server")
                buffer += data

            response = json.loads(buffer.strip())
            result = response.get("result")

            # Do not fetch serial logs for purely OS-level functions to avoid printing background noise
            if self.debug and func_name not in ("get_last_serial_logs", "get_bluetooth_address", "get_wifi_account"):
                try:
                    log_payload = json.dumps({"func": "get_last_serial_logs", "args": []}) + "\n"
                    self.sock.sendall(log_payload.encode('utf-8'))
                    log_buffer = ""
                    while "\n" not in log_buffer:
                        log_data = self.sock.recv(1024).decode('utf-8')
                        if not log_data:
                            break
                        log_buffer += log_data
                    
                    log_response = json.loads(log_buffer.strip())
                    real_logs = log_response.get("result", {})
                    
                    if real_logs != -1 and isinstance(real_logs, dict):
                        esp32_logs = real_logs.get("esp32", {})
                        for category, log_lines in esp32_logs.items():
                            for log_line in log_lines:
                                if category == "write":
                                    self._esp32_log.debug("_write: " + log_line)
                                elif category == "read":
                                    self._esp32_log.debug("_read : " + log_line)
                                elif category == "error":
                                    self.log.error(log_line)
                            
                        dm_logs = real_logs.get("dm_can", {})
                        for category, log_lines in dm_logs.items():
                            for log_line in log_lines:
                                if category == "write":
                                    self._dm_log.debug("_write: " + log_line)
                                elif category == "read":
                                    self._dm_log.debug("_read : " + log_line)
                                elif category == "error":
                                    self.log.error(log_line)
                except Exception:
                    pass

            if "error" in response and response["error"] is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
                raise ValueError(response["error"])
            return result
        except socket.timeout as e:
            raise e
        except Exception as e:
            raise e

    def close(self):
        """Close the active TCP Socket connection."""
        MyAGVPlusSocket._instance_created = False
        self.sock.close()
        try:
            self._singleton_socket.close()
        except Exception:
            pass

    # ============== System & Version ==============

    def get_system_version(self):
        """Get the main firmware version of the robot controller.

        Returns:
            float: version number (e.g. 1.2), or -1 if failed.
        """
        return self._rpc_call("get_system_version")

    def get_modify_version(self):
        """Get the sub-firmware modification version.

        Returns:
            int: modification version number, or -1 if failed.
        """
        return self._rpc_call("get_modify_version")

    # ============== Debug ==============

    def set_debug_state(self, state):
        """Set the debug level of the bottom control board.

        Args:
            state (int): Debug mode bitmask.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, debug_state=state)
        return self._rpc_call("set_debug_state", state)

    def get_debug_state(self):
        """Get the current active debug state mask.

        Returns:
            int: debug mask state, or -1 if failed.
        """
        return self._rpc_call("get_debug_state")

    # ============== Power ==============

    def power_on(self):
        """Power on the robot relays, enable all motors and set control mode to velocity control.

        Returns:
            int: 1 if power on succeeded, -1 if failed.
        """
        return self._rpc_call("power_on")

    def power_off(self):
        """Disable all wheel motors and power off the relay switch.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        return self._rpc_call("power_off")

    def is_power_on(self):
        """Check whether the bottom relay power is active.

        Returns:
            int: 1 if powered on, 0 if offline, -1 if failed.
        """
        return self._rpc_call("is_power_on")

    # ============== Status ==============

    def get_robot_status(self):
        """Read robot status bits.

        Returns:
            list[int]: [battery_state, gyro_state, power_level], or -1 if failed.
        """
        return self._rpc_call("get_robot_status")

    def get_all_msg(self):
        """Read real-time battery voltage and raw gyroscope data packets.

        Returns:
            list: status values list, or -1 if failed.
        """
        return self._rpc_call("get_all_msg")

    # ============== Motion Control ==============

    def move_forward(self, speed):
        """Move the robot chassis forward.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._rpc_call("move_forward", speed)

    def move_backward(self, speed):
        """Move the robot chassis backward.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._rpc_call("move_backward", speed)

    def move_left_lateral(self, speed):
        """Move the robot chassis laterally to the left.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._rpc_call("move_left_lateral", speed)

    def move_right_lateral(self, speed):
        """Move the robot chassis laterally to the right.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._rpc_call("move_right_lateral", speed)

    def turn_left(self, angular_speed):
        """Turn the robot chassis left (counter-clockwise rotation).

        Args:
            angular_speed (float): chassis angular velocity in rad/s, range 0.01 ~ 7.27.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angular_speed=angular_speed)
            
        return self._rpc_call("turn_left", angular_speed)

    def turn_right(self, angular_speed):
        """Turn the robot chassis right (clockwise rotation).

        Args:
            angular_speed (float): chassis angular velocity in rad/s, range 0.01 ~ 7.27.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angular_speed=angular_speed)
            
        return self._rpc_call("turn_right", angular_speed)

    def stop(self):
        """Stop all motions immediately.

        Returns:
            int: 1 if dispatched, -1 if failed.
        """
        return self._rpc_call("stop")

    # ============== Auto Report ==============

    def set_auto_report_state(self, state):
        """Set ESP32 background auto report switch state.

        Args:
            state (int): 0 to disable, 1 to enable.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._rpc_call("set_auto_report_state", state)

    def get_auto_report_state(self):
        """Get ESP32 auto report state.

        Returns:
            int: 1 if active, 0 if disabled, -1 if failed.
        """
        return self._rpc_call("get_auto_report_state")

    def get_auto_report_message(self):
        """Read the latest captured automatic reporting data frames.

        Returns:
            list: parsed auto-report data when active,
            None if auto-report is disabled or no data has been received yet.
        """
        return self._rpc_call("get_auto_report_message")

    # ============== Motor Control ==============

    def set_motor_enable(self, motor_id, state):
        """Enable or disable target wheel motor torque.

        Args:
            motor_id (int): 1~4, or 254 (all motors).
            state (int): 1 to enable, 0 to disable.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, motor_id=motor_id, state=state)
        return self._rpc_call("set_motor_enable", motor_id, state)

    def get_motor_enable_status(self):
        """Read the enable state of all wheel motors.

        Returns:
            list[int]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] (1=enabled, 0=disabled), or -1 if not ready.
        """
        return self._rpc_call("get_motor_enable_status")

    def get_motor_status(self):
        """Read error codes for all wheel motors.

        Returns:
            list[int]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] error codes (0=normal), or -1 if not ready.
        """
        return self._rpc_call("get_motor_status")

    def clear_motor_error(self, motor_id):
        """Clear error codes of target motor by restarting and re-enabling it.

        Args:
            motor_id (int): 1~4 or 254.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, motor_id=motor_id)
        return self._rpc_call("clear_motor_error", motor_id)

    def get_motor_temps(self):
        """Read temperature sensors on wheel motor MOS controllers.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] MOS temperatures, or -1 if not ready.
        """
        return self._rpc_call("get_motor_temps")

    def get_motor_param_cache(self, motor_id, rid):
        """Read parameter value cache stored inside DM motor model structure.

        Args:
            motor_id (int): 1~4.
            rid (int): Parameter ID.

        Returns:
            float/int: cached parameter value.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, rid=rid)
        return self._rpc_call("get_motor_param_cache", motor_id, rid)


    def get_motor_velocity(self, motor_id):
        """Read real-time angular velocity of target motor.

        Args:
            motor_id (int): 1~4.

        Returns:
            float: angular speed (rad/s), or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        return self._rpc_call("get_motor_velocity", motor_id)

    def get_motor_torque(self, motor_id):
        """Read real-time feedback torque of target motor.

        Args:
            motor_id (int): 1~4.

        Returns:
            float: torque value (Nm), or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        res = self._rpc_call("get_motor_torque", motor_id)
        if isinstance(res, (int, float)):
            return float(round(res, 2))
        return res

    def get_motor_move_speeds(self):
        """Read real-time linear speed translation of all wheels over socket.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] linear speeds (m/s), or -1 if not ready.
        """
        res = self._rpc_call("get_motor_move_speeds")
        return res if res is not None else -1

    def get_motor_turn_speeds(self):
        """Read real-time angular speed translation (rotation) of all wheels over socket.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] angular speeds (rad/s), or -1 if not ready.
        """
        res = self._rpc_call("get_motor_turn_speeds")
        return res if res is not None else -1

    def get_motor_torques(self):
        """Read feedback torque values of all motors.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] torques (Nm), or -1 if not ready.
        """
        res = self._rpc_call("get_motor_torques")
        if isinstance(res, list):
            return [float(round(v, 2)) for v in res]
        return res

    def set_stall_protection(self, threshold=1.0):
        """Enable or disable active motor stall protection.
        
        Args:
            threshold (float): Torque limit (Nm). If any motor's torque exceeds this, 
                               the AGV will emergency stop. Set to 0 to disable.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, threshold=threshold)
        return self._rpc_call("set_stall_protection", threshold)





    # ============== LED ==============

    def set_led_color(self, brightness, color):
        """Set DIY LED strip brightness and RGB color values.

        Args:
            brightness (int): 0 ~ 255.
            color (tuple/list): [R, G, B] each 0 ~ 255.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, brightness=brightness, color=color)
        return self._rpc_call("set_led_color", brightness, color)

    def set_led_mode(self, mode):
        """Set active LED display mode.

        Args:
            mode (int): 0 for battery level display, 1 for DIY strip control.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._rpc_call("set_led_mode", mode)

    # ============== IO ==============

    def get_pin_input(self, pin):
        """Read voltage input levels from target IO pin ports.

        Args:
            pin (int): 0~6. 0 reads all 6 input pins.

        Returns:
            list[int]/int: pin states (255 mapped to -1), or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin=pin)
        return self._rpc_call("get_pin_input", pin)

    def set_pin_output(self, pin, state=0):
        """Write voltage output levels to target IO pin ports.

        Args:
            pin (int): 0~6.
            state (int): 0 for Low, 1 for High.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin=pin, state=state)
        return self._rpc_call("set_pin_output", pin, state)

    # ============== Fan ==============

    def set_fan_state(self, state=1):
        """Turn on or off internal chassis cooling fans.

        Args:
            state (int): 0 for Off, 1 for On.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._rpc_call("set_fan_state", state)

    # ============== Communication ==============

    def set_communication_state(self, state):
        """Toggle robot communication protocol channel state.

        Args:
            state (int): 0 for Serial, 1 for Socket, 2 for Bluetooth.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, communication_state=state)
        return self._rpc_call("set_communication_state", state)

    def get_communication_state(self):
        """Get active communication protocol channel mode.

        Returns:
            int: active mode index (0, 1, or 2), or -1 if failed.
        """
        return self._rpc_call("get_communication_state")

    # ============== Network & Bluetooth ==============

    def get_wifi_ip(self):
        """Get robot network adapter active local IP.

        Returns:
            str: IP address string.
        """
        return self._rpc_call("get_wifi_ip")

    def get_wifi_account(self):
        """Get connected router SSID account name info.

        Returns:
            str: SSID name string.
        """
        return self._rpc_call("get_wifi_account")

    def get_bluetooth_address(self):
        """Read adapter Bluetooth MAC address from hardware config logs.

        Returns:
            str: Bluetooth MAC address string.
        """
        return self._rpc_call("get_bluetooth_address")
