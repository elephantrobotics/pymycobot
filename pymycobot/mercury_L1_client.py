# coding=utf-8
import locale
import socket
import threading
import time

import numpy as np

from pymycobot.L1_close_loop import L1CloseLoop
from pymycobot.common import ProtocolCode, ProGripper
from pymycobot.robot_info import _interpret_status_code, RobotStatusL1Info


class MercuryL1Client(L1CloseLoop):
    def __init__(self, ip='192.168.1.232', netport=6501, debug=False, save_serial_log=False):
        """
        Args:
            ip     : Server IP address, default '192.168.1.232'
            netport : Socket port number, default is 6501
            debug    : whether show debug info
        """
        super(MercuryL1Client, self).__init__(debug)
        self.save_serial_log = save_serial_log
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()
        self.is_stop = False
        self.sync_mode = True
        self.read_threading = threading.Thread(target=self.read_thread, args=("socket",))
        self.read_threading.daemon = True
        self.read_threading.start()
        self.language, _ = locale.getdefaultlocale()
        if self.language not in ["zh_CN", "en_US"]:
            self.language = "en_US"
        self.max_joint, self.min_joint = 0, 0
        self._joint_limits_initialized = False

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

    def _decode_l1_dual_servo_joints(self, genre, valid_data):
        """Parse first 34 bytes as 8 left + 9 right int16 fields (0xE1 / 0xE2 / 0xE4, firmware V1.0.2+)."""
        raw = bytes(valid_data[:34])
        left_pairs = [raw[i : i + 2] for i in range(0, 16, 2)]
        right_pairs = [raw[i : i + 2] for i in range(16, 34, 2)]
        if genre == ProtocolCode.GET_SERVO_STATUS:
            return [
                [int.from_bytes(p, "big", signed=False) for p in left_pairs],
                [int.from_bytes(p, "big", signed=False) for p in right_pairs],
            ]
        return [
            [self._int2angle(self._decode_int16(p)) for p in left_pairs],
            [self._int2angle(self._decode_int16(p)) for p in right_pairs],
        ]

    def _mesg(self, genre, *args, **kwargs):
        read_data = super(MercuryL1Client, self)._mesg(genre, *args, **kwargs)
        if read_data is None:
            return -1
        elif read_data == 1:
            if genre in [ProtocolCode.SET_TOOL_485_BAUD_RATE, ProtocolCode.SET_TOOL_SERIAL_TIMEOUT]:
                time.sleep(0.3)
                return 1
            elif genre in [ProtocolCode.SET_DIGITAL_OUTPUT]:
                time.sleep(0.02)
                return 1
            else:
                return 1
        elif read_data == -2:
            return 1
        elif read_data == 0:
            return read_data
        if isinstance(read_data, tuple):
            valid_data, data_len = read_data
        else:
            return -1

        res = []
        # print('data_len:', data_len, valid_data)
        if genre == ProtocolCode.GET_MOTORS_TEMPERATURE:
            if data_len == 34:
                data = list(valid_data[:34])
                left = data[0:7] + data[7:15]
                right = data[15:24] + data[24:33]
                return [left, right]
            if data_len == 14:
                return [int(x) for x in valid_data]
        elif (genre in (
                ProtocolCode.GET_SERVO_SPEED,
                ProtocolCode.GET_SERVO_CURRENTS,
                ProtocolCode.GET_SERVO_STATUS,
        ) and data_len >= 34):
            return self._decode_l1_dual_servo_joints(genre, valid_data)
        elif genre == ProtocolCode.PRO450_GET_DIGITAL_INPUTS:
            if data_len >= 4:
                return [int(valid_data[i]) for i in range(1, 5)]
        elif genre == ProtocolCode.GET_TOOL_485_BAUD_RATE_TIMEOUT:
            if data_len >= 7:
                return [
                        int.from_bytes(valid_data[1:5], "big", signed=False),
                        int.from_bytes(valid_data[5:7], "big", signed=False),
                    ]
        elif genre ==  ProtocolCode.GET_DIGITAL_INPUT:
            if data_len >= 2:
                return valid_data[1]
            return -1
        elif genre in (
            ProtocolCode.GET_CONTROL_MODE,
            ProtocolCode.GET_FRESH_MODE,
            ProtocolCode.IS_PAUSED,
            ProtocolCode.IS_MOVING,
            ProtocolCode.IS_FREE_MODE,
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_IN_POSITION,
        ):
            if data_len >= 2:
                return [valid_data[0], valid_data[1]]
            return -1
        elif genre == ProtocolCode.GET_DEBUG_LOG_MODE:
            if data_len >= 2:
                return [valid_data[0], valid_data[1]]
            if data_len >= 1:
                return valid_data[0]
            return -1
        elif genre in (ProtocolCode.GET_JOINT_MIN_ANGLE, ProtocolCode.GET_JOINT_MAX_ANGLE):
            for header_i in range(0, len(valid_data), 2):
                one = valid_data[header_i: header_i + 2]
                res.append(self._decode_int16(one))
            angles = [self._int2coord(angle) for angle in res]
            return angles

        elif genre in [ProtocolCode.GET_SPEED, ProtocolCode.GET_MAX_ACC] and data_len == 4:
            return [
                self._decode_int16(valid_data[0:2]),
                self._decode_int16(valid_data[2:4]),
            ]
        elif genre == ProtocolCode.GET_MODEL_DIRECTION:
            if data_len == 14:
                return [list(valid_data[0:7]), list(valid_data[7: ])]
            return -1
        elif data_len == 19 and genre == ProtocolCode.IS_INIT_CALIBRATION:
            if valid_data[0] == 1:
                return 1
            return [list(valid_data[1:10]), list(valid_data[10: ])]
        elif data_len in [8, 12, 14, 16, 26, 60]:
            if data_len == 8 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
                res_list = [i for i in valid_data]
                return res_list
            elif data_len == 8 and genre == ProtocolCode.GET_ENCODERS:
                return [
                        int.from_bytes(valid_data[0:4], "big", signed=False),
                        int.from_bytes(valid_data[4:8], "big", signed=False),
                    ]
            else:
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i: header_i + 2]
                    res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.GET_ROBOT_ID]:
                high, low = valid_data
                motor_type = (high << 8) | low  # 组合成 16 位整数
                return motor_type
            elif genre in [ProtocolCode.GET_ATOM_VERSION, ProtocolCode.GET_TOOL_MODIFY_VERSION]:
                res.append(self._decode_int8(valid_data[1:]))
            else:
                res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre == ProtocolCode.COBOTX_GET_ANGLE:
                for i in range(0, data_len, 2):
                    res.append(self._decode_int16(valid_data[i:i + 2]))
            elif genre == ProtocolCode.GET_ERROR_INFO:
                def parse_error(two_bytes):
                    high, low = two_bytes[0], two_bytes[1]

                    if high == 0xD0:
                        return low
                    else:
                        return 0

                if data_len != 4:
                    return -1

                left = parse_error(valid_data[0:2])
                right = parse_error(valid_data[2:4])

                return [left, right]
            else:
                for i in range(1, 4):
                    res.append(valid_data[i])
        elif data_len == 7:
            error_list = [i for i in valid_data]
            return error_list
        elif data_len in [34]:
            for i in range(0, data_len, 2):
                res.append(self._decode_int16(valid_data[i:i + 2]))
        elif data_len in [24] and genre == ProtocolCode.GET_COORDS:
            for i in range(0, data_len, 2):
                res.append(self._decode_int16(valid_data[i:i + 2]))
        elif data_len in [89]:
            for i in valid_data:
                res.append(i)

        elif data_len == 24:
            res = self._bytes4_to_int(valid_data)
        elif data_len == 40:
            i = 0
            while i < data_len:
                if i < 28:
                    res += self._bytes4_to_int(valid_data)
                    i += 4
                else:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        elif data_len == 30:
            i = 0
            res = []
            while i < 30:
                if i < 9 or i >= 23:
                    res.append(valid_data[i])
                    i += 1
                elif i < 23:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        elif data_len == 38:
            i = 0
            res = []
            while i < data_len:
                if i < 10 or i >= 30:
                    res.append(valid_data[i])
                    i += 1
                elif i < 38:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        # elif data_len == 56:
        #     for i in range(0, data_len, 8):

        #         byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
        #         res.append(byte_value)
        elif data_len in [6, 9, 32, 37]:
            for i in valid_data:
                res.append(i)

        elif data_len == 11 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
            res_list = [i for i in valid_data]
            return res_list
        elif data_len == 18 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
            res_list = [i for i in valid_data]
            return res_list
        else:
            if genre in [
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.GET_SERVO_TEMPS,
            ]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i: i + 1])
                    res.append(0xFF & data1 if data1 < 0 else data1)
            res.append(self._decode_int8(valid_data))
        if res == []:
            return -1

        if genre in [
            ProtocolCode.MODIFY_VERSION,
            ProtocolCode.GET_END_TYPE,
            ProtocolCode.GET_MOVEMENT_TYPE,
            ProtocolCode.GET_REFERENCE_FRAME,
            ProtocolCode.POWER_ON,
            ProtocolCode.POWER_OFF,
            ProtocolCode.STOP,
            ProtocolCode.SET_BREAK,
            ProtocolCode.GET_VR_MODE,
            ProtocolCode.GET_FILTER_LEN,
            ProtocolCode.GET_TOOL_MODIFY_VERSION,
            ProtocolCode.GET_FUSION_PARAMETERS,
            ProtocolCode.GET_COLLISION_MODE,
            ProtocolCode.GET_IDENTIFY_MODE,
            ProtocolCode.GET_FRESH_SPEED_MODE,
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_ANGLES]:
            angles = [self._int2angle(angle) for angle in res]
            left_angles = angles[:8]
            right_angles = angles[8:]
            return [left_angles, right_angles]
        elif genre == ProtocolCode.MERCURY_ERROR_COUNTS:
            left_info = res[:4]
            right_info = res[4:]
            return [left_info, right_info]
        elif genre == ProtocolCode.GET_ENCODERS:
            return [
                int.from_bytes(res[0:4], "big", signed=False),
                int.from_bytes(res[4:8], "big", signed=False),
            ],
        elif genre in [
            ProtocolCode.GET_COORDS,
            ProtocolCode.GET_TOOL_REFERENCE,
            ProtocolCode.GET_WORLD_REFERENCE,
        ]:
            if res:
                left_coords = []
                right_coords = []
                for idx in range(3):
                    left_coords.append(self._int2coord(res[:6][idx]))
                    right_coords.append(self._int2coord(res[6:][idx]))
                for idx in range(3, 6):
                    left_coords.append(self._int2angle(res[:6][idx]))
                    right_coords.append(self._int2angle(res[6:][idx]))
                return [left_coords, right_coords]
            else:
                return res
        elif genre in [ProtocolCode.SOLVE_INV_KINEMATICS]:
            if res == [-57295, -57295, -57295, -57295, -57295, -57295]:
                return 'No solution for conversion'
            return [self._int2angle(angle) for angle in res]
        elif genre in [ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
            return self._int2coord(self._process_single(res))
        elif genre in [ProtocolCode.COBOTX_GET_ANGLE]:
            return [self._int2angle(angle) for angle in res]
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 89:
                info = RobotStatusL1Info.error_info[self.language]
                output_msgs = []

                # LEFT ARM
                left = res[:42]

                left_crashed = left[0]
                left_moving = left[1]
                left_limits = left[2:10]
                left_motor = left[10:26]
                left_comm = left[26:42]

                # RIGHT ARM
                right = res[42:]

                right_crashed = right[0]
                right_moving = right[1]
                right_limits = right[2:11]
                right_motor = right[11:29]
                right_comm = right[29:47]

                parsed = []
                def parse_arm(name, crashed, moving, limits, motor, comm):
                    arm_parsed = []
                    arm_parsed.append(crashed)
                    arm_parsed.append(moving)
                    for i in limits:
                        arm_parsed.append(i)
                    # arm_parsed.append(limits)

                    # Motor Error
                    for i in range(len(motor) // 2):
                        val = (motor[i * 2] << 8) | motor[i * 2 + 1]
                        arm_parsed.append(self._val_to_bits_list(val))

                    # Communication Error
                    for i in range(len(comm) // 2):
                        val = (comm[i * 2] << 8) | comm[i * 2 + 1]
                        arm_parsed.append(self._val_to_bits_list(val))

                    if crashed == 1:
                        msg = f"{name} 碰撞检测触发" if self.language == "zh_CN" else f"{name} collision detected"
                        print(f"⚠️ {msg}")
                        output_msgs.append(msg)

                    if moving == 1:
                        msg = f"{name} 正在运动" if self.language == "zh_CN" else f"{name} is moving"
                        output_msgs.append(msg)

                    # Joint Limit
                    for i, val in enumerate(limits):
                        if val == 1:
                            msg = f"{name} J{i + 1} 超限位" if self.language == "zh_CN" else f"{name} J{i + 1} Limit Exceeded"
                            print(f"⚠️ {msg}")
                            output_msgs.append(msg)

                    # Motor Error
                    for i in range(len(motor) // 2):
                        val = (motor[i * 2] << 8) | motor[i * 2 + 1]

                        if val != 0:
                            msg = info["motor_error"].get(val, "未知错误" if self.language == "zh_CN" else "Unknown error")
                            print(f"{name} 电机错误: J{i + 1} - {msg}")
                            output_msgs.append(f"{name} J{i + 1} 电机异常: {msg}")

                    # Communication Error
                    for i in range(len(comm) // 2):
                        val = (comm[i * 2] << 8) | comm[i * 2 + 1]

                        if val != 0:
                            bits = [bit for bit in range(16) if (val >> bit) & 1]
                            for bit in bits:
                                msg = info["comm_error"].get(bit, "未知错误" if self.language == "zh_CN" else "Unknown error")
                                print(f"{name} 通信错误: J{i + 1} - {msg}")
                                output_msgs.append(f"{name} J{i + 1} 通信异常: {msg}")

                    return arm_parsed

                parsed_left  = parse_arm("左臂" if self.language == 'zh_CN' else 'Left arm', left_crashed, left_moving, left_limits, left_motor, left_comm)
                parsed_right = parse_arm("右臂" if self.language == 'zh_CN' else 'Right arm', right_crashed, right_moving, right_limits, right_motor, right_comm)

                parsed = [parsed_left, parsed_right]

                if not output_msgs:
                    msg = "机器人状态正常" if self.language == "zh_CN" else "Robot status is normal"
                    print(f"✅ {msg}")
                    output_msgs.append(msg)

                return parsed

        else:
            return res

    def _val_to_bits_list(self, val):
        if val == 0:
            return 0
        return [bit for bit in range(16) if (val >> bit) & 1]

    def _modbus_crc(self, data: bytes, mode='little') -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder=mode)

    def _joint_limit_init(self):
        mx = self.get_joint_max_angle()
        mn = self.get_joint_min_angle()

        if isinstance(mx, (list, tuple, np.ndarray)):
            max_vals = [float(v) for v in mx]
        else:
            max_vals = [float(mx)] * 10
        if isinstance(mn, (list, tuple, np.ndarray)):
            min_vals = [float(v) for v in mn]
        else:
            min_vals = [float(mn)] * 10

        if len(max_vals) < 10:
            max_vals.extend([max_vals[-1]] * (10 - len(max_vals)))
        if len(min_vals) < 10:
            min_vals.extend([min_vals[-1]] * (10 - len(min_vals)))

        # Protocol mapping:
        # 1-7: shared arm joints (used by both left/right arm chains)
        # 8: left waist joint
        # 9-10: right neck/head joints
        max_l = max_vals[:8]
        min_l = min_vals[:8]
        max_r = max_vals[:7] + max_vals[8:10]
        min_r = min_vals[:7] + min_vals[8:10]
        self.max_joint_left = max_l
        self.max_joint_right = max_r
        self.min_joint_left = min_l
        self.min_joint_right = min_r
        self.max_joint, self.min_joint = max_l, min_l
        self._joint_limits_initialized = True
        return max_l, min_l

    def _joint_limit_judge(self, angles):
        offset = 3

        try:
            left_angles, right_angles = angles

            for i in range(len(left_angles)):
                if not (self.min_joint_left[i] + offset < left_angles[i] < self.max_joint_left[i] - offset):
                    if self.language == "zh_CN":
                        return f"左臂关节{i + 1} 当前角度为{left_angles[i]}, 范围：{self.min_joint_left[i]} ~ {self.max_joint_left[i]}"
                    return f"Left joint {i + 1} = {left_angles[i]}, limit {self.min_joint_left[i]} ~ {self.max_joint_left[i]}"

            for i in range(len(right_angles)):
                if not (self.min_joint_right[i] + offset < right_angles[i] < self.max_joint_right[i] - offset):
                    if self.language == "zh_CN":
                        return f"右臂关节{i + 1} 当前角度为{right_angles[i]}, 范围：{self.min_joint_right[i]} ~ {self.max_joint_right[i]}"
                    return f"Right joint {i + 1} = {right_angles[i]}, limit {self.min_joint_right[i]} ~ {self.max_joint_right[i]}"

        except Exception as e:
            return f"joint limit error: {str(e)}"

        return ""

    def _Singularity(self, angles, arm_zh="", arm_en=""):
        try:
            if not isinstance(angles, (list, tuple)) or len(angles) < 6:
                return ""
            # Joint 6: 0 and 180 degrees are singular points
            singular_angles = [0, 180]
            state = ""
            offset = 5
            for singular in singular_angles:
                if singular - offset < angles[5] < singular + offset:
                    if self.language == "zh_CN":
                        prefix = f"{arm_zh}" if arm_zh else ""
                        return f"{prefix}在关节 6 处检测到奇点：{angles[5]} 度"
                    prefix = f"{arm_en} " if arm_en else ""
                    return f"{prefix}Singularity detected at joint 6: {angles[5]} degrees"
            return state
        except Exception:
            return "Singularity error"

    def _check_coords(self, new_coords, is_print=0, arm_zh="", arm_en=""):
        try:
            first_three = list(new_coords[:3])
            first_three[2] -= 83.64
            info = ""
            # Calculate the Euclidean norm (magnitude)
            magnitude = np.linalg.norm(first_three)
            if is_print == 1:
                if self.language == "zh_CN":
                    label = f"{arm_zh}" if arm_zh else ""
                    info += f"{label}当前臂展为{magnitude}, 最大的臂展为{self.arm_span}"
                else:
                    label = f"{arm_en} " if arm_en else ""
                    info += f"{label}Arm span is {magnitude}, max is {self.arm_span}"
            return info
        except Exception:
            return "check coords error"

    def _status_explain(self, status):
        error_info = _interpret_status_code(self.language, status)
        if error_info != "":
            self.arm_span = 440
        if 0x00 < status <= 0x07:
            angles = self.get_angles()
            if not self._joint_limits_initialized:
                self._joint_limit_init()
            error_info += self._joint_limit_judge(angles)
        elif status in [32, 33]:
            error_coords = self.get_coords()
            if (
                isinstance(error_coords, (list, tuple))
                and len(error_coords) == 2
                and isinstance(error_coords[0], (list, tuple))
            ):
                error_info += self._check_coords(error_coords[0], 1, arm_zh="左臂", arm_en="Left arm")
                error_info += self._check_coords(error_coords[1], 1, arm_zh="右臂", arm_en="Right arm")
            else:
                error_info += self._check_coords(error_coords, 1)
        elif status == 36:
            angles = self.get_angles()
            if (
                isinstance(angles, (list, tuple))
                and len(angles) == 2
                and isinstance(angles[0], (list, tuple))
            ):
                error_info += self._Singularity(angles[0], arm_zh="左臂", arm_en="Left arm")
                error_info += self._Singularity(angles[1], arm_zh="右臂", arm_en="Right arm")
            else:
                error_info += self._Singularity(angles)

        return error_info

    def _check_jog_allowed(self, arm_id=None):
        """JOG is only allowed in interpolation mode (fresh mode 0) for the affected arm(s).

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
        Returns:
            ``None`` if JOG is allowed. Otherwise a ``str`` for the public API to return to the user
        """
        fm = self.get_fresh_mode()
        if not isinstance(fm, (list, tuple)) or len(fm) < 2:
            return None
        left_m, right_m = fm[0], fm[1]
        bad = []
        if arm_id in (None, 0):
            if left_m == 1:
                bad.append("left")
            if right_m == 1:
                bad.append("right")
        elif arm_id == 1:
            if left_m == 1:
                bad.append("left")
        elif arm_id == 2:
            if right_m == 1:
                bad.append("right")
        if not bad:
            return None
        if self.language == "zh_CN":
            parts = []
            if "left" in bad:
                parts.append("左臂")
            if "right" in bad:
                parts.append("右臂")
            arms = "、".join(parts)
            return "错误：{}处于刷新模式，无法使用 JOG，请将该臂切换为插补模式。".format(arms)
        # Default / en_US: literal English copy (not translated from elsewhere).
        if len(bad) == 2:
            return (
                "Error: Left and right arms are in refresh mode. "
                "JOG is only allowed in interpolation mode; switch the affected arm(s) with set_fresh_mode(arm_id, 0)."
            )
        return "Error: {} arm is in refresh mode. JOG is only allowed in interpolation mode.".format(
            "Left" if bad[0] == "left" else "Right"
        )

    def open(self):
        self.sock = self.connect_socket()

    def close(self):
        self.sock.close()

    def set_motor_enabled(self, arm_id, joint_id, state):
        """Set the robot torque state.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id: joint id 1-7, 254-all joints
            state: 1 - enable, 0 - disable
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, set_motor_enabled=joint_id, state=state)
        return self._mesg(ProtocolCode.SET_MOTOR_ENABLED, arm_id, joint_id, state)

    def flash_tool_firmware(self, arm_id, main_version, modified_version=0, _async=False):
        """Burn arm tool firmware

        Args:
            arm_id (int):
                1 - left arm
                2 - right arm
            main_version (str): Tool firmware version (format: 'x.y')
            modified_version (int): Tool firmware modified version, 0~255, defaults to 0
        Returns:
            (str): firmware version

        """
        wait_time = 45
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, tool_main_version=main_version,
                                    tool_modified_version=modified_version)
        main_version = int(float(main_version) * 10)
        if _async:
            return self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE, arm_id, [main_version], modified_version)
        else:
            self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE, arm_id, [main_version], modified_version)
            self.log.info('Firmware burning in progress, expected to take 50 seconds, please wait patiently...')
            print('Firmware burning in progress, expected to take 50 seconds, please wait patiently...')

            time.sleep(wait_time)

            for _ in range(5):
                tool_main_version = self.get_atom_version(arm_id)
                tool_modify_version = self.get_tool_modify_version(arm_id)

                if tool_main_version != -1 and tool_modify_version != -1:
                    version_str = f"v{tool_main_version}.{tool_modify_version}"
                    msg = f"Current firmware version：{version_str}"
                    self.log.info(msg)
                    return msg

                time.sleep(1)
            self.log.info("Burning complete, but failed to read the end version number")
            print("⚠️ Burning complete, but failed to read the end version number")
            return -1

    def get_comm_error_counts(self, joint_id):
        """Read the number of communication exceptions

        Args:
            joint_id (int): joint ID, 1 ~ 7

        Returns: List [[left count], [right count]]. eg: [[0, 0, 0, 0], [0, 0, 0, 0]], left arm is [0, 0, 0, 0]:
            [0] - Joint Send Error Count
            [1] - Joint Read Error Count
            [2] - End-effector STM32F103 Send Error Count
            [3] - End-effector STM32F103 Read Error Count
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, joint_id)

    # def set_break(self, joint_id, value):
    #     """Set break point
    #
    #     Args:
    #         joint_id: int. joint id 1 - 6
    #         value: int. 0 - disable, 1 - enable
    #
    #     Return:
    #         0 : failed
    #         1 : success
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, joint_id=joint_id, value=value)
    #     return self._mesg(ProtocolCode.SET_BREAK, joint_id, value)

    def get_tool_modify_version(self, arm_id):
        """Read end correction version number

        Args:
            arm_id (int): 1 - left arm, 2 - right arm

        Returns: int
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id)
        return self._mesg(ProtocolCode.GET_TOOL_MODIFY_VERSION, arm_id)

    def set_fresh_mode(self, arm_id, mode):
        """Set command refresh / interpolation mode.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            mode (int):
                0 - interpolation (queue) mode
                1 - refresh mode (latest command first)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, mode=mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, arm_id, mode)

    def get_fresh_mode(self):
        """Query motion mode for both arms (single read, no parameters).

        Returns:
            list [left_mode, right_mode] — each 0 - interpolation, 1 - refresh.
        """
        return self._mesg(ProtocolCode.GET_FRESH_MODE, has_reply=True)

    def servo_restore(self, arm_id, joint_id):
        """Abnormal recovery of joints

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id (int): Joint ID.
                arm : 1 ~ 9
                All joints: 254
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, servo_restore=joint_id
        )
        return self._mesg(ProtocolCode.SERVO_RESTORE, arm_id, joint_id)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 9.

        Returns:
            (float) left and right angle, eg: [left, right]
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id)

    def set_debug_state(self, arm_id, log_state):
        """
        Set the debug log mode of the robot.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            log_state (int): Debug state as bitmask (0~7)
                0: No debug logs
                1: Only common debug log (_debug.log)
                2: Only motion-related log (_move.log)
                3: Common + motion-related logs (_debug.log+_move.log)
                4: Motor read/control frequency log (_clock_rate_debug.log)
                5: Common + Motor read/control frequency logs (_debug.log+_clock_rate_debug.log)
                6: Motion + Motor read/control frequency logs (_move.log+_clock_rate_debug.log)
                7: All logs

        Returns:
            int: 1-success, 0-failure, -1-error
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, log_state=log_state)
        return self._mesg(ProtocolCode.SET_DEBUG_LOG_MODE, arm_id, log_state)

    def get_debug_state(self):
        """
        Get the current debug log mode of the robot.

        Returns:
            list: ``[left_state, right_state]`` — each 
            - 0: no debug logs,
            - 1: only common debug log (_debug.log),
            - 2: only motion-related log (_move.log),
            - 3: common + motion-related logs (_debug.log+_move.log),
            - 4: motor read/control frequency log (_clock_rate_debug.log),
            - 5: common + Motor read/control frequency logs (_debug.log+_clock_rate_debug.log),
            - 6: motion + Motor read/control frequency logs (_move.log+_clock_rate_debug.log),
            - 7: all logs
        """
        return self._mesg(ProtocolCode.GET_DEBUG_LOG_MODE, has_reply=True)

    def jog_angle(self, arm_id, joint_id,  speed, l_direction=None, r_direction=None, _async=True):
        """Jog control angle.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id (int): Joint id 1 - 7.
            speed (int): int range 1 - 100
            l_direction (int): left arm direction, 0 - decrease, 1 - increase
            r_direction (int): right arm direction, 0 - decrease, 1 - increase
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, speed=speed)
        if arm_id == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id, left_direction=l_direction, right_direction=r_direction)
        elif arm_id == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id, left_direction=l_direction)
            r_direction = 0
        elif arm_id == 2:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id, right_direction=r_direction)
            l_direction = 0
        left_direction = l_direction
        right_direction = r_direction
        msg = self._check_jog_allowed(arm_id)
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_ANGLE, arm_id, joint_id, left_direction, right_direction, speed, _async=_async, has_reply=True)

    def jog_coord(self, arm_id, coord_id, speed, l_direction=None, r_direction=None, _async=True):
        """Jog control coord. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the jog_base_coord interface

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            coord_id (int): int 1-6
            speed (int): int range 1 - 100
            l_direction (int): left arm direction, 0 - decrease, 1 - increase
            r_direction (int): right arm direction, 0 - decrease, 1 - increase
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.

        Returns:
            1: End of the Movement

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, coord_id=coord_id, speed=speed)
        if arm_id == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, left_direction=l_direction, right_direction=r_direction)
        elif arm_id == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, left_direction=l_direction)
            r_direction = 0
        elif arm_id == 2:
            self.calibration_parameters(
                class_name=self.__class__.__name__, right_direction=r_direction)
            l_direction = 0
        left_direction = l_direction
        right_direction = r_direction
        msg = self._check_jog_allowed(arm_id)
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_COORD, arm_id, coord_id, left_direction, right_direction, speed, _async=_async, has_reply=True)

    # def jog_rpy(self, axis, direction, speed, _async=True):
    #     """Rotate the end point around the fixed axis of the base coordinate system
    #
    #     Args:
    #         axis (int): 1 ~ 3. 1 - Roll, 2 - Pitch, 3 - Yaw
    #         direction (int): 1 - Forward. 0 - Reverse.
    #         speed (int): 1 ~ 100.
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, axis=axis, direction=direction, speed=speed)
    #     msg = self._check_jog_allowed()
    #     if msg:
    #         return msg
    #     return self._mesg(ProtocolCode.JOG_RPY, axis, direction, speed, _async=_async, has_reply=True)

    def set_fusion_parameters(self, rank_mode, value):
        """Set speed fusion planning parameters
        Args:
            rank_mode: 0 ~ 4
                0: Restore default parameters (only available in set mode)
                1: Fusion joint velocity
                2: Fusion joint acceleration
                3: Fusion coordinate velocity
                4: Fusion coordinate acceleration
            value: 0 ~ 10000
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, rank_mode=rank_mode, rank_mode_value=value)
        return self._mesg(ProtocolCode.SET_FUSION_PARAMETERS, rank_mode, [value])

    def jog_increment_angle(self, arm_id, joint_id, speed, l_increment=None, r_increment=None, _async=False):
        """Single angle incremental motion control.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id: Joint id 1 - 7.
            speed: int (1 - 100)
            l_increment: left arm angle increment value
            r_increment: right arm angle increment value
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id,  speed=speed)
        if arm_id == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id,left_increment_angle=l_increment,
                right_increment_angle=r_increment)
        elif arm_id == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id, left_increment_angle=l_increment)
            r_increment = 0
        elif arm_id == 2:
            self.calibration_parameters(
                class_name=self.__class__.__name__, joint_id=joint_id, right_increment_angle=r_increment)
            l_increment = 0
        left_scaled_increment = self._angle2int(l_increment)
        left_scaled_increment = max(min(left_scaled_increment, 32767), -32768)
        right_scaled_increment = self._angle2int(r_increment)
        right_scaled_increment = max(min(right_scaled_increment, 32767), -32768)
        msg = self._check_jog_allowed(arm_id)
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_INCREMENT, arm_id, joint_id, [left_scaled_increment], [right_scaled_increment],
                          speed, has_reply=True, _async=_async)

    def jog_increment_coord(self, arm_id, coord_id, speed, l_increment=None, r_increment=None, _async=False):
        """Single coordinate incremental motion control.
        This interface is based on a single arm 1-axis coordinate system.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            coord_id: axis id 1 - 6.
            speed: int (1 - 100)
            l_increment: left arm coord increment value
            r_increment: right arm coord increment value

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, coord_id=coord_id, speed=speed)
        if arm_id == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, left_increment_coord=l_increment, right_increment_coord=r_increment)
        elif arm_id == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, left_increment_coord=l_increment)
            r_increment = 0
        elif arm_id == 2:
            self.calibration_parameters(
                class_name=self.__class__.__name__, right_increment_coord=r_increment)
            l_increment = 0

        if coord_id <= 3:
            left_value = self._coord2int(l_increment)
            right_value = self._coord2int(r_increment)
        else:
            left_scaled_increment = self._angle2int(l_increment)
            right_scaled_increment = self._angle2int(r_increment)
            left_value = max(min(left_scaled_increment, 32767), -32768)
            right_value = max(min(right_scaled_increment, 32767), -32768)
        msg = self._check_jog_allowed()
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, arm_id, coord_id, [left_value], [right_value],
                          speed, has_reply=True, _async=_async)

    def set_world_reference(self, coords):
        """Set the world coordinate system

        Args:
            coords: a list of coords value(List[float]). [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(class_name=self.__class__.__name__, world_coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, coord_list)

    def set_tool_reference(self, arm_id, coords):
        """Set tool coordinate system

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            coords: a list of coords value(List[float])
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, tool_coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_TOOL_REFERENCE, arm_id, coord_list)

    def go_home(self, arm_id=0, speed=20, _async=False):
        """Control the machine to return to the zero position.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            speed (int): 1 ~ 100
        Return:
            1 : All motors return to zero position.
            0 : failed.
        """
        return self.send_angles(arm_id, speed, [0] * 7, [0] * 7, _async=_async)

    def get_digital_inputs(self, arm_id):
        """Read end-effector digital inputs for **both** arms in one response (protocol 0x7B).

        Pin order per arm: IN1, IN2, end button 1, end button 2 (0/1).

        Returns:
            List [left, right], eg: [[IN1, IN2, btn1, btn2], [IN1, IN2, btn1, btn2]]
        """
        return self._mesg(ProtocolCode.PRO450_GET_DIGITAL_INPUTS, arm_id)

    # def set_torque_comp(self, joint_id, damping, comp_value=0):
    #     """Set joint torque compensation
    #
    #     Args:
    #         joint_id (int): joint ID， range 1 ~ 6
    #         damping (int): damping  0-close 1-open
    #         comp_value (int): Compensation value, range is 0 ~ 250, default is 0, The smaller the value, the harder it is to drag the joint
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, joint_id=joint_id, comp_value=comp_value, damping=damping)
    #     return self._mesg(ProtocolCode.SET_TORQUE_COMP, joint_id, comp_value, damping)
    #
    # def get_torque_comp(self):
    #     """Get joint torque compensation
    #     """
    #     return self._mesg(ProtocolCode.GET_TORQUE_COMP)

    def set_limit_switch(self, limit_mode, state):
        """Set the master switch for motion closed loop.

        Args:
            limit_mode (int): 1 - Location out of tolerance. 2 - Synchronous control
            state (int): 0 - close. 1 - open

                set_limit_switch(2, 0) indicates that the motion loop is closed.
                set_limit_switch(2, 1) indicates that the motion closed loop is opened.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, limit_mode=limit_mode, state=state)
        if limit_mode == 2 and state == 0:
            self.sync_mode = False
        elif limit_mode == 2 and state == 1:
            self.sync_mode = True

    def set_tool_serial_baud_rate(self, arm_id, baud_rate=115200):
        """ Set the end 485 baud rate

            Args:
                arm_id (int):
                    1 - left arm
                    2 - right arm
                baud_rate (int): Standard baud rates, such as 115200, 1000000, 57600, 19200, 9600, 4800.
                                defaults to 115200
            """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, end_485_baud_rate=baud_rate)
        data = bytearray()
        data += baud_rate.to_bytes(4, 'big')
        return self._mesg(ProtocolCode.SET_TOOL_485_BAUD_RATE, arm_id, *data)

    def set_tool_serial_timeout(self, arm_id, timeout=10000):
        """
        Set end 485 timeout (unit: ms)

        Args:
            arm_id (int):
                1 - left arm
                2 - right arm
            timeout (int): Timeout period, in ms, range 0 ~ 10000 ms, defaults to 10000
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, timeout=timeout)

        high_byte = (timeout >> 8) & 0xFF
        low_byte = timeout & 0xFF

        return self._mesg(ProtocolCode.SET_TOOL_SERIAL_TIMEOUT, arm_id, high_byte, low_byte)

    def get_tool_config(self, arm_id):
        """Read end-effector RS485 baud rate and timeout for both arms (protocol 0xBA).

        Args:
            arm_id (int):
            1 - left arm
            2 - right arm
        Returns:
            List [baud, timeout ms]
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id)
        return self._mesg(ProtocolCode.GET_TOOL_485_BAUD_RATE_TIMEOUT, arm_id)

    def set_free_move_mode(self, arm_id, mode):
        """ Set the free move mode

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            mode (int): 0 - close. 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, mode=mode)
        return self._mesg(ProtocolCode.SET_FREE_MODE, arm_id, mode)

    def get_free_move_mode(self):
        """ Set the free move mode

        Returns: List [left mode, right mode]. 0 - close. 1 - open
        """
        return self._mesg(ProtocolCode.IS_FREE_MODE)

    def get_motor_temps(self):
        """Read motor temperature.

       Returns: list[int] eg:[[Left Coil Temperature and MOS Tube Temperature], [Right Coil Temperature and MOS Tube Temperature]].
        """
        return self._mesg(ProtocolCode.GET_MOTORS_TEMPERATURE)

    # def set_fresh_speed_mode(self, mode):
    #     """Set refresh speed mode - fastest 286°/s, default 150°/s, will not save settings upon power failure.
    #     (Note: Interpolation mode cannot be used to set high-speed motion mode; it is only applicable to refresh mode.)
    #
    #     Args:
    #         mode: int.
    #             1 - High speed.
    #             0 - Low speed.
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
    #     if self.get_fresh_mode() != 1:
    #         if self.language == "en_US":
    #             return 'Warning: High-speed motion mode cannot be set in interpolation mode. Please switch to refresh mode.'
    #         else:
    #             return '警告：插补模式无法设置高速运动模式，请切换刷新模式使用'
    #     return self._mesg(ProtocolCode.SET_FRESH_SPEED_MODE, mode)
    #
    # def get_fresh_speed_mode(self):
    #     """Read refresh rate mode
    #
    #     Returns:
    #         0 - Low speed, 1 - High speed.
    #     """
    #     return self._mesg(ProtocolCode.GET_FRESH_SPEED_MODE, has_reply=True)
