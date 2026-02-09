# coding=utf-8
import locale
import time
import threading
import socket

import numpy as np
import serial
from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.L1_close_loop import L1CloseLoop
from pymycobot.robot_info import _interpret_status_code, RobotStatusL1Info
from pymycobot.common import ProtocolCode, ProGripper


class MercuryL1(L1CloseLoop):
    def __init__(self, ip='192.168.0.232', netport=6500, debug=False, save_serial_log=False):
        """
        Args:
            ip     : Server IP address, default '192.168.0.232'
            netport : Socket port number, default is 6500
            debug    : whether show debug info
        """
        super(MercuryL1, self).__init__(debug)
        self.save_serial_log = save_serial_log
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()
        self.is_stop = False
        self.sync_mode = True
        self.read_threading = threading.Thread(target=self.read_thread)
        self.read_threading.daemon = True
        self.read_threading.start()
        self.language, _ = locale.getdefaultlocale()
        if self.language not in ["zh_CN", "en_US"]:
            self.language = "en_US"
        self.max_joint, self.min_joint = 0, 0

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

    def _mesg(self, genre, *args, **kwargs):
        read_data = super(MercuryL1, self)._mesg(genre, *args, **kwargs)
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
        if genre == ProtocolCode.SET_BASE_EXTERNAL_CONTROL:
            res = [i for i in valid_data]
        elif data_len in [8, 12, 14, 16, 26, 60]:
            if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1, n):
                    res.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                res = self.bytes4_to_int(valid_data)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES,
                                             ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    res.append(valid_data[i])
            elif data_len == 8 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
                res_list = [i for i in valid_data]
                return res_list
            elif data_len == 14 and genre == ProtocolCode.GET_MOTORS_TEMPERATURE:
                res_list = [i for i in valid_data]
                return res_list
            else:
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i: header_i + 2]
                    res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            elif genre in [ProtocolCode.GET_ERROR_INFO]:
                return self._decode_int8(valid_data[1:])
            elif genre in [ProtocolCode.GET_ROBOT_ID]:
                high, low = valid_data
                motor_type = (high << 8) | low  # 组合成 16 位整数
                return motor_type
            res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre == ProtocolCode.COBOTX_GET_ANGLE:
                res = self.bytes4_to_int(valid_data)
            elif genre == ProtocolCode.PRO450_GET_DIGITAL_INPUTS:
                for i in range(4):
                    res.append(valid_data[i])
            else:
                for i in range(1, 4):
                    res.append(valid_data[i])
        elif data_len == 7:
            error_list = [i for i in valid_data]
            if genre == ProtocolCode.IS_INIT_CALIBRATION:
                res = error_list
            else:
                return error_list
            # for i in error_list:
            #     if i in range(16,23):
            #         res.append(1)
            #     elif i in range(23,29):
            #         res.append(2)
            #     elif i in range(32,112):
            #         res.append(3)
            #     else:
            #         res.append(i)
        elif data_len == 24:
            res = self.bytes4_to_int(valid_data)
        elif data_len == 40:
            i = 0
            while i < data_len:
                if i < 28:
                    res += self.bytes4_to_int(valid_data)
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

            if genre == ProtocolCode.GET_TOOL_485_BAUD_RATE_TIMEOUT:
                baud_rate = int.from_bytes(res[0:4], byteorder="big", signed=False)
                timeout = int.from_bytes(res[4:6], byteorder="big", signed=False)
                return [baud_rate, timeout]
        elif data_len == 11 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
            res_list = [i for i in valid_data]
            return res_list
        else:
            if genre in [
                ProtocolCode.GET_SERVO_VOLTAGES,
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
            ProtocolCode.ROBOT_VERSION,
            ProtocolCode.GET_ROBOT_ID,
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_CONTROLLER_CONNECTED,
            ProtocolCode.IS_PAUSED,
            ProtocolCode.IS_IN_POSITION,
            ProtocolCode.IS_MOVING,
            ProtocolCode.IS_SERVO_ENABLE,
            ProtocolCode.IS_ALL_SERVO_ENABLE,
            ProtocolCode.GET_SERVO_DATA,
            ProtocolCode.GET_DIGITAL_INPUT,
            ProtocolCode.GET_GRIPPER_VALUE,
            ProtocolCode.IS_GRIPPER_MOVING,
            ProtocolCode.GET_SPEED,
            ProtocolCode.GET_ENCODER,
            ProtocolCode.GET_BASIC_INPUT,
            ProtocolCode.GET_TOF_DISTANCE,
            ProtocolCode.GET_END_TYPE,
            ProtocolCode.GET_MOVEMENT_TYPE,
            ProtocolCode.GET_REFERENCE_FRAME,
            ProtocolCode.GET_FRESH_MODE,
            ProtocolCode.GET_GRIPPER_MODE,
            ProtocolCode.SET_SSID_PWD,
            ProtocolCode.GET_ERROR_DETECT_MODE,
            ProtocolCode.POWER_ON,
            ProtocolCode.POWER_OFF,
            ProtocolCode.RELEASE_ALL_SERVOS,
            ProtocolCode.RELEASE_SERVO,
            ProtocolCode.FOCUS_ALL_SERVOS,
            ProtocolCode.FOCUS_SERVO,
            ProtocolCode.STOP,
            ProtocolCode.SET_BREAK,
            ProtocolCode.IS_BTN_CLICKED,
            ProtocolCode.GET_CONTROL_MODE,
            ProtocolCode.GET_VR_MODE,
            ProtocolCode.GET_FILTER_LEN,
            ProtocolCode.IS_SERVO_ENABLE,
            ProtocolCode.GET_POS_SWITCH,
            ProtocolCode.GET_TOOL_MODIFY_VERSION,
            ProtocolCode.GET_FUSION_PARAMETERS,
            ProtocolCode.GET_MAX_ACC,
            ProtocolCode.GET_ERROR_INFO,
            ProtocolCode.GET_COLLISION_MODE,
            ProtocolCode.GET_IDENTIFY_MODE,
            ProtocolCode.GET_COMMUNICATION_MODE,
            ProtocolCode.IS_MOTOR_PAUSE,
            ProtocolCode.IS_FREE_MODE,
            ProtocolCode.GET_FRESH_SPEED_MODE,
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_ANGLES, ProtocolCode.GET_SERVO_SPEED]:
            return [self._int2angle(angle) for angle in res]
        elif genre in [
            ProtocolCode.GET_COORDS,
            ProtocolCode.MERCURY_GET_BASE_COORDS,
            ProtocolCode.GET_TOOL_REFERENCE,
            ProtocolCode.GET_WORLD_REFERENCE,
        ]:
            if res:
                r = []
                for idx in range(3):
                    r.append(self._int2coord(res[idx]))
                for idx in range(3, 6):
                    r.append(self._int2angle(res[idx]))
                return r
            else:
                return res
        elif genre in [ProtocolCode.GET_SERVO_VOLTAGES]:
            return [self._int2coord(angle) for angle in res]
        elif genre in [ProtocolCode.SOLVE_INV_KINEMATICS]:
            if res == [-57295, -57295, -57295, -57295, -57295, -57295]:
                return 'No solution for conversion'
            return [self._int2angle(angle) for angle in res]
        elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
            return self._int2coord(self._process_single(res))
        elif genre in [
            ProtocolCode.GET_JOINT_MAX_ANGLE,
            ProtocolCode.GET_JOINT_MIN_ANGLE,
        ]:
            return self._int2coord(res[0])
        elif genre == ProtocolCode.GET_ANGLES_COORDS:
            r = []
            for index in range(len(res)):
                if index < 7:
                    r.append(self._int2angle(res[index]))
                elif index < 10:
                    r.append(self._int2coord(res[index]))
                else:
                    r.append(self._int2angle(res[index]))
            return r
        elif genre == ProtocolCode.GO_ZERO:
            r = []
            if res:
                if 1 not in res[1:]:
                    return res[0]
                else:
                    for i in range(1, len(res)):
                        if res[i] == 1:
                            r.append(i)
            return r
        elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES,
                       ProtocolCode.GET_POS_OVER]:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 37:
                parsed = res[:9]
                for start in (9, 23):
                    for i in range(start, start + 14, 2):
                        val = (res[i] << 8) | res[i + 1]
                        parsed.append(self._val_to_bits_list(val))

                info = RobotStatusL1Info.error_info[self.language]
                output_msgs = []  # Record all erroneous text

                if res[0] == 1:
                    msg = '机器人发生碰撞检测' if self.language == "zh_CN" else 'Robot collision detected'
                    print(f"⚠️ {msg}")
                    output_msgs.append(msg)
                elif res[1] == 1:
                    msg = '机器人正在运动' if self.language == "zh_CN" else 'Robot is moving'
                    print(f"⚠️ {msg}")
                    output_msgs.append(msg)
                # Byte3-9: Joint over-limit
                for i, val in enumerate(res[2:9]):
                    if val == 1:
                        msg = info["joint_limit"][i]
                        print(f"⚠️ {msg}")
                        output_msgs.append(msg)

                # Byte10-22: Motor error
                for i in range(7):
                    high = res[9 + i * 2]
                    low = res[9 + i * 2 + 1]
                    val = (high << 8) | low
                    if val != 0:
                        msg = info["motor_error"].get(val, "未知错误" if self.language == "zh_CN" else "Unknown error")
                        print(
                            f"电机错误: 关节{i + 1} - {msg}" if self.language == "zh_CN" else f"Motor Error: Joint{i + 1} - {msg}")
                        output_msgs.append(f"J{i + 1} 电机异常: {msg}")

                # Byte23-36: Communication error
                for i in range(7):
                    high = res[23 + i * 2]
                    low = res[23 + i * 2 + 1]
                    val = (high << 8) | low
                    if val != 0:
                        bits = [bit for bit in range(16) if (val >> bit) & 1]
                        for bit in bits:
                            msg = info["comm_error"].get(bit,
                                                         "未知错误" if self.language == "zh_CN" else "Unknown error")
                            print(
                                f"通信错误: 关节{i + 1} - {msg}" if self.language == "zh_CN" else f"Communication Error: Joint{i + 1} - {msg}")
                            output_msgs.append(f"J{i + 1} 通信异常: {msg}")

                if not output_msgs:
                    msg = "机器人状态正常" if self.language == "zh_CN" else "Robot status is normal"
                    print(f"✅ {msg}")
                    output_msgs.append(msg)

                return parsed
        elif genre == ProtocolCode.IS_INIT_CALIBRATION:
            if res == [1] * 7:
                return 1
            return res
        elif genre == ProtocolCode.GET_BASE_EXTERNAL_CONFIG:
            mode = res[0]
            baud_rate = int.from_bytes(res[1:5], byteorder="big", signed=False)
            timeout = int.from_bytes(res[5:9], byteorder="big", signed=False)
            return [mode, baud_rate, timeout]
        elif genre == ProtocolCode.SET_BASE_EXTERNAL_CONTROL:
            mode = res[0]
            if mode == 1:
                return res
            elif mode == 2:
                can_id = (res[1] << 24) | (res[2] << 16) | (res[3] << 8) | res[4]
                return [mode, can_id] + res[5:]
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

    def _send_modbus_command(self, gripper_id, func_code, reg_addr, value_high=None, value_low=None, custom_mode=False):
        """
        General Modbus command sending method

        Args:
            gripper_id: Device ID
            func_code: Function code (0x03 = read, 0x06 = write)
            reg_addr: Register address
            value_high: High byte of the write data (can be None for read operations)
            value_low: Low byte of the write data (can be None for read operations)
        """
        # Base payload part shared by both modes
        payload = [gripper_id, func_code,
                   (reg_addr >> 8) & 0xFF, reg_addr & 0xFF]

        # Append value for write, or 0x00 0x00 for read
        if func_code == 0x06:
            payload.extend([value_high, value_low])
        else:
            payload.extend([0x00, 0x00])

        # Build full command based on mode
        if not custom_mode:
            # Modbus RTU classic format
            cmd = payload.copy()
            cmd.extend(self._modbus_crc(cmd))  # little-endian
        else:
            # Custom packet: FE FE LEN + payload + CRC(big-end)
            # LEN = payload length + CRC length (2)，即 6+2 = 8
            cmd = [0xFE, 0xFE, 0x08] + payload
            cmd.extend(self._modbus_crc(cmd, mode='big'))
        recv = self.tool_serial_write_data(cmd)
        if not recv:
            return cmd, -1
        return cmd, recv

    def _check_gripper_id(self, gripper_id):

        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)

    def _write_and_check(self, gripper_id, reg_addr, value, custom_mode=False):
        """Write register and verify response robustly (support calibration delay)"""
        self._check_gripper_id(gripper_id)
        high, low = (value >> 8) & 0xFF, value & 0xFF
        # Continuously read the response packets, and send a read command to trigger feedback each time.
        _, recv = self._send_modbus_command(gripper_id, 0x06, reg_addr, high, low, custom_mode)

        # Basic validity check
        if not isinstance(recv, (list, bytearray)) or len(recv) < 6:
            return -1

        # Two modes have different byte offsets
        # Modbus RTU standard: [id][cmd][regH][regL][valH][valL]...
        # Custom packet       : [fe][fe][len][id][cmd][regH][regL][valH][valL]...
        offset = 0 if not custom_mode else 3

        cmd_idx = 1 + offset  # command index
        reg_h_idx = 2 + offset  # register high
        reg_l_idx = 3 + offset  # register low
        val_h_idx = 4 + offset  # value high
        val_l_idx = 5 + offset  # value low

        # Verify command
        if recv[cmd_idx] != 0x06:
            return -1

        # Verify register address consistency
        if recv[reg_h_idx] != (reg_addr >> 8) & 0xFF or recv[reg_l_idx] != (reg_addr & 0xFF):
            return -1

        # Determine return status
        if recv[val_h_idx] == 0x00 and recv[val_l_idx] == 0x01:
            return 1

        return -1

    def _read_register(self, gripper_id, reg_addr):
        """Reads a register with command verification"""
        self._check_gripper_id(gripper_id)

        cmd, recv = self._send_modbus_command(gripper_id, 0x03, reg_addr)

        if isinstance(recv, (list, bytearray)) and len(recv) >= 6:
            recv_func = recv[1]
            recv_addr = (recv[2] << 8) | recv[3]
            if recv_func == 0x03 and recv_addr == reg_addr:
                return (recv[4] << 8) | recv[5]
            else:
                return -1

        return -1

    def _joint_limit_init(self):
        max_joint = np.zeros(6)
        min_joint = np.zeros(6)
        for i in range(6):
            max_joint[i] = self.get_joint_max_angle(i + 1)
            min_joint[i] = self.get_joint_min_angle(i + 1)
        return max_joint, min_joint

    def _joint_limit_judge(self, angles):
        offset = 3
        try:
            for i in range(6):
                if self.min_joint[i] + offset < angles[i] < self.max_joint[i] - offset:
                    pass
                else:
                    if self.language == "zh_CN":
                        return f"当前角度为{angles[i]}, 角度范围为： {self.min_joint[i]} ~ {self.max_joint[i]}"
                    return f"current value = {angles[i]}, limit is {self.min_joint[i]} ~ {self.max_joint[i]}"
        except TypeError:
            return "joint limit error"
        return "over limit error {}".format(angles)

    def _Singularity(self, angles):
        try:
            # Joint 6: 0 and 180 degrees are singular points
            singular_angles = [0, 180]
            state = ""
            offset = 5
            for singular in singular_angles:
                if singular - offset < angles[5] < singular + offset:
                    if self.language == "zh_CN":
                        return f"在关节 6 处检测到奇点：{angles[5]} 度"
                    return f"Singularity detected at joint 6: {angles[5]} degrees"
            return state
        except:
            return "Singularity error"

    def _check_coords(self, new_coords, is_print=0):
        try:
            first_three = new_coords[:3]
            first_three[2] -= 83.64
            info = ""
            # Calculate the Euclidean norm (magnitude)
            magnitude = np.linalg.norm(first_three)
            if is_print == 1:
                if self.language == "zh_CN":
                    info += f"当前臂展为{magnitude}, 最大的臂展为{self.arm_span}"
                else:
                    info += f"Arm span is {magnitude}, max is {self.arm_span}"

            # if magnitude > self.arm_span - 10:
            #     if self.language == "zh_CN":
            #         info += f"当前臂展为{magnitude}超出物理限位, 最大的臂展为{self.arm_span}"
            #     else:
            #         info += f"Arm span is {magnitude} exceeds physical limit, max is {self.arm_span}"
            return info
        except:
            return "check coords error"

    def _status_explain(self, status):
        error_info = _interpret_status_code(self.language, status)
        if error_info != "":
            self.arm_span = 440
        if 0x00 < status <= 0x07:
            angles = self.get_angles()
            if type(self.max_joint) == int and self.max_joint == 0:
                self.max_joint, self.min_joint = self._joint_limit_init()
            error_info += self._joint_limit_judge(angles)
        elif status in [32, 33]:
            error_coords = self.get_coords()
            error_info += self._check_coords(error_coords, 1)
        elif status == 36:
            angles = self.get_angles()
            error_info += self._Singularity(angles)

        return error_info

    def _check_jog_allowed(self):
        """Check whether jog motion is allowed based on fresh mode."""
        if self.get_fresh_mode() != 0:
            if self.language == "en_US":
                return 'Error: JOG motion cannot be used in refresh mode. Please switch to interpolation mode.'
            else:
                return '错误：刷新模式无法使用JOG运动，请切换插补模式使用'
        return None

    def open(self):
        self._serial_port.open()

    def close(self):
        self._serial_port.close()

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
        """Burn tool firmware

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            main_version (str): Tool firmware version (format: 'x.y')
            modified_version (int): Tool firmware modified version, 0~255, defaults to 0
        Returns:
            (str): firmware version

        """
        wait_time = 45
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, tool_main_version=main_version,
                                    tool_modified_version=modified_version)
        main_version = int(float(main_version) * 10)
        if _async:
            return self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE, arm_id, [main_version], modified_version)
        else:
            self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE, arm_id, [main_version], modified_version)

            print(f'Firmware burning in progress, expected to take 50 seconds, please wait patiently...')

            time.sleep(wait_time)

            for _ in range(5):
                tool_main_version = self.get_atom_version()
                tool_modify_version = self.get_tool_modify_version()

                if tool_main_version != -1 and tool_modify_version != -1:
                    version_str = f"v{tool_main_version}.{tool_modify_version}"
                    msg = f"Current firmware version：{version_str}"
                    return msg

                time.sleep(1)

            print("⚠️ Burning complete, but failed to read the end version number")
            return -1

    def get_comm_error_counts(self, arm_id, joint_id):
        """Read the number of communication exceptions

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id (int): joint ID, 1 ~ 7

        Returns:
             A list of length 4, such as [0, 0, 0, 0], represents:
               - `[0]`: Number of joint sending exceptions
               - `[1]`: Number of joint reading exceptions
               - `[2]`: Number of end-point sending exceptions
               - `[3]`: Number of end-point sending exceptions
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, joint_id=joint_id)
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, arm_id, joint_id)

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

    def get_tool_modify_version(self):
        """Read end correction version number
        """
        return self._mesg(ProtocolCode.GET_TOOL_MODIFY_VERSION)

    def get_pro_gripper_firmware_version(self, gripper_id=14):
        """ Read the firmware major and minor version numbers

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (float): x.x
        """
        val = self._read_register(gripper_id, ProGripper.MODBUS_GET_FIRMWARE_VERSION)
        return val / 10.0 if val >= 0 else -1

    def get_pro_gripper_firmware_modified_version(self, gripper_id=14):
        """ Read the firmware revision number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (int)
        """
        val = self._read_register(gripper_id, ProGripper.MODBUS_GET_FIRMWARE_MODIFY_VERSION)
        return val if val >= 0 else -1

    def set_pro_gripper_id(self, target_id, gripper_id=14):
        """ Set the gripper ID

        Args:
            target_id (int): Target ID, 1 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, target_id=target_id)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ID, target_id)

    def get_pro_gripper_id(self, gripper_id=14):
        """ Read the gripper ID

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_id (int): 1 ~ 254
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_ID)

    def set_pro_gripper_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ANGLE, gripper_angle)

    def get_pro_gripper_angle(self, gripper_id=14):
        """ Get the gripper angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_ANGLE)

    def set_pro_gripper_open(self, gripper_id=14):
        """ Open the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self.set_pro_gripper_angle(100, gripper_id)

    def set_pro_gripper_close(self, gripper_id=14):
        """ Close the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self.set_pro_gripper_angle(0, gripper_id)

    def set_pro_gripper_calibration(self, gripper_id=14):
        """ Set the gripper zero position

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_CALIBRATION, 0)

    def get_pro_gripper_status(self, gripper_id=14):
        """ Get the gripper status

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_STATUS)

    def set_pro_gripper_enabled(self, state, gripper_id=14):
        """ Set the gripper enable state

        Args:
            state (bool): 0 or 1, 0 - Disable 1 - Enable
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ENABLED, state)

    def set_pro_gripper_torque(self, gripper_torque, gripper_id=14):
        """ Set the gripper torque

        Args:
            gripper_torque (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_torque=gripper_torque)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_TORQUE, gripper_torque)

    def get_pro_gripper_torque(self, gripper_id=14):
        """ Set the gripper torque

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_torque (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_TORQUE)

    def set_pro_gripper_speed(self, speed, gripper_id=14):
        """ Set the gripper torque

        Args:
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_SPEED, speed)

    def get_pro_gripper_speed(self, gripper_id=14):
        """ Get the gripper speed

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            speed (int): 1 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_SPEED)

    def set_pro_gripper_abs_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper absolute angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ABS_ANGLE, gripper_angle)

    def set_pro_gripper_io_open_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper IO open angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_IO_OPEN_ANGLE, gripper_angle)

    def get_pro_gripper_io_open_angle(self, gripper_id=14):
        """ Get the gripper IO open angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_IO_OPEN_ANGLE)

    def set_pro_gripper_io_close_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper IO close angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_IO_CLOSE_ANGLE, gripper_angle)

    def get_pro_gripper_io_close_angle(self, gripper_id=14):
        """ Get the gripper IO close angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_IO_CLOSE_ANGLE)

    def set_pro_gripper_mini_pressure(self, pressure_value, gripper_id=14):
        """ Set the gripper mini pressure

        Args:
            pressure_value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pressure_value=pressure_value)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MINI_PRESSURE, pressure_value)

    def get_pro_gripper_mini_pressure(self, gripper_id=14):
        """ Get the gripper mini pressure

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            mini pressure (int): 0 ~ 254
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_MINI_PRESSURE)

    def set_pro_gripper_protection_current(self, current_value, gripper_id=14):
        """ Set the gripper protection current

        Args:
            current_value (int): 100 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, current_value=current_value)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_PROTECTION_CURRENT, current_value)

    def get_pro_gripper_protection_current(self, gripper_id=14):
        """ Get the gripper protection current

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            current_value (int): 100 ~ 300
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_PROTECTION_CURRENT)

    def set_fresh_mode(self, mode):
        """Set command refresh mode

        Args:
            mode: int.
                1 - Always execute the latest command first.
                0 - Execute instructions sequentially in the form of a queue.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, mode)

    def get_fresh_mode(self):
        """Query sports mode

        Returns:
            0 - interpolation mode, 1 - refresh mode
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
                arm : 1 ~ 7
                All joints: 254
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, servo_restore=joint_id
        )
        return self._mesg(ProtocolCode.SERVO_RESTORE, arm_id, joint_id)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 7.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id)

    def set_debug_state(self, log_state):
        """
        Set the debug log mode of the robot.

        Args:
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
        self.calibration_parameters(class_name=self.__class__.__name__, log_state=log_state)
        return self._mesg(ProtocolCode.SET_DEBUG_LOG_MODE, log_state)

    def get_debug_state(self):
        """
        Get the current debug log mode of the robot.

        Returns:
            int: Current debug state (0-7), or -1 if failed
                0: No debug logs
                1: Only common debug log (_debug.log)
                2: Only motion-related log (_move.log)
                3: Common + motion-related logs (_debug.log+_move.log)
                4: Motor read/control frequency log (_clock_rate_debug.log)
                5: Common + Motor read/control frequency logs (_debug.log+_clock_rate_debug.log)
                6: Motion + Motor read/control frequency logs (_move.log+_clock_rate_debug.log)
                7: All logs
        """
        return self._mesg(ProtocolCode.GET_DEBUG_LOG_MODE)

    def jog_angle(self, arm_id, joint_id, direction, speed, _async=True):
        """Jog control angle.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id (int): Joint id 1 - 9.
            direction (int): 0 - decrease, 1 - increase
            speed (int): int range 1 - 100
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, joint_id=joint_id, direction=direction, speed=speed)
        msg = self._check_jog_allowed()
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_ANGLE, arm_id, joint_id, direction, speed, _async=_async, has_reply=True)

    def jog_coord(self, arm_id, coord_id, direction, speed, _async=True):
        """Jog control coord. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the jog_base_coord interface

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            coord_id (int): int 1-6
            direction (int): 0 - decrease, 1 - increase
            speed (int): 1 - 100
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.

        Returns:
            1: End of the Movement

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, coord_id=coord_id, direction=direction, speed=speed)
        msg = self._check_jog_allowed()
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_COORD, arm_id, coord_id, direction, speed, _async=_async, has_reply=True)

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

    def jog_increment_angle(self, arm_id, joint_id, increment, speed, _async=False):
        """Single angle incremental motion control.

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            joint_id: Joint id 1 - 7.
            increment: Angle increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, arm_id=arm_id, joint_id=joint_id, increment_angle=increment, speed=speed)
        scaled_increment = self._angle2int(increment)
        scaled_increment = max(min(scaled_increment, 32767), -32768)
        msg = self._check_jog_allowed()
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [scaled_increment], speed, has_reply=True,
                          _async=_async)

    def jog_increment_coord(self, coord_id, increment, speed, _async=False):
        """Single coordinate incremental motion control.
        This interface is based on a single arm 1-axis coordinate system.

        Args:
            coord_id: axis id 1 - 6.
            increment: Coord increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, increment_coord=increment, speed=speed)
        if coord_id <= 3:
            value = self._coord2int(increment)
        else:
            scaled_increment = self._angle2int(increment)
            value = max(min(scaled_increment, 32767), -32768)
        msg = self._check_jog_allowed()
        if msg:
            return msg
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed, has_reply=True, _async=_async)

    # def set_communication_mode(self, protocol_mode=0):
    #     """Set Modbus communication mode
    #     Args:
    #         protocol_mode (int):
    #          0 - close modbus protocol, default
    #          1 - open modbus protocol
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, protocol_mode=protocol_mode)
    #     return self._mesg(ProtocolCode.SET_COMMUNICATION_MODE, protocol_mode)
    #
    # def get_communication_mode(self):
    #     """Get communication mode
    #     Returns:
    #         protocol_mode (int, optional):
    #             0 - Custom protocol
    #             1 - Modbus protocol
    #     """
    #     return self._mesg(ProtocolCode.GET_COMMUNICATION_MODE)

    # def set_base_external_config(self, communicate_mode, baud_rate, timeout):
    #     """Bottom external device configuration
    #
    #     Args:
    #         communicate_mode (int): 1 - 485. 2 - can
    #         baud_rate (int): Baud rate
    #         timeout (int): Timeout ms
    #
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, communicate_mode=communicate_mode,
    #                                 baud_rate=baud_rate, timeout=timeout)
    #     data = bytearray()
    #     data += communicate_mode.to_bytes(1, 'big')
    #     data += baud_rate.to_bytes(4, 'big')
    #     data += timeout.to_bytes(4, 'big')
    #     return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONFIG, *data)

    # def get_base_external_config(self):
    #     """Read the bottom external device configuration
    #
    #     Returns:
    #         communicate_mode (int): 1 - 485. 2 - can
    #         baud_rate (int): Baud rate
    #         timeout (int): Timeout ms
    #
    #     """
    #     return self._mesg(ProtocolCode.GET_BASE_EXTERNAL_CONFIG)

    # def base_external_can_control(self, can_id, can_data):
    #     """Bottom external device can control
    #
    #     Args:
    #         can_id (int): 1 - 65535.
    #         can_data (list): The maximum length is 64
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, can_id=can_id, can_data=can_data)
    #     can_id_bytes = can_id.to_bytes(4, 'big')
    #     return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONTROL, *can_id_bytes, *can_data)
    #
    # def base_external_485_control(self, data):
    #     """Bottom external device 485 control
    #
    #     Args:
    #         data (list): The maximum length is 64
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, data_485=data)
    #     return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONTROL, *data)

    def set_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot end.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR_PRO450, r, g, b)

    # def parameter_identify(self):
    #     """Kinetic parameter identification"""
    #     return self._mesg(ProtocolCode.PARAMETER_IDENTIFY)

    # def fourier_trajectories(self, trajectory, _async=False):
    #     """Execute dynamic identification trajectory
    #
    #     Args:
    #         trajectory (int): 0 ~ 1
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, trajectory=trajectory)
    #     # return self._mesg(ProtocolCode.FOURIER_TRAJECTORIES, trajectory)
    #     res = self._mesg(ProtocolCode.FOURIER_TRAJECTORIES, trajectory, _async=_async)
    #
    #     # 如果是异步模式，直接返回
    #     if _async:
    #         return res
    #
    #     stable_stop = 0
    #     # print("Fourier trajectory started, waiting for finish...")
    #
    #     while True:
    #         moving = self.is_moving()
    #         # print("moving:", moving)
    #
    #         if moving == 0:
    #             stable_stop += 1
    #             if stable_stop >= 2:  # 连续两次停止 → 执行完毕
    #                 return 0
    #         else:
    #             stable_stop = 0
    #
    #         time.sleep(0.01)

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
        return self.send_angles(arm_id, [0]*17, speed, _async=_async)

    def get_digital_inputs(self):
        """Read the status of all pins at the end,
        including: IN1, IN2, button 1 (right),
        and button 2 (button 2 is closer to the emergency stop, left).
        """
        return self._mesg(ProtocolCode.PRO450_GET_DIGITAL_INPUTS)

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

    def is_motor_pause(self):
        """Read motor pause status

        Return:
            1 : Paused, can be resumed using the resume() interface.
            0 : Not paused.
        """
        return self._mesg(ProtocolCode.IS_MOTOR_PAUSE)

    def set_pro_gripper_modbus(self, state, custom_mode=False, gripper_id=14):
        """ Set the gripper modbus mode

        Args:
            state (int): 0 or 1, 0 - close modbus 1 - open modbus
            custom_mode (bool):
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 -  failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        if custom_mode:
            return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MODE, state, custom_mode=custom_mode)
        else:
            return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MODE, state)

    def set_pro_gripper_baud(self, baud_rate=0, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            baud_rate (int): 0 ~ 5, defaults to 0 - 115200
                0 - 115200
                1 - 1000000
                2 - 57600
                3 - 19200
                4 - 9600
                5 - 4800
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_baud_rate=baud_rate)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_BAUD_RATE, baud_rate)

    def get_pro_gripper_baud(self, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            baud_rate (int): 0 ~ 5, defaults to 0 - 115200
                0 - 115200
                1 - 1000000
                2 - 57600
                3 - 19200
                4 - 9600
                5 - 4800
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_BAUD_RATE)

    def set_tool_serial_baud_rate(self, arm_id, baud_rate=115200):
        """ Set the end 485 baud rate

            Args:
                arm_id (int):
                    0 - left and right arm
                    1 - left arm
                    2 - right arm
                baud_rate (int): Standard baud rates, such as 115200, 1000000, 57600, 19200, 9600, 4800.
                                defaults to 115200
            """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, end_485_baud_rate=baud_rate)
        data = bytearray()
        data += baud_rate.to_bytes(4, 'big')
        return self._mesg(ProtocolCode.SET_TOOL_485_BAUD_RATE, arm_id, *data)

    def set_tool_serial_timeout(self, arm_id, timeout=10000):
        """
        Set end 485 timeout (unit: ms)

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
            timeout (int): Timeout period, in ms, range 0 ~ 10000 ms, defaults to 10000
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, timeout=timeout)

        high_byte = (timeout >> 8) & 0xFF
        low_byte = timeout & 0xFF

        return self._mesg(ProtocolCode.SET_TOOL_SERIAL_TIMEOUT, arm_id, high_byte, low_byte)

    def get_tool_config(self):
        """ Get the end 485 baud rate and timeout

            Returns: (list) [baud_rate, timeout]
            """
        return self._mesg(ProtocolCode.GET_TOOL_485_BAUD_RATE_TIMEOUT)

    def set_free_move_mode(self, arm_id, mode):
        """ Set the free move mode

        Args:
            arm_id (int):
                0 - left and right arm
                1 - left arm
                2 - right arm
        """
        self.calibration_parameters(class_name=self.__class__.__name__, arm_id=arm_id, mode=mode)
        return self._mesg(ProtocolCode.SET_FREE_MODE, arm_id, mode)

    def get_free_move_mode(self):
        """ Set the free move mode"""
        return self._mesg(ProtocolCode.IS_FREE_MODE)

    def set_pro_gripper_init(self, gripper_id=14):
        """
        Initialize the Pro450 gripper and automatically recover communication.

        This function automatically handles **four possible gripper states** and
        brings the device back to Modbus mode at **115200 baud**:

        1. Already in Modbus mode with baudrate = 115200
           - Angle reading works directly.

        2. Modbus mode but baudrate incorrect (e.g., 1,000,000)
           - Angle fails at 115200 but succeeds at alternative baudrate.

        3. Custom (non-Modbus) mode + correct baudrate
           - Baudrate is correct but Modbus commands fail.
           - Forcing Modbus mode succeeds.

        4. Custom mode + wrong baudrate
           - Requires scanning possible baudrates and forcing Modbus mode.

        After successful initialization:
          - Gripper is set to Modbus mode (mode = 1)
          - Baudrate is restored to 115200
          - Tool serial port baudrate is restored to 115200

        Args:
            gripper_id (int): Modbus ID of the gripper, range 1–254.
                Defaults to 14.

        Returns:
            bool: True if initialization succeeds, otherwise False.
        """

        try_bauds = [115200, 1000000]

        print("The gripper is initializing, please wait...")

        self.set_tool_serial_timeout(250)

        test = self.get_pro_gripper_angle(gripper_id=gripper_id)
        if test != -1:
            self.set_pro_gripper_modbus(1, gripper_id=gripper_id)  # ensure normal modbus mode

            self.set_pro_gripper_baud(0, gripper_id=gripper_id)  # gripper -> 115200

            self.set_tool_serial_baud_rate(115200)  # end -> 115200
            self.set_tool_serial_timeout(10000)

            print("Gripper Initialization Successful!")
            return True

        for baud in try_bauds:
            # print(f"\n👉 Try the end baud rate: {baud}")
            self.set_tool_serial_baud_rate(baud_rate=baud)

            cfg = self.get_tool_config()
            # print(f"   485 current config: {cfg}")

            # Read the angle again, applicable to: Baud rate = Correct, Mode = Modbus
            test = self.get_pro_gripper_angle(gripper_id=gripper_id)
            # print(f"🔁 Test Modbus to read angle return: {test}")

            if test != -1:
                self.set_pro_gripper_baud(0, gripper_id=gripper_id)
                self.set_tool_serial_baud_rate(115200)
                self.set_tool_serial_timeout(10000)
                print("Gripper Initialization Successful!")
                return True

            # print(f"\n👉 Try the end baud rate again: {baud}")
            self.set_tool_serial_baud_rate(baud_rate=baud)
            cfg = self.get_tool_config()
            # print(f"   485 current config: {cfg}")
            ret = self.set_pro_gripper_modbus(1, True, gripper_id=gripper_id)
            # print(f"   set_modbus(custom) ret={ret}")

            if ret == 1:
                for i in range(3):
                    t = self.get_pro_gripper_angle(gripper_id=gripper_id)
                    # print(f" 🔧 Test angle read[{i}] -> {t}")
                    if t != -1:
                        break
                    time.sleep(0.1)

                if t != -1:
                    self.set_pro_gripper_baud(0, gripper_id=gripper_id)  # change gripper → 115200

                    self.set_tool_serial_baud_rate(115200)  # end back → 115200
                    self.set_tool_serial_timeout(10000)

                    print("Gripper Initialization Successful!")
                    return True

        print("Gripper Initialization Failed!")
        return False

    # def set_motor_type(self, motor_type):
    #     """Set motor type.
    #
    #     Args:
    #         motor_type (hex/int/str): motor type, can be 0xA1C2, 0xA3C0, or 'A1C2', 'a3c0'
    #     """
    #
    #     self.calibration_parameters(class_name=self.__class__.__name__, motor_type=motor_type)
    #
    #     if isinstance(motor_type, str):
    #         motor_type = int(motor_type, 16)
    #
    #     high_byte = (motor_type >> 8) & 0xFF  # 0xA3
    #     low_byte = motor_type & 0xFF  # 0xC0
    #
    #     return self._mesg(ProtocolCode.PRO450_SET_MOTOR_TYPE, high_byte, low_byte)

    def get_motor_temps(self):
        """Read motor temperature

        Return: A list, bits 1-7 represent coil temperature, bits 8-14 represent MOSFET temperature.
        """
        return self._mesg(ProtocolCode.GET_MOTORS_TEMPERATURE)

    def set_fresh_speed_mode(self, mode):
        """Set refresh speed mode - fastest 286°/s, default 150°/s, will not save settings upon power failure.
        (Note: Interpolation mode cannot be used to set high-speed motion mode; it is only applicable to refresh mode.)

        Args:
            mode: int.
                1 - High speed.
                0 - Low speed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        if self.get_fresh_mode() != 1:
            if self.language == "en_US":
                return 'Warning: High-speed motion mode cannot be set in interpolation mode. Please switch to refresh mode.'
            else:
                return '警告：插补模式无法设置高速运动模式，请切换刷新模式使用'
        return self._mesg(ProtocolCode.SET_FRESH_SPEED_MODE, mode)

    def get_fresh_speed_mode(self):
        """Read refresh rate mode

        Returns:
            0 - Low speed, 1 - High speed.
        """
        return self._mesg(ProtocolCode.GET_FRESH_SPEED_MODE, has_reply=True)