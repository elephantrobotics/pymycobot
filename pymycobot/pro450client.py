# coding=utf-8
import locale
import time
import threading
import socket

import numpy as np

from pymycobot.close_loop import CloseLoop
from pymycobot.robot_info import _interpret_status_code
from pymycobot.common import ProtocolCode,ProGripper


class Pro450Client(CloseLoop):
    def __init__(self, ip='192.168.0.232', netport=4500, debug=False):
        """
        Args:
            ip     : Server IP address, default '192.168.0.232'
            netport : Socket port number, default is 4500
            debug    : whether show debug info
        """
        super(Pro450Client, self).__init__(debug)
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

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock
        
    def _mesg(self, genre, *args, **kwargs):
        read_data = super(Pro450Client, self)._mesg(genre, *args, **kwargs)
        if read_data is None:
            return -1
        elif read_data == 1:
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
        if data_len in [8, 12, 14, 16, 26, 60]:
            if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1,n):
                    res.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                res = self.bytes4_to_int(valid_data)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    res.append(valid_data[i])
            elif data_len == 8 and genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
                res_list = [i for i in valid_data]
                return res_list
            else:
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i : header_i + 2]
                    res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            elif genre in [ProtocolCode.GET_ERROR_INFO]:
                return self._decode_int8(valid_data[1:])
            res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre == ProtocolCode.COBOTX_GET_ANGLE:
                res = self.bytes4_to_int(valid_data)
            for i in range(1,4):
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
                    i+=4
                else:
                    one = valid_data[i : i + 2]
                    res.append(self._decode_int16(one))
                    i+=2
        elif data_len == 30:
            i = 0
            res = []
            while i < 30:
                if i < 9 or i >= 23:
                    res.append(valid_data[i])
                    i+=1
                elif i < 23:
                    one = valid_data[i : i + 2]
                    res.append(self._decode_int16(one))
                    i+=2
        elif data_len == 38:
            i = 0
            res = []
            while i < data_len:
                if i < 10 or i >= 30:
                    res.append(valid_data[i])
                    i+=1
                elif i < 38:
                    one = valid_data[i : i + 2]
                    res.append(self._decode_int16(one))
                    i+=2
        # elif data_len == 56:
        #     for i in range(0, data_len, 8):
                
        #         byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
        #         res.append(byte_value)
        elif data_len in [6, 9, 32]:
            for i in valid_data:
                res.append(i)
        else:
            if genre in [
                ProtocolCode.GET_SERVO_VOLTAGES,
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.GET_SERVO_TEMPS,
            ]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i : i + 1])
                    res.append(0xFF & data1 if data1 < 0 else data1)
            res.append(self._decode_int8(valid_data))
        if res == []:
            return -1
        
        if genre in [
            ProtocolCode.ROBOT_VERSION,
            ProtocolCode.GET_ROBOT_ID,
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_CONTROLLER_CONNECTED,
            ProtocolCode.IS_PAUSED,  # TODO have bug: return b''
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
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_ANGLES]:
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
        elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, ProtocolCode.GET_POS_OVER]:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 32:
                parsed = res[:8]
                for start in (8, 20):
                    for i in range(start, start + 12, 2):
                        val = (res[i] << 8) | res[i + 1]
                        parsed.append(self._val_to_bits_list(val))

                return parsed
        elif genre == ProtocolCode.IS_INIT_CALIBRATION:
            if res == [1] * 7:
                return 1
            return res
        elif genre == ProtocolCode.GET_BASE_EXTERNAL_CONFIG:
            mode = res[0]
            baud_rate = int.from_bytes(res[1:5], byteorder="little", signed=False)
            timeout = int.from_bytes(res[5:9], byteorder="little", signed=False)
            return [mode, baud_rate, timeout]
        else:
            return res

    def _val_to_bits_list(self, val):
        if val == 0:
            return 0
        return [bit for bit in range(16) if (val >> bit) & 1]

    def _modbus_crc(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def _send_modbus_command(self, gripper_id, func_code, reg_addr, value_high=None, value_low=None, expect_len=7):
        """
        General Modbus command sending method

        Args:
            gripper_id: Device ID
            func_code: Function code (0x03 = read, 0x06 = write)
            reg_addr: Register address
            value_high: High byte of the write data (can be None for read operations)
            value_low: Low byte of the write data (can be None for read operations)
            expect_len: Expected return data length
        """
        cmd = [gripper_id, func_code, (reg_addr >> 8) & 0xFF, reg_addr & 0xFF]
        if func_code == 0x06:
            cmd.extend([value_high, value_low])
        else:
            cmd.extend([0x00, 0x00])

        cmd.extend(self._modbus_crc(cmd))
        recv = self.tool_serial_write_data(cmd)
        # time.sleep(0.05)
        # recv = []
        # for _ in range(3):
        #     recv = self.tool_serial_read_data(expect_len)
        #     if recv:
        #         break
        if not recv:
            return cmd, -1
        return cmd, recv

    def _check_gripper_id(self, gripper_id):

        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)

    def _write_and_check(self, gripper_id, reg_addr, value):
        """Write register and verify return"""
        self._check_gripper_id(gripper_id)
        high, low = (value >> 8) & 0xFF, value & 0xFF
        cmd, recv = self._send_modbus_command(gripper_id, 0x06, reg_addr, high, low)
        if isinstance(recv, (list, bytearray)) and len(recv) >= 6 and recv[1] == 0x06:
            if recv[4] == 0x00 and recv[5] == 0x01:
                return 1
            elif recv[4] == 0x00 and recv[5] == 0x00:
                return 0
            else:
                return -1
        return -1

    def _read_register(self, gripper_id, reg_addr):
        """Reads a register and returns an integer value"""
        self._check_gripper_id(gripper_id)
        _, recv = self._send_modbus_command(gripper_id, 0x03, reg_addr)
        if isinstance(recv, (list, bytearray)) and len(recv) >= 5 and recv[1] == 0x03:
            return (recv[4] << 8) | recv[5]
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

    def open(self):
        self.sock = self.connect_socket()
        
    def close(self):
        self.sock.close()

    def set_motor_enabled(self, joint_id, state):
        """Set the robot torque state.

        Args:
            joint_id: joint id 1-6, 254-all joints
            state: 1 - enable, 0 - disable
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, set_motor_enabled=joint_id, state=state)
        return self._mesg(ProtocolCode.SET_MOTOR_ENABLED, joint_id, state)

    def set_over_time(self, timeout=1000):
        """
        Set the timeout (unit: ms)
        Default is 1000ms (1 second)

        Args:
            timeout (int): Timeout period, in ms, range 0~65535
        """
        if not isinstance(timeout, int) or not (0 <= timeout <= 65535):
            raise ValueError("Timeout must be an integer between 0 and 65535 milliseconds.")

        high_byte = (timeout >> 8) & 0xFF
        low_byte = timeout & 0xFF

        return self._mesg(ProtocolCode.SET_OVER_TIME, high_byte, low_byte)

    def flash_tool_firmware(self, main_version, modified_version=0):
        """Burn tool firmware

        Args:
            main_version (str): Tool firmware version (format: 'x.y')
            modified_version (int): Tool firmware modified version, 0~255, defaults to 0

        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_main_version=main_version, tool_modified_version=modified_version)
        main_version = int(float(main_version) *10)
        return self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE, [main_version], modified_version)

    def get_comm_error_counts(self, joint_id):
        """Read the number of communication exceptions

        Args:
            joint_id (int): joint ID, 1 ~ 6
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, joint_id)

    def set_break(self, joint_id, value):
        """Set break point

        Args:
            joint_id: int. joint id 1 - 6
            value: int. 0 - disable, 1 - enable

        Return:
            0 : failed
            1 : success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, value=value)
        return self._mesg(ProtocolCode.SET_BREAK, joint_id, value)

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

    def set_base_io_output(self, pin_no, pin_signal):
        """Set the base output IO status

        Args:
            pin_no: pin port number. range 1 ~ 12
            pin_signal: 0 - low. 1 - high.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_no_base=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_base_io_input(self, pin_no):
        """Get the input IO status of the base

        Args:
            pin_no: (int) pin port number. range 1 ~ 12
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_no_base=pin_no)
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)

    def servo_restore(self, joint_id):
        """Abnormal recovery of joints

        Args:
            joint_id (int): Joint ID.
                arm : 1 ~ 6
                All joints: 254
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, servo_restore=joint_id
        )
        return self._mesg(ProtocolCode.SERVO_RESTORE, joint_id)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 6.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id)

    def send_angles(self, angles, speed, _async=False):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 6.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True, _async=_async)

    def send_angle(self, joint_id, angle, speed, _async=False):
        """Send one angle of joint to robot arm.

        Args:
            joint_id : Joint id(genre.Angle)， int 1-6.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, angle=angle, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, joint_id, [self._angle2int(angle)], speed, has_reply=True,
                          _async=_async)

    def send_coord(self, coord_id, coord, speed, _async=False):
        """Send one coord to robot arm.

        Args:
            coord_id (int): coord id, range 1 ~ 6
            coord (float): coord value.
                The coord range of `X` is -466 ~ 466.
                The coord range of `Y` is -466 ~ 466.
                The coord range of `Z` is -230 ~ 614.
                The coord range of `RX` is -180 ~ 180.
                The coord range of `RY` is -180 ~ 180.
                The coord range of `RZ` is -180 ~ 180.
            speed (int): 1 ~ 100
        """

        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, coord=coord, speed=speed)
        value = self._coord2int(
            coord) if coord_id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, coord_id, [value], speed, has_reply=True, _async=_async)

    def send_coords(self, coords, speed, _async=False):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]). len 6 [x, y, z, rx, ry, rz]
                The coord range of `X` is -466 ~ 466.
                The coord range of `Y` is -466 ~ 466.
                The coord range of `Z` is -230 ~ 614.
                The coord range of `RX` is -180 ~ 180.
                The coord range of `RY` is -180 ~ 180.
                The coord range of `RZ` is -180 ~ 180.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True, _async=_async)

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            joint_id: Joint id 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: Joint id 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id)

    def set_joint_max_angle(self, joint_id, degree):
        """Set the maximum angle of the joint (must not exceed the maximum angle specified for the joint)

        Args:
            joint_id (int): Joint id 1 - 6
            degree: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -120 ~ 120. The angle range of joint 3 is -158 ~ 158. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -165 ~ 165. The angle range of joint 6 is -175 ~ 175.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, degree=degree)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, joint_id, degree)

    def set_joint_min_angle(self, joint_id, degree):
        """Set the minimum angle of the joint (must not be less than the minimum angle specified by the joint)

        Args:
            joint_id (int): Joint id 1 - 6.
            degree: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -120 ~ 120. The angle range of joint 3 is -158 ~ 158. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -165 ~ 165. The angle range of joint 6 is -175 ~ 175.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, degree=degree)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, joint_id, degree)

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

    def jog_angle(self, joint_id, direction, speed, _async=True):
        """Jog control angle.

        Args:
            joint_id (int): Joint id 1 - 6.
            direction (int): 0 - decrease, 1 - increase
            speed (int): int range 1 - 100
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed, _async=_async, has_reply=True)

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

    def set_max_acc(self, mode, max_acc):
        """Set maximum acceleration

        Args:
            mode (int): 0 - angle acceleration. 1 - coord acceleration.
            max_acc (int): maximum acceleration value. Angular acceleration range is 1 ~ 200°/s. Coordinate acceleration range is 1 ~ 400mm/s
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode, max_acc=max_acc)
        return self._mesg(ProtocolCode.SET_MAX_ACC, mode, [max_acc])

    def jog_increment_angle(self, joint_id, increment, speed, _async=False):
        """Single angle incremental motion control.

        Args:
            joint_id: Joint id 1 - 6.
            increment: Angle increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, increment_angle=increment, speed=speed)
        scaled_increment = self._angle2int(increment)
        scaled_increment = max(min(scaled_increment, 32767), -32768)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [scaled_increment], speed, has_reply=True, _async=_async)

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
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed, has_reply=True, _async=_async)

    def set_communication_mode(self, communication_mode, protocol_mode=None):
        """Set communication mode
        Args:
            communication_mode (int):
                0 - socket communication mode
                1 - 485 communication mode
            protocol_mode (int, optional):
                0 - Custom protocol
                1 - Modbus protocol
                Default: None (means not specified)
        """
        if protocol_mode is not None:
            self.calibration_parameters(
                class_name=self.__class__.__name__, communication_mode=communication_mode, protocol_mode=protocol_mode)
            return self._mesg(ProtocolCode.SET_COMMUNICATION_MODE, communication_mode, protocol_mode)
        else:
            self.calibration_parameters(
                class_name=self.__class__.__name__, communication_mode=communication_mode)
            return self._mesg(ProtocolCode.SET_COMMUNICATION_MODE, communication_mode)

    def get_communication_mode(self):
        """Get communication mode
        Returns:
            communication_mode (int):
                0 - socket communication mode
                1 - 485 communication mode
            protocol_mode (int, optional):
                0 - Custom protocol
                1 - Modbus protocol
        """
        return self._mesg(ProtocolCode.GET_COMMUNICATION_MODE)

    def set_base_external_config(self, communicate_mode, baud_rate, timeout):
        """Bottom external device configuration

        Args:
            communicate_mode (int): 1 - 485. 2 - can
            baud_rate (int): Baud rate
            timeout (int): Timeout ms

        """
        self.calibration_parameters(class_name=self.__class__.__name__, communicate_mode=communicate_mode,
                                    baud_rate=baud_rate, timeout=timeout)
        data = bytearray()
        data += communicate_mode.to_bytes(1, 'big')
        data += baud_rate.to_bytes(4, 'big')
        data += timeout.to_bytes(4, 'big')
        return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONFIG, *data)

    def get_base_external_config(self):
        """Read the bottom external device configuration

        Returns:
            communicate_mode (int): 1 - 485. 2 - can
            baud_rate (int): Baud rate
            timeout (int): Timeout ms

        """
        return self._mesg(ProtocolCode.GET_BASE_EXTERNAL_CONFIG)

    def set_model_direction(self, joint_id, direction):
        """Set the direction of the robot model

        Args:
            joint_id (int): joint ID, 1 ~ 6.
            direction (int): 1 - forward, 0 - backward
        """
        return self._mesg(ProtocolCode.SET_MODEL_DIRECTION, joint_id, direction)

    def set_control_mode(self, mode=0):
        """Set robot motion mode

        Args:
            mode (int): 0 - location mode, 1 - torque mode

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_CONTROL_MODE, mode)

    def base_external_can_control(self, can_id, can_data):
        """Bottom external device can control

        Args:
            can_id (int): 1 - 4.
            can_data (list): The maximum length is 64

        """
        self.calibration_parameters(class_name=self.__class__.__name__, can_id=can_id, can_data=can_data)
        can_id_bytes = can_id.to_bytes(4, 'big')
        return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONTROL, *can_id_bytes, *can_data)

    def base_external_485_control(self, data):
        """Bottom external device 485 control

        Args:
            data (list): The maximum length is 64

        """
        self.calibration_parameters(class_name=self.__class__.__name__, data_485=data)
        return self._mesg(ProtocolCode.SET_BASE_EXTERNAL_CONTROL, *data)

    def get_error_information(self):
        """Obtaining robot error information

        Return:
            0: No error message.
            1 ~ 6: The corresponding joint exceeds the limit position.
            32-36: Coordinate motion error.
                32: No coordinate solution. Please check if the arm span is near the limit.
                33: No adjacent solution for linear motion.
                34: Velocity fusion error.
                35: No adjacent solution for null space motion.
                36: No solution for singular position. Please use joint control to leave the singular point.
        """
        return self._mesg(ProtocolCode.GET_ERROR_INFO)

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

    def parameter_identify(self):
        """Kinetic parameter identification"""
        return self._mesg(ProtocolCode.PARAMETER_IDENTIFY)

    def fourier_trajectories(self, trajectory):
        """Execute dynamic identification trajectory

        Args:
            trajectory (int): 0 ~ 1
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, trajectory=trajectory)
        return self._mesg(ProtocolCode.FOURIER_TRAJECTORIES, trajectory)

    def set_world_reference(self, coords):
        """Set the world coordinate system

        Args:
            coords: a list of coords value(List[float]). [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(class_name = self.__class__.__name__, world_coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, coord_list)

    def set_tool_reference(self, coords):
        """Set tool coordinate system

        Args:
            coords: a list of coords value(List[float])
        """
        self.calibration_parameters(class_name = self.__class__.__name__, tool_coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_TOOL_REFERENCE, coord_list)

    def go_home(self, speed=20, _async=False):
        """Control the machine to return to the zero position.

        Args:
            speed (int): 1 ~ 100
        Return:
            1 : All motors return to zero position.
            0 : failed.
        """
        return self.send_angles([0, 0, 0, 0, 0, 0], speed, _async=_async)

