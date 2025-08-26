# coding=utf-8

import time
import threading
import socket

from pymycobot.close_loop import CloseLoop
from pymycobot.common import ProtocolCode,ProGripper


class Pro450Client(CloseLoop):
    def __init__(self, ip, netport=4500, debug=False):
        """
        Args:
            ip     : Server IP address
            netport : Socket port number, default is 4500
            debug    : whether show debug info
        """
        super(Pro450Client, self).__init__(debug)
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()
        self.is_stop = False
        self.sync_mode = False
        self.read_threading = threading.Thread(target=self.read_thread, args=("socket",))
        self.read_threading.daemon = True
        self.read_threading.start()

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock
        
    def _mesg(self, genre, *args, **kwargs):
        read_data = super(Pro450Client, self)._mesg(genre, *args, **kwargs)
        if read_data is None:
            return None
        elif read_data == 1:
            return 1
        elif read_data == -2:
            return -1
        valid_data, data_len = read_data
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
                return [self._decode_int8(valid_data[1:])]
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
        elif data_len == 6 or data_len == 32:
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
            ProtocolCode.GET_TOOL_MODIFY_VERSION
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
        if not (1 <= gripper_id <= 254):
            raise ValueError("The range of 'gripper_id' is 1 ~ 254, but the received value is {}".format(gripper_id))

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
            return (recv[3] << 8) | recv[4]
        return -1

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

    def flash_tool_firmware(self):
        """Burn tool firmware

        """
        return self._mesg(ProtocolCode.FLASH_TOOL_FIRMWARE)

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
        if not (1 <= target_id <= 254):
            raise ValueError("The range of 'target_id' is 1 ~ 254, but the received value is {}".format(target_id))
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
        if not (0 <= gripper_angle <= 100):
            raise ValueError("The range of 'gripper_angle' is 0 ~ 100, but the received value is {}".format(gripper_angle))
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
        if state not in [0, 1]:
            raise ValueError("The range of 'state' is 0 or 1, but the received value is {}".format(state))
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ENABLED, state)

    def set_pro_gripper_torque(self, gripper_torque, gripper_id=14):
        """ Set the gripper torque

        Args:
            gripper_torque (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        if not (0 <= gripper_torque <= 100):
            raise ValueError("The range of 'gripper_torque' is 0 ~ 100, but the received value is {}".format(gripper_torque))
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
        if not (1 <= speed <= 100):
            raise ValueError("The range of 'speed' is 1 ~ 100, but the received value is {}".format(speed))
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
        if not (0 <= gripper_angle <= 100):
            raise ValueError("The range of 'gripper_angle' is 0 ~ 100, but the received value is {}".format(gripper_angle))
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ABS_ANGLE, gripper_angle)

    def set_pro_gripper_io_open_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper IO open angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        if not (0 <= gripper_angle <= 100):
            raise ValueError("The range of 'gripper_angle' is 0 ~ 100, but the received value is {}".format(gripper_angle))
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
        if not (0 <= gripper_angle <= 100):
            raise ValueError("The range of 'gripper_angle' is 0 ~ 100, but the received value is {}".format(gripper_angle))
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
        if not (0 <= pressure_value <= 254):
            raise ValueError("The range of 'pressure_value' is 0 ~ 254, but the received value is {}".format(pressure_value))
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
        if not (100 <= current_value <= 300):
            raise ValueError("The range of 'current_value' is 100 ~ 300, but the received value is {}".format(current_value))
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
        if mode not in [0, 1]:
            raise ValueError("The range of 'mode' is 0 or 1, but the received value is {}".format(mode))
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