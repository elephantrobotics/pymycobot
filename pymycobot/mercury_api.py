
# coding=utf-8

import time
import struct

from pymycobot.error import calibration_parameters
from pymycobot.common import DataProcessor, ProtocolCode, write, read

class MercuryCommandGenerator(DataProcessor):
    _write = write
    _read = read
    def __init__(self, debug=False):
        super(MercuryCommandGenerator, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.is_stop = False
        self.write_command = []
        self.read_command = []
        self.send_jog_command = False
        self.sync_mode = True

    def _mesg(self, genre, *args, **kwargs):
        """

        Args:
            genre: command type (Command)
            *args: other data.
                   It is converted to octal by default.
                   If the data needs to be encapsulated into hexadecimal,
                   the array is used to include them. (Data cannot be nested)
            **kwargs: support `has_reply`
                has_reply: Whether there is a return value to accept.
        """
        real_command, has_reply = super(
            MercuryCommandGenerator, self)._mesg(genre, *args, **kwargs)
        is_in_position = False
        is_get_return = False
        lost_times = 0
        with self.lock:
            self.write_command.append(genre)
            if self.__class__.__name__ == "Mercury":
                self._write(self._flatten(real_command))
            elif self.__class__.__name__ == "MercurySocket":
                self._write(self._flatten(real_command), method="socket")
        t = time.time()
        wait_time = 0.15
        if genre == ProtocolCode.POWER_ON:
            wait_time = 8
        elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.RELEASE_ALL_SERVOS, ProtocolCode.FOCUS_ALL_SERVOS,
                       ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP, ProtocolCode.SET_CONTROL_MODE, ProtocolCode.MERCURY_DRAG_TEACH_CLEAN]:
            wait_time = 3
        elif genre in [
                ProtocolCode.SEND_ANGLE,
                ProtocolCode.SEND_ANGLES,
                ProtocolCode.SEND_COORD,
                ProtocolCode.SEND_COORDS,
                ProtocolCode.JOG_ANGLE,
                ProtocolCode.JOG_COORD,
                ProtocolCode.JOG_INCREMENT,
                ProtocolCode.JOG_INCREMENT_COORD,
                ProtocolCode.COBOTX_SET_SOLUTION_ANGLES,
                ProtocolCode.MERCURY_SET_BASE_COORDS,
                ProtocolCode.MERCURY_JOG_BASE_COORD,
                ProtocolCode.MERCURY_SET_BASE_COORD,
                ProtocolCode.OVER_LIMIT_RETURN_ZERO,
                ProtocolCode.JOG_BASE_INCREMENT_COORD,
                ProtocolCode.WRITE_MOVE_C,
                ProtocolCode.JOG_RPY] and self.sync_mode:
            wait_time = 300
            is_in_position = True
        elif genre in [ProtocolCode.SERVO_RESTORE]:
            wait_time = 0.3
        need_break = False
        data = None
        while True and time.time() - t < wait_time:
            # print("--------------", time.time() - t)
            for v in self.read_command:

                # v == b'\xfe\xfe\x04[\x01\r\x87'
                if is_in_position and v[2] == 0x04 and v[3] == 0x5b:
                    # print(-1)
                    # print("到位反馈", flush=True)
                    is_get_return = True
                    need_break = True
                    data = v
                    with self.lock:
                        self.read_command.remove(v)
                        self.write_command.remove(genre)

                elif genre == v[3] and v[2] == 5 and v[4] == 0xFF:
                    # 通信闭环
                    # print(-2)
                    # print("闭环", flush=True)
                    is_get_return = True
                    with self.lock:
                        self.read_command.remove(v)
                    if has_reply == False:
                        # print(-3)
                        # print("仅闭环退出", flush=True)
                        need_break = True
                        data = v
                elif genre == v[3]:
                    # print(-4)
                    # print("正常读取", flush=True)
                    need_break = True
                    data = v
                    with self.lock:
                        self.read_command.remove(v)
                        self.write_command.remove(genre)
                    break

            if is_in_position and time.time() - t > 0.2 and is_get_return == False:
                # 运动指令丢失，重发
                # print("运动指令丢失，重发", flush=True)
                lost_times += 1
                with self.lock:
                    self.write_command.append(genre)
            if need_break:
                # print("退出", flush=True)
                break
            if lost_times > 2:
                # 重传3次失败，返回-1
                # print("重传3次失败，返回-1", flush=True)
                return -1
            if t < self.is_stop and genre != ProtocolCode.STOP:
                # 打断除了stop指令外的其他指令的等待
                self.is_stop = 0
                break
            time.sleep(0.001)
        else:
            # print("---超时---")
            pass
        if data is None:
            # print("未拿到数据")
            return data
        res = []
        data = bytearray(data)
        data_len = data[2] - 3
        # unique_data = [ProtocolCode.GET_BASIC_INPUT,
        #                ProtocolCode.GET_DIGITAL_INPUT]
        if genre in [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.IS_SERVO_ENABLE]:
            # 指令后面多一位ID位
            data_pos = 5
            data_len -= 1
        else:
            data_pos = 4
        if is_get_return:
            data_len -= 1
            data_pos += 1
            if data[2] == 5:
                # print("握手成功")

                return data[5]
            elif data[2] == 4:
                # print("到位或者失败反馈")
                return data[4]
        # print("111 4: ",data_pos, data_len, genre)
        valid_data = data[data_pos: data_pos + data_len]
        if data_len in [6, 8, 12, 14, 16, 24, 26, 60]:
            if (data_len == 8 or data_len == 9)and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1, n):
                    res.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                i = 0
                while i < data_len:
                    byte_value = int.from_bytes(
                        valid_data[i:i+4], byteorder='big', signed=True)
                    i += 4
                    res.append(byte_value)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    res.append(valid_data[i])
            else:
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i: header_i + 2]
                    res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            elif genre in [ProtocolCode.GET_LIMIT_SWITCH]:
                for i in valid_data:
                    res.append(i)
            elif genre in [ProtocolCode.GET_ERROR_INFO]:
                res = self._decode_int16(valid_data)
                if res != 0:
                    res -= 53248
                return res
            else:
                res.append(self._decode_int16(valid_data))
        elif data_len == 3:
            res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.GET_ENCODER]:
                byte_value = int.from_bytes(
                    valid_data, byteorder='big', signed=True)
                res.append(byte_value)
            else:
                for i in range(1, 4):
                    res.append(valid_data[i])
        elif data_len == 7:
            # if genre ==ProtocolCode.GET_TORQUE_COMP:
            for i in range(0, data_len):
                res.append(valid_data[i])
            # else:
            #     error_list = [i for i in valid_data]
            #     for i in error_list:
            #         if i in range(16, 23):
            #             res.append(1)
            #         elif i in range(23, 29):
            #             res.append(2)
            #         elif i in range(32, 112):
            #             res.append(3)
            #         else:
            #             res.append(i)
        elif data_len == 28:
            for i in range(0, data_len, 4):
                byte_value = int.from_bytes(
                    valid_data[i:i+4], byteorder='big', signed=True)
                res.append(byte_value)
        elif data_len == 40 and genre == ProtocolCode.GET_ANGLES_COORDS:
            i = 0
            while i < data_len:
                if i < 28:
                    byte_value = int.from_bytes(
                        valid_data[i:i+4], byteorder='big', signed=True)
                    res.append(byte_value)
                    i += 4
                else:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        elif data_len == 40 and genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            # 右臂上位机错误
            i = 0
            res = []
            while i < data_len:
                if i < 10 or i >= 30:
                    res.append(valid_data[i])
                    i += 1
                elif i < 30:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
        elif data_len == 37:
            i = 0
            res = []
            while i < data_len:
                if i < 9:
                    res.append(valid_data[i])
                    i += 1
                else:
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
        elif data_len == 56:
            for i in range(0, data_len, 8):
                byte_value_send = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
                byte_value_current = int.from_bytes(valid_data[i+4:i+4], byteorder='big', signed=True)
                res.append([byte_value_send, byte_value_current])
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
            return None

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
            # ProtocolCode.GET_ERROR_DETECT_MODE,
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
            ProtocolCode.MERCURY_ERROR_COUNTS,
            ProtocolCode.GET_MAX_ACC,
            ProtocolCode.GET_MONITOR_MODE,
            ProtocolCode.GET_COLLISION_MODE
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_DRAG_FIFO]:
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
        elif genre in [ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, ProtocolCode.MERCURY_GET_POS_OVER_SHOOT, ProtocolCode.GET_SERVO_CW]:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.GET_ANGLES:
            return [self._int3angle(angle) for angle in res]
        elif genre == ProtocolCode.COBOTX_GET_ANGLE:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 23:
                i = 9
                for i in range(9, len(res)):
                    if res[i] != 0:
                        data = bin(res[i])[2:]
                        res[i] = []
                        while len(data) != 16:
                            data = "0"+data
                        for j in range(16):
                            if data[j] != "0":
                                res[i].append(15-j)
                return res
            else:
                for i in range(10, len(res)):
                    if res[i] != 0:
                        data = bin(res[i])[2:]
                        res[i] = []
                        while len(data) != 16:
                            data = "0"+data
                        for j in range(16):
                            if data[j] != "0":
                                res[i].append(15-j)
                return res
        else:
            return res

    def _process_received(self, data):
        if not data:
            return []

        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        header_i, header_j = 0, 1
        while header_j < data_len - 4:
            if self._is_frame_header(data, header_i, header_j):
                cmd_id = data[header_i + 3]
                if cmd_id in self.write_command or cmd_id == 0x5B:
                    return data
            header_i += 1
            header_j += 1
        else:
            return []

    def read_thread(self, method=None):
        while True:
            datas = b""
            data_len = -1
            k = 0
            pre = 0
            t = time.time()
            wait_time = 0.15
            if method is not None:
                try:
                    self.sock.settimeout(wait_time)
                    data = self.sock.recv(1024)
                    if isinstance(data, str):
                        datas = bytearray()
                        for i in data:
                            datas += hex(ord(i))
                except:
                    data = b""
                if self.check_python_version() == 2:
                    command_log = ""
                    for d in data:
                        command_log += hex(ord(d))[2:] + " "
                    self.log.debug("_read : {}".format(command_log))
                    # self.log.debug("_read: {}".format([hex(ord(d)) for d in data]))
                else:
                    command_log = ""
                    for d in data:
                        command_log += hex(d)[2:] + " "
                    self.log.debug("_read : {}".format(command_log))
                if data:
                    res = self._process_received(data)
                    with self.lock:
                        self.read_command.append(res)
            else:
                while True and time.time() - t < wait_time:
                    # print("r", end=" ", flush=True)
                    if self._serial_port.inWaiting() > 0:
                        data = self._serial_port.read()
                        k += 1
                        # print(datas, flush=True)
                        if data_len == 3:
                            datas += data
                            crc = self._serial_port.read(2)
                            if self.crc_check(datas) == [v for v in crc]:
                                datas += crc
                                break
                        if data_len == 1 and data == b"\xfa":
                            datas += data
                            # if [i for i in datas] == command:
                            #     datas = b''
                            #     data_len = -1
                            #     k = 0
                            #     pre = 0
                            #     continue
                            # break
                        elif len(datas) == 2:
                            data_len = struct.unpack("b", data)[0]
                            datas += data
                        elif len(datas) > 2 and data_len > 0:
                            datas += data
                            data_len -= 1
                        elif data == b"\xfe":
                            if datas == b"":
                                datas += data
                                pre = k
                            else:
                                if k - 1 == pre:
                                    datas += data
                                else:
                                    datas = b"\xfe"
                                    pre = k
                    # else:
                    #     print("no data", flush=True)
                else:
                    datas = b''
                if datas:
                    # print("read:", datas)
                    if self.send_jog_command and datas[3] == 0x5b:
                        self.send_jog_command = False
                        continue
                    res = self._process_received(datas)
                    if self.check_python_version() == 2:
                        command_log = ""
                        for d in datas:
                            command_log += hex(ord(d))[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                    else:
                        command_log = ""
                        for d in datas:
                            command_log += hex(d)[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                    if datas[3] == 0x5D:
                        debug_data = []
                        for i in range(4, 32, 4):
                            byte_value = int.from_bytes(datas[i:i+4], byteorder='big', signed=True)
                            debug_data.append(byte_value)
                        self.log.debug("_read : {}".format(debug_data))
                        continue
                    if res == []:
                        continue
                    with self.lock:
                        self.read_command.append(res)
                # return datas

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION)

    def get_atom_version(self):
        """Get atom firmware version.

        Returns:
            float: version number.
        """
        return self._mesg(ProtocolCode.GET_ATOM_VERSION)

    def is_power_on(self):
        """Adjust robot arm status

        Return:
            1 - power on
            0 - power off
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_POWER_ON)

    def get_coords(self, angles=None):
        """Get the coords from robot arm, coordinate system based on base. The target angle can be passed in, and the coordinate position corresponding to the target angle can be obtained.
        
        Args:
            angles (list): The angles of six joints. e.g. [0, 0, 0, 0, 0, 0]

        Return:
            list : A float list of coord . [x, y, z, rx, ry, rz].

        """
        if angles is None:
            return self._mesg(ProtocolCode.GET_COORDS)
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.GET_COORDS, angles)

    def is_paused(self):
        """Judge whether the manipulator pauses or not.

        Return:
            1 - paused
            0 - not paused
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_PAUSED)

    def set_solution_angles(self, angle, speed):
        """Set zero space deflection angle value

        Args:
            angle: Angle of joint 1. The angle range is -90 ~ 90
            speed: 1 - 100.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, speed=speed, solution_angle=angle
        )
        return self._mesg(
            ProtocolCode.COBOTX_SET_SOLUTION_ANGLES, [self._angle2int(angle)], speed, has_reply=True
        )

    def get_solution_angles(self):
        """Get zero space deflection angle value"""
        return self._mesg(ProtocolCode.COBOTX_GET_SOLUTION_ANGLES)

    def write_move_c(self, transpoint, endpoint, speed):
        """Circular trajectory motion

        Args:
            transpoint (list): Arc passing point coordinates
            endpoint (list): Arc end point coordinates
            speed (int): 1 ~ 100
        """
        start = []
        end = []
        for index in range(6):
            if index < 3:
                start.append(self._coord2int(transpoint[index]))
                end.append(self._coord2int(endpoint[index]))
            else:
                start.append(self._angle2int(transpoint[index]))
                end.append(self._angle2int(endpoint[index]))
        return self._mesg(ProtocolCode.WRITE_MOVE_C, start, end, speed, has_reply=True)

    def focus_all_servos(self):
        """Lock all joints"""
        return self._mesg(ProtocolCode.FOCUS_ALL_SERVOS)

    def go_home(self, robot, speed=20):
        """Control the machine to return to the zero position.

        Args:
            robot (int): 
                1 - left arm
                2 - right arm
            speed (int): 1 ~ 100
        Return:
            1 : All motors return to zero position.
            0 : failed.
        """
        if robot == 1:
            return self.send_angles([0, 0, 0, 0, 90, 0], speed)
        else:
            self.send_angle(11, 0, speed)
            self.send_angle(12, 0, speed)
            return self.send_angles([0, 0, 0, 0, 90, 0], speed)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 7 or 11 ~ 13.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id)

    def get_angles(self):
        """ Get the angle of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES)
    

    def servo_restore(self, joint_id):
        """Abnormal recovery of joints

        Args:
            joint_id (int): Joint ID.
                arm : 1 ~ 7 
                waist : 13
                All joints: 254
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, servo_restore=joint_id
        )
        return self._mesg(ProtocolCode.SERVO_RESTORE, joint_id)

    # def set_error_detect_mode(self, mode):
    #     """Set error detection mode. Turn off without saving, default to open state

    #     Return:
    #         mode : 0 - close 1 - open.
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, mode=mode
    #     )
    #     self._mesg(ProtocolCode.SET_ERROR_DETECT_MODE, mode)

    # def get_error_detect_mode(self):
    #     """Set error detection mode"""
    #     return self._mesg(ProtocolCode.GET_ERROR_DETECT_MODE, has_reply=True)

    def sync_send_angles(self, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached

        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f == 1:
                return 1
            time.sleep(0.1)
        return 0

    def get_base_coords(self):
        """get base coords"""
        return self._mesg(ProtocolCode.MERCURY_GET_BASE_COORDS)

    def send_base_coord(self, axis, coord, speed):
        """Single coordinate control with the torso base as the coordinate system

        Args:
            axis (int): 1 to 6 correspond to x, y, z, rx, ry, rz
            coord (float): coord value.
                The coord range of `X` is -351.11 ~ 566.92.
                The coord range of `Y` is -645.91 ~ 272.12.
                The coord range of `Y` is -262.91 ~ 655.13.
                The coord range of `RX` is -180 ~ 180.
                The coord range of `RY` is -180 ~ 180.
                The coord range of `RZ` is -180 ~ 180.
            speed (int): 1 ~ 100
        """
        if axis < 4:
            coord = self._coord2int(coord)
        else:
            coord = self._angle2int(coord)
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORD, axis, [coord], speed)

    def send_base_coords(self, coords, speed):
        """Full coordinate control

        Args:
            coords (list): coordinate value, [x, y, z, rx, ry, rz]
            speed (int): 1 ~ 100
        """
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORDS, coord_list, speed)

    def jog_base_coord(self, axis, direction, speed, sync=True):
        """Single-coordinate unidirectional motion control

        Args:
            axis (int): 1 ~ 6 correspond to [x, y, z, rx, ry, rz]
            direction (int): Direction of movement. 0 or 1.
            speed (int): 1 ~ 100.
            sync (bool): Whether to wait for the movement to complete. Default to True.
            
        Return:
            0: End of exercise.
            1 ~ 7: Corresponding joint exceeds limit.
            16 ~ 19: Collision protection.
            32 ~ 35: Coordinates have no adjacent solutions.
            49: Not powered on.
            50: Motor abnormality.
            51: Motor encoder error
            52: Not reaching the designated location or not reaching the designated location for more than 5 minutes (only J11, J12 available)
        """
        if sync:
            return self._mesg(ProtocolCode.MERCURY_JOG_BASE_COORD, axis, direction, speed, has_reply=True)
        self.send_jog_command = True
        return self._mesg(ProtocolCode.MERCURY_JOG_BASE_COORD, axis, direction, speed)

    def drag_teach_save(self):
        """Start recording the dragging teaching point. In order to show the best sports effect, the recording time should not exceed 90 seconds."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_SAVE)

    def drag_teach_execute(self):
        """Start dragging the teaching point and only execute it once."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE)

    def drag_teach_pause(self):
        """Pause recording of dragging teaching point"""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_PAUSE)

    # def is_gripper_moving(self, mode=None):
    #     """Judge whether the gripper is moving or not

    #     Args:
    #         mode: 1 - pro gripper(default)  2 - Parallel gripper

    #     Returns:
    #         0 - not moving
    #         1 - is moving
    #         -1- error data
    #     """
    #     if mode:
    #         return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, mode, has_reply=True)
    #     return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    def set_gripper_enabled(self, value):
        """Pro adaptive gripper enable setting

        Args:
            value (int): 
                1 : enable
                0 : release
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, value=value)
        return self._mesg(ProtocolCode.SET_GRIPPER_ENABLED, value)

    def is_btn_clicked(self):
        """Check if the end button has been pressed.

        Return:
            1 : pressed.
            0 : not pressed.
        """
        return self._mesg(ProtocolCode.IS_BTN_CLICKED)

    # def tool_serial_restore(self):
    #     """485 factory reset
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_RESTORE)

    # def tool_serial_ready(self):
    #     """Set up 485 communication

    #     Return:
    #         0 : not set
    #         1 : Setup completed
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_READY, has_reply=True)

    # def tool_serial_available(self):
    #     """Read 485 buffer length

    #     Return:
    #         485 buffer length available for reading
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_AVAILABLE, has_reply=True)

    # def tool_serial_read_data(self, data_len):
    #     """Read fixed length data. Before reading, read the buffer length first. After reading, the data will be cleared

    #     Args:
    #         data_len (int): The number of bytes to be read, range 1 ~ 45
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, data_len=data_len)
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_READ_DATA, data_len, has_reply=True)

    # def tool_serial_write_data(self, command):
    #     """End 485 sends data， Data length range is 1 ~ 45 bytes

    #     Args:
    #         command : data instructions

    #     Return:
    #         number of bytes received
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_WRITE_DATA, command, has_reply=True)

    # def tool_serial_flush(self):
    #     """Clear 485 buffer
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_FLUSH)

    # def tool_serial_peek(self):
    #     """View the first data in the buffer, the data will not be cleared

    #     Return:
    #         1 byte data
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_PEEK, has_reply=True)

    # def tool_serial_set_baud(self, baud=115200):
    #     """Set 485 baud rate, default 115200

    #     Args:
    #         baud (int): baud rate
    #     """
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_SET_BAUD, baud)

    # def tool_serial_set_timeout(self, max_time):
    #     """Set 485 timeout in milliseconds, default 30ms

    #     Args:
    #         max_time (int): timeout
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, max_time=max_time)
    #     return self._mesg(ProtocolCode.TOOL_SERIAL_SET_TIME_OUT, max_time)

    def get_robot_status(self):
        return self._mesg(ProtocolCode.MERCURY_ROBOT_STATUS)

    def power_on(self):
        """Power on the robot
        
        Return:
            1: success
            2: failed
        """
        res = self._mesg(ProtocolCode.POWER_ON)
        if res == 1:
            self.get_limit_switch()
        time.sleep(1)
        return res

    def power_off(self):
        """Robot power outage"""
        with self.lock:
            self.read_command.clear()
        return self._mesg(ProtocolCode.POWER_OFF)

    def release_all_servos(self):
        """Relax all joints
        """
        return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int. joint id 1 - 7
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: int. joint id 1 - 7
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

    def get_robot_type(self):
        """Get robot type
        """
        return self._mesg(ProtocolCode.GET_ROBOT_ID)

    def get_zero_pos(self):
        """Read the zero encoder value

        Returns:
            list: The values of the zero encoders of the seven joints
        """
        return self._mesg(ProtocolCode.GET_ZERO_POS)

    def is_init_calibration(self):
        """Check if the robot is initialized for calibration

        Returns:
            bool: True if the robot is initialized for calibration, False otherwise
        """
        return self._mesg(ProtocolCode.IS_INIT_CALIBRATION)

    def set_break(self, joint_id, value):
        """Set break point

        Args:
            joint_id: int. joint id 1 - 7
            value: int. 0 - disable, 1 - enable

        Return:
            0 : failed
            1 : success 
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id, value=value)
        return self._mesg(ProtocolCode.SET_BREAK, joint_id, value)

    def over_limit_return_zero(self):
        """Return to zero when the joint is over the limit
        """
        return self._mesg(ProtocolCode.OVER_LIMIT_RETURN_ZERO, has_reply=True)

    def jog_increment_angle(self, joint_id, increment, speed):
        """Single angle incremental motion control. 

        Args:
            joint_id: Joint id 1 - 7.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed, has_reply=True)

    def jog_increment_coord(self, coord_id, increment, speed):
        """Single coordinate incremental motion control. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the job_base_increment_coord interface

        Args:
            coord_id: axis id 1 - 6.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=coord_id, speed=speed)
        value = self._coord2int(
            increment) if coord_id <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed, has_reply=True)

    def drag_teach_clean(self):
        """clear sample
        """
        return self._mesg(ProtocolCode.MERCURY_DRAG_TEACH_CLEAN)

    def get_comm_error_counts(self, joint_id, _type):
        """Read the number of communication exceptions

        Args:
            joint_id (int): joint ID
            _type (int): Error type to be read, 1 ~ 4.
                1-The number of exceptions sent by the joint
                2-The number of exceptions obtained by the joint
                3-The number of exceptions sent by the end
                4-The number of exceptions read by the end
        """
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, joint_id, _type)

    def set_pos_over_shoot(self, value):
        return self._mesg(ProtocolCode.MERCURY_SET_POS_OVER_SHOOT, [value*100])

    def get_pos_over_shoot(self):
        return self._mesg(ProtocolCode.MERCURY_GET_POS_OVER_SHOOT)

    def stop(self, deceleration=0):
        """Robot stops moving

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - Stop completion
        """
        self.is_stop = time.time()
        if deceleration == 1:
            return self._mesg(ProtocolCode.STOP, 1)
        else:
            return self._mesg(ProtocolCode.STOP)

    def pause(self, deceleration=0):
        """Robot pauses movement

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - pause completion
        """
        self.is_stop = time.time()
        if deceleration == 1:
            return self._mesg(ProtocolCode.PAUSE, 1)
        else:
            return self._mesg(ProtocolCode.PAUSE)

    def get_modified_version(self):
        return self._mesg(ProtocolCode.ROBOT_VERSION)

    def get_pos_over(self):
        return self._mesg(ProtocolCode.GET_POS_OVER)

    def clear_encoders_error(self, joint_id):
        return self._mesg(ProtocolCode.CLEAR_ENCODERS_ERROR, joint_id)

    def get_down_encoders(self):
        return self._mesg(ProtocolCode.GET_DOWN_ENCODERS)

    def set_control_mode(self, mode):
        """Set robot motion mode

        Args:
            mode (int): 0 - location mode, 1 - torque mode

        """
        return self._mesg(ProtocolCode.SET_CONTROL_MODE, mode)

    def get_control_mode(self):
        """Get robot motion mode

        Returns:
            int: 0 - location mode, 1 - torque mode
        """
        return self._mesg(ProtocolCode.GET_CONTROL_MODE)

    def set_collision_mode(self, mode):
        """Set collision detection mode

        Args:
            mode (int): 0 - disable, 1 - enable

        """
        return self._mesg(ProtocolCode.SET_COLLISION_MODE, mode)

    def set_collision_threshold(self, joint_id, value=100):
        """Set joint collision threshold

        Args:
            joint_id (int): joint ID， range 1 ~ 6
            value (int): Collision threshold, range is 50 ~ 250, default is 100, the smaller the value, the easier it is to trigger a collision
        """
        return self._mesg(ProtocolCode.SET_COLLISION_THRESHOLD, joint_id, value)

    def get_collision_threshold(self):
        """Get joint collision threshold
        """
        return self._mesg(ProtocolCode.GET_COLLISION_THRESHOLD)

    def set_torque_comp(self, joint_id, value=100):
        """Set joint torque compensation

        Args:
            joint_id (int): joint ID， range 1 ~ 6
            value (int): Compensation value, range is 0 ~ 250, default is 100, The smaller the value, the harder it is to drag the joint
        """
        return self._mesg(ProtocolCode.SET_TORQUE_COMP, joint_id, value)

    def get_torque_comp(self):
        """Get joint torque compensation
        """
        return self._mesg(ProtocolCode.GET_TORQUE_COMP)

    def power_on_only(self):
        """Only turn on the power
        """
        return self._mesg(ProtocolCode.POWER_ON_ONLY)

    def get_vr_mode(self):
        """Check if the robot is in VR mode
        """
        return self._mesg(ProtocolCode.GET_VR_MODE)

    def set_vr_mode(self, mode):
        """Set VR mode

        Args:
            mode (int): 0 - open, 1 - close
        """
        return self._mesg(ProtocolCode.SET_VR_MODE, mode)

    def get_model_direction(self):
        """Get the direction of the robot model
        """
        return self._mesg(ProtocolCode.GET_MODEL_DIRECTION)

    def set_model_direction(self, id, direction):
        """Set the direction of the robot model

        Args:
            id (int): joint ID, 1 ~ 7.
            direction (int): 0 - forward, 1 - backward
        """
        return self._mesg(ProtocolCode.SET_MODEL_DIRECTION, id, direction)

    def get_filter_len(self, rank):
        """Get the filter length

        Args:
            rank (int): 
                1 : Drag teaching sampling filter
                2 : Drag teaching execution filter
                3 : Joint velocity fusion filter
                4 : Coordinate velocity fusion filter
                5 : Drag teaching sampling period
        """
        return self._mesg(ProtocolCode.GET_FILTER_LEN, rank)

    def set_filter_len(self, rank, value):
        """Set the filter length

        Args:
            rank (int): 
                1 : Drag teaching sampling filter
                2 : Drag teaching execution filter
                3 : Joint velocity fusion filter
                4 : Coordinate velocity fusion filter
                5 : Drag teaching sampling period
            value (int): Filter length, range is 1 ~ 100
        """
        return self._mesg(ProtocolCode.SET_FILTER_LEN, rank, value)

    def clear_zero_pos(self):
        return self._mesg(ProtocolCode.CLEAR_ZERO_POS)

    def set_servo_cw(self, joint_id, err_angle):
        """Set the joint in-place feedback error angle

        Args:
            joint_id (int): Joint ID, 11 or 12.
            err_angle (float): Error range is 0 ~ 5.
        """
        return self._mesg(ProtocolCode.SET_SERVO_CW, joint_id, [self._angle2int(err_angle)])

    def get_servo_cw(self, joint_id):
        """Get the joint in-place feedback error angle

        Args:
            joint_id (int): Joint ID, 11 or 12.

        Returns:
            float: Error angle, range is 0 ~ 5.
        """
        return self._mesg(ProtocolCode.GET_SERVO_CW, joint_id)

    def clear_waist_queue(self):
        """Clear the cache points of the three motors in the torso
        """
        return self._mesg(ProtocolCode.CLEAR_WAIST_QUEUE)

    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO)

    def get_error_information(self):
        """Obtaining robot error information

        Return:
            0: No error message.
            1 ~ 6: The corresponding joint exceeds the limit position.
            16 ~ 19: Collision protection.
            32: Kinematics inverse solution has no solution.
            33 ~ 34: Linear motion has no adjacent solution.
        """
        return self._mesg(ProtocolCode.GET_ERROR_INFO)

    def send_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 6.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True)

    def send_angle(self, id, degree, speed):
        """Send one angle of joint to robot arm.

        Args:
            id : Joint id(genre.Angle)， int 1-6.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one coord to robot arm.

        Args:
            id (int): coord id, range 1 ~ 6
            coord (float): coord value.
                The coord range of `X` is -351.11 ~ 566.92.
                The coord range of `Y` is -645.91 ~ 272.12.
                The coord range of `Y` is -262.91 ~ 655.13.
                The coord range of `RX` is -180 ~ 180.
                The coord range of `RY` is -180 ~ 180.
                The coord range of `RZ` is -180 ~ 180.
            speed (int): 1 ~ 100
        """

        self.calibration_parameters(
            class_name=self.__class__.__name__, id=id, coord=coord, speed=speed)
        value = self._coord2int(coord) if id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, id, [value], speed, has_reply=True)

    def send_coords(self, coords, speed):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]). len 6 [x, y, z, rx, ry, rz]
                The coord range of `X` is -351.11 ~ 566.92.
                The coord range of `Y` is -645.91 ~ 272.12.
                The coord range of `Y` is -262.91 ~ 655.13.
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
        return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True)

    def resume(self):
        """Recovery movement"""
        return self._mesg(ProtocolCode.RESUME)

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point, 

        Args:
            servo_id: Serial number of articulated steering gear. Joint id 1 - 6
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)
    
    def set_servos_calibration(self):
        for id in range(1, 8):
            self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, id)

    def is_in_position(self, data, id=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords. angles len 6, coords len 6.
            id: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if id == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, angles=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, id)

    def is_moving(self):
        """Detect if the robot is moving

        Return:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_MOVING)

    def jog_angle(self, joint_id, direction, speed, sync=False):
        """Jog control angle.

        Args:
            joint_id (int): Joint id 1 - 6.
            direction (int): 0 - decrease, 1 - increase
            speed (int): int range 1 - 100
            sync (bool, optional): Waiting for the exercise to end. Defaults to False.

        Return:
            0: End of exercise.
            1 ~ 7: Corresponding joint exceeds limit.
            16 ~ 19: Collision protection.
            32 ~ 35: Coordinates have no adjacent solutions.
            49: Not powered on.
            50: Motor abnormality.
            51: Motor encoder error
            52: Not reaching the designated location or not reaching the designated location for more than 5 minutes (only J11, J12 available)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id, direction=direction)
        if sync:
            return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed, has_reply=True)
        self.send_jog_command = True
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed, sync=False):
        """Jog control coord. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the jog_base_coord interface

        Args:
            coord_id (int): int 1-6
            direction (int): 0 - decrease, 1 - increase
            speed (int): 1 - 100
            sync (bool, optional): Waiting for the exercise to end. Defaults to False.

        Returns:
            1: End of the Movement

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, direction=direction)
        if sync:
            return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed, has_reply=True)
        self.send_jog_command = True
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_base_increment_coord(self, axis_id, increment, speed):
        """Single coordinate incremental motion control

        Args:
            axis_id (int): axis id, range 1 ~ 6 corresponds to [x,y,z,rx,ry,rz]
            increment (float): Incremental value
            speed (int): speed
            
        Return:
            0: End of exercise.
            1 ~ 7: Corresponding joint exceeds limit.
            16 ~ 19: Collision protection.
            32 ~ 35: Coordinates have no adjacent solutions.
            49: Not powered on.
            50: Motor abnormality.
            51: Motor encoder error
            52: Not reaching the designated location or not reaching the designated location for more than 5 minutes (only J11, J12 available)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=axis_id, speed=speed)
        coord_list = []
        if axis_id < 4:
            coord_list.append(self._coord2int(increment))
        else:
            coord_list.append(self._angle2int(increment))
        return self._mesg(ProtocolCode.JOG_BASE_INCREMENT_COORD, axis_id, coord_list, speed, has_reply=True)

    def get_max_speed(self, mode):
        """Get maximum speed
        
        Args:
            mode (int): 0 - angle speed. 1 - coord speed.
            
        Return: 
            angle speed range 1 ~ 150°/s. coord speed range 1 ~ 200mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.GET_SPEED, mode)

    def set_max_speed(self, mode, max_speed):
        """Set maximum speed

        Args:
            mode (int): 0 - angle speed. 1 - coord speed.
            max_speed (int): angle speed range 1 ~ 150°/s. coord speed range 1 ~ 200mm/s

        Returns:
            1: _description_
        """
        self.calibration_parameters(lass_name=self.__class__.__name__, mode=mode, max_speed=max_speed)
        return self._mesg(ProtocolCode.SET_SPEED, max_speed)

    def set_max_acc(self, mode, max_acc):
        """Set maximum acceleration

        Args:
            mode (int): 0 - angle acceleration. 1 - coord acceleration.
            max_acc (int): maximum acceleration value. Angular acceleration range is 1~150°/s. Coordinate acceleration range is 1~400mm/s
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode, max_acc=max_acc)
        return self._mesg(ProtocolCode.SET_MAX_ACC, mode, max_acc)

    def get_max_acc(self, mode):
        """Get maximum acceleration

        Args:
            mode (int): 0 - angle acceleration. 1 - coord acceleration.
        """
        return self._mesg(ProtocolCode.GET_MAX_ACC, mode)

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args: 
            joint_id: Joint id 1 - 6 or 11 ~ 12

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: Joint id 1 - 6 or 11 ~ 12

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id)

    def set_joint_max_angle(self, id, angle):
        """Set the maximum angle of the joint (must not exceed the maximum angle specified for the joint)

        Args:
            id (int): Joint id 1 - 6  or 11 ~ 12
            angle: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -55 ~ 95. The angle range of joint 3 is -173 ~ 5. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -20 ~ 265. The angle range of joint 6 is -180 ~ 180. The angle range of joint 11 is -60 ~ 0. The angle range of joint 12 is -138 ~ 188.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    def set_joint_min_angle(self, id, angle):
        """Set the minimum angle of the joint (must not be less than the minimum angle specified by the joint)

        Args:
            id (int): Joint id 1 - 6.
            angle: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -55 ~ 95. The angle range of joint 3 is -173 ~ 5. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -20 ~ 265. The angle range of joint 6 is -180 ~ 180. The angle range of joint 11 is -60 ~ 0. The angle range of joint 12 is -138 ~ 188.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id: Joint id 1 - 6

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id)

    def is_all_servo_enable(self):
        """Detect the connection status of all joints

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE)

    def get_servo_data(self, servo_id, address, mode):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Joint id 11 or 12
            address: Data address. range 5 ~ 69
            mode: 1 - indicates that value is one byte(default), 2 - represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode not in [1, 2]:
            raise Exception("mode must be 1 or 2")
        self.calibration_parameters(class_name = self.__class__.__name__, servo_id=servo_id, address=address)
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, servo_id, address, mode
        )
        
    def set_servo_data(self, servo_id, address, value, mode):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Joint id 11 or 12
            address: Data address. range 5 ~ 69
            value: 0 - 4096
            mode: 1 - indicates that value is one byte(default), 2 - represents a value of two bytes.
        """
        if mode not in [1, 2]:
            raise Exception("mode must be 1 or 2")
        self.calibration_parameters(class_name = self.__class__.__name__, servo_id=servo_id, address=address)
        if mode == 1:
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, address, value, mode)
        else:
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, address, [value], mode)

    def get_servo_speeds(self):
        """Get joint speed
        
        Return: 
            unit step/s
        """
        return self._mesg(ProtocolCode.GET_SERVO_SPEED)
    
    def get_servo_currents(self):
        """Get joint current
        
        Return: 
            0 ~ 5000 mA
        """
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS)
    
    def get_servo_status(self):
        """Get joint status
        
        """
        return self._mesg(ProtocolCode.GET_SERVO_STATUS)
    
    def set_gripper_state(self, flag, speed, gripper_type=None):
        """Set gripper switch state

        Args:
            flag  (int): 0 - open, 1 - close
            speed (int): 1 ~ 100
            gripper_type (int): default 1
                1 : Adaptive gripper. default to adaptive gripper
                2 : Parallel Gripper
        """
        if gripper_type is None:
            self.calibration_parameters(class_name = self.__class__.__name__, flag=flag, speed=speed)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)
        else:
            self.calibration_parameters(class_name = self.__class__.__name__, flag=flag, speed=speed, gripper_type=gripper_type)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed, gripper_type)
        
    def set_gripper_value(self, gripper_value, speed, gripper_type=None):
        """Set gripper value

        Args:
            gripper_value (int): 0 ~ 100
            speed (int): 1 ~ 100
            gripper_type (int): default 1
                1 : Adaptive gripper. default to adaptive gripper
                2 : Parallel Gripper
        """
        if gripper_type is not None:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_value=gripper_value, speed=speed,
                                        gripper_type=gripper_type)
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed, gripper_type)
        else:
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed)
        
    def set_gripper_calibration(self):
        """Set the current position to zero, set current position value is `2048`."""
        return self._mesg(ProtocolCode.SET_GRIPPER_CALIBRATION)
    
    def set_gripper_mode(self, mode):
        """Set gripper mode
        
        Args:
            mode: 0 - transparent transmission. 1 - Port Mode.
        
        """
        self.calibration_parameters(class_name = self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_GRIPPER_MODE, mode)
    
    def get_gripper_mode(self):
        """Get gripper mode
        
        Return:
            mode: 0 - transparent transmission. 1 - Port Mode.
        """
        return self._mesg(ProtocolCode.GET_GRIPPER_MODE)
    
    def set_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot end.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(class_name = self.__class__.__name__, rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)
    
    def set_digital_output(self, pin_no, pin_signal):
        """Set the end-of-arm IO status

        Args:
            pin_no (int): 1 or 2
            pin_signal (int): 0 / 1
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_no=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """Read the end-of-arm IO status

        Args:
            pin_no (int): 1 or 2

        Returns:
            int: 0 or 1
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_no=pin_no)
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no)

    def get_world_reference(self):
        """Get the world coordinate system"""
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE)
    
    def set_tool_reference(self, coords):
        """Set tool coordinate system
        
        Args:
            coords: a list of coords value(List[float])
        """
        self.calibration_parameters(class_name = self.__class__.__name__, coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_TOOL_REFERENCE, coord_list)
    
    def get_tool_reference(self):
        """Get tool coordinate system """
        return self._mesg(ProtocolCode.GET_TOOL_REFERENCE)
    
    def set_reference_frame(self, rftype):
        """Set the base coordinate system
        
        Args:
            rftype: 0 - base 1 - tool.
        """
        self.calibration_parameters(class_name = self.__class__.__name__, rftype=rftype)
        return self._mesg(ProtocolCode.SET_REFERENCE_FRAME, rftype)
    
    def get_reference_frame(self):
        """Get the base coordinate system
        
        Return: 
            0 - base 1 - tool.
        """
        return self._mesg(ProtocolCode.GET_REFERENCE_FRAME)
    
    def set_movement_type(self, move_type):
        """Set movement type
        
        Args:
            move_type: 1 - movel, 0 - moveJ
        """
        self.calibration_parameters(class_name = self.__class__.__name__, move_type=move_type)
        return self._mesg(ProtocolCode.SET_MOVEMENT_TYPE, move_type)
    
    def get_movement_type(self):
        """Get movement type
        
        Return: 
            1 - movel, 0 - moveJ
        """
        return self._mesg(ProtocolCode.GET_MOVEMENT_TYPE)
    
    def set_end_type(self, end):
        """Set end coordinate system
        
        Args:
            end: int
                0 - flange, 1 - tool
        """
        self.calibration_parameters(class_name = self.__class__.__name__, end=end)
        return self._mesg(ProtocolCode.SET_END_TYPE, end)
    
    def get_end_type(self):
        """Get end coordinate system
        
        Return: 
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE)
        
    def get_monitor_mode(self):
        return self._mesg(ProtocolCode.GET_MONITOR_MODE)
    
    def set_monitor_mode(self,mode):
        return self._mesg(ProtocolCode.SET_MONITOR_MODE, mode)
    
    def set_limit_switch(self, limit_mode, state):
        """Set the limit switches. No save after power off

        Args:
            limit_mode (int): 1 - Location out of tolerance. 2 - Synchronous control
            state (int): 0 - close. 1 - open
        """
        self.calibration_parameters(class_name = self.__class__.__name__, limit_mode=limit_mode, state=state)
        if limit_mode == 2 and state == 0:
            self.sync_mode = False
        elif limit_mode == 2 and state == 1:
            self.sync_mode = True
        return self._mesg(ProtocolCode.SET_LIMIT_SWITCH, limit_mode, state)
    
    def get_limit_switch(self):
        """Get the limit switches
        
        Return:
            list : [Location out of tolerance state, sync state], 0 - close. 1 - open
        """
        res = self._mesg(ProtocolCode.GET_LIMIT_SWITCH)
        if isinstance(res, list) and res[1] == 0:
            self.sync_mode = False
        return res
    
    def solve_inv_kinematics(self, new_coords, old_angles):
        """_summary_
        """
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(new_coords[idx]))
        for angle in new_coords[3:]:
            coord_list.append(self._angle2int(angle))
        angles = [self._angle2int(angle) for angle in old_angles]
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)
    
    def get_drag_fifo(self):
        return self._mesg(ProtocolCode.GET_DRAG_FIFO, has_reply=True)
    
    def set_drag_fifo(self, angles):
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SET_DRAG_FIFO, angles)
    
    def get_drag_fifo_len(self):
        return self._mesg(ProtocolCode.GET_DRAG_FIFO_LEN, has_reply=True)
    
    def jog_rpy(self, axis, direction, speed):
        """Rotate the end point around the fixed axis of the base coordinate system

        Args:
            axis (int): _description_
            direction (int): 1 - Forward. 0 - Reverse.
            speed (int): 1 ~ 100.
        """
        return self._mesg(ProtocolCode.JOG_RPY, axis, direction, speed, has_reply=True)
    
    def get_collision_mode(self):
        """Get collision detection status
        
        Return:
            1 - open
            0 - close
        """
        return self._mesg(ProtocolCode.GET_COLLISION_MODE)
    
    def set_electric_gripper(self, mode):
        """Set the state of the electric gripper

        Args:
            mode (int): 0 - open. 1 - close
        """
        self.calibration_parameters(class_name = self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_ELECTRIC_GRIPPER, mode)
    
    def init_electric_gripper(self):
        """Electric Gripper Initialization
        """
        return self._mesg(ProtocolCode.INIT_ELECTRIC_GRIPPER)
    
    def get_servo_encoder(self, id):
        return self._mesg(ProtocolCode.GET_ENCODER, id)
    
    def get_servo_encoders(self):
        return self._mesg(ProtocolCode.GET_ENCODERS)
    
    def set_base_io_output(self, pin_no, pin_signal):
        """Set the base output IO status

        Args:
            pin_no: pin port number. range 1 ~ 6
            pin_signal: 0 - low. 1 - high.
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_no=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_base_io_input(self, pin_no):
        """Get the input IO status of the base

        Args:
            pin_no: (int) pin port number. range 1 ~ 6
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_no=pin_no)
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)