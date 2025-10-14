# coding=utf-8
import threading
import time
import struct
from datetime import datetime
import re

from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode, write, read, DataProcessor, FingerGripper
from pymycobot.end_control import ForceGripper, ThreeHand


class CloseLoop(DataProcessor, ForceGripper, ThreeHand):
    _write = write
    _read = read

    def __init__(self, debug=False):
        super(CloseLoop, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.is_stop = False
        self.write_command = []
        self.read_command = []
        self.no_return = False
        self.sync_mode = True
        self.all_debug_data = []
        self.all_read_data = b""
        self.lock_out = threading.Lock()
        self.lock = threading.Lock()

    def _send_command(self, genre, real_command):
        self.write_command.append(genre)
        if "Socket" in self.__class__.__name__ or "Client" in self.__class__.__name__:
            self._write(self._flatten(real_command), method="socket")
        else:
            self._write(self._flatten(real_command))

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
        real_command, has_reply, _async = super(
            CloseLoop, self)._mesg(genre, *args, **kwargs)
        is_in_position = False
        is_get_return = False
        lost_times = 0
        with self.lock:
            self._send_command(genre, real_command)
        if _async:
            with self.lock:
                if genre in self.write_command:
                    self.write_command.remove(genre)
            return 1
        t = time.time()
        wait_time = 0.15
        big_wait_time = False
        if genre == ProtocolCode.POWER_ON:
            wait_time = 8
            big_wait_time = True
        elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.POWER_ON_ONLY, ProtocolCode.RELEASE_ALL_SERVOS,
                       ProtocolCode.FOCUS_ALL_SERVOS,
                       ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP,
                       ProtocolCode.SET_CONTROL_MODE, ProtocolCode.MERCURY_DRAG_TEACH_CLEAN]:
            wait_time = 3
            big_wait_time = True
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
                ProtocolCode.JOG_RPY,
                ProtocolCode.WRITE_MOVE_C_R,
                ProtocolCode.MERCURY_DRAG_TECH_EXECUTE] and self.sync_mode:
            wait_time = 300
            is_in_position = True
            big_wait_time = True
        elif genre in [ProtocolCode.SERVO_RESTORE]:
            wait_time = 0.3
            big_wait_time = True

        elif genre in (ProtocolCode.MERCURY_SET_TOQUE_GRIPPER, ProtocolCode.MERCURY_GET_TOQUE_GRIPPER):
            wait_time = 0.3
            if real_command[3] == FingerGripper.SET_HAND_GRIPPER_CALIBRATION:
                wait_time = 10
        if genre == ProtocolCode.SET_FRESH_MODE and self.__class__.__name__ == 'Pro450Client':
            wait_time = 4

        need_break = False
        data = None

        if self.__class__.__name__ == "MercurySocket":
            timeout = 1
        elif self.__class__.__name__ == "Pro450Client":
            if genre == ProtocolCode.SET_FRESH_MODE:
                timeout = 4
            else:
                timeout = 3
        elif self.__class__.__name__ == "MercuryArmsSocket":
            timeout = 1
        else:
            timeout = 0.5

        interval_time = time.time()
        is_moving = 0
        check_is_moving_t = 1
        while True and time.time() - t < wait_time:
            with self.lock_out:
                for v in self.read_command:
                    read_data = v[0]
                    if is_get_return and is_in_position and read_data[2] == 0x04 and read_data[3] == 0x5b:
                        if v[1] < t:
                            with self.lock:
                                if v in self.read_command:
                                    self.read_command.remove(v)
                            continue
                        # print("到位反馈", flush=True)
                        is_get_return = True
                        need_break = True
                        data = read_data
                        with self.lock:
                            if v in self.read_command:
                                self.read_command.remove(v)
                            if genre in self.write_command:
                                self.write_command.remove(genre)

                    elif genre == read_data[3] and read_data[2] == 5 and read_data[4] == 0xFF:
                        # 通信闭环
                        # print(-2)
                        # print("闭环", flush=True)
                        is_get_return = True
                        with self.lock:
                            if v in self.read_command:
                                self.read_command.remove(v)
                        if has_reply == False or self.sync_mode == False:
                            # print(-3)
                            # print("仅闭环退出", flush=True)
                            need_break = True
                            data = read_data
                    elif genre == read_data[3]:
                        # print(-4)
                        # print("正常读取", flush=True)
                        need_break = True
                        data = read_data
                        with self.lock:
                            if v in self.read_command:
                                self.read_command.remove(v)
                            if genre in self.write_command:
                                self.write_command.remove(genre)
                        break
            if (not big_wait_time or is_in_position) and not is_get_return and time.time() - t > timeout:
                # 发送指令以后，超时0.5S没有收到第一次反馈，并且是运动指令，并且超时时间是正常指令
                t = time.time()
                moving = self.is_moving()
                if isinstance(moving, int):
                    if moving != 0:
                        continue
                # 运动指令丢失，重发
                # print("运动指令丢失，重发", flush=True)
                lost_times += 1
                # print("运动指令丢失，重发")
                with self.lock:
                    self._send_command(genre, real_command)
            if need_break:
                # print("正常退出", flush=True)
                break
            if lost_times > 2:
                # 重传3次失败，返回-1
                # print("重传3次失败，返回-1", flush=True)
                return -1
            # if t < self.is_stop and genre != ProtocolCode.STOP:
            #     # 打断除了stop指令外的其他指令的等待
            #     self.is_stop = 0
            #     break
            if is_in_position and time.time() - interval_time > check_is_moving_t and wait_time == 300:
                interval_time = time.time()
                moving = self.is_moving()
                # if isinstance(moving, int) and moving == 0:
                #     print("停止运动，退出")
                #     is_moving += 1
                #     # 由于is_moving接口比到位反馈更快，所以第一次收到停止运动后，将下一次的检测时间更改为0.25s，防止此处先退出，返回-2
                #     check_is_moving_t = 0.25
                #     if is_moving > 1:
                #         # 累计两次才退出
                #         with self.lock:
                #             if genre in self.write_command:
                #                 self.write_command.remove(genre)
                #         return -2
                if isinstance(moving, int) and moving == 0:
                    is_moving += 1
                    if is_moving == 1:
                        # 第一次检测到停止，只是标记，不退出
                        check_is_moving_t = 0.25
                    elif is_moving > 1:
                        # 第二次检测到停止才真正退出
                        with self.lock:
                            if genre in self.write_command:
                                self.write_command.remove(genre)
                        return -2
            time.sleep(0.001)
        else:
            # print("ERROR: ---超时---")
            pass
        if data is None:
            # print("未拿到数据")
            return data
        data = bytearray(data)
        data_len = data[2] - 3
        # unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]
        if genre == ProtocolCode.GET_BASIC_INPUT:
            data_pos = 5
            data_len -= 1
            if self.__class__.__name__ == "Pro630Client":
                data_len += 1
            elif self.__class__.__name__ == "Pro450Client":
                data_len += 1
                data_pos -= 1
        elif genre == ProtocolCode.GET_DIGITAL_INPUT:
            if self.__class__.__name__ == "Pro450Client":
                data_len = 1
                data_pos = 4
        else:
            data_pos = 4
        if is_get_return:
            data_len -= 1
            data_pos += 1
            if data[2] == 5:
                # print("握手成功")
                return data[5]
            elif data[2] == 4:
                res = self._status_explain(data[4])
                if res != "":
                    print(res)
                return data[4]
        valid_data = data[data_pos: data_pos + data_len]
        return (valid_data, data_len)

    def read_thread(self, method=None):
        all_data = b''
        while True:
            try:
                datas = b""
                data_len = -1
                k = 0
                pre = 0
                t = time.time()
                wait_time = 0.5
                if method is not None:
                    try:
                        self.sock.settimeout(wait_time)
                        data = self.sock.recv(1024)
                        if isinstance(data, str):
                            datas = bytearray()
                            for i in data:
                                datas += hex(ord(i))
                            data = datas
                        elif isinstance(data, bytes):
                            hex_string = ''.join(f'{b:02X}' for b in data)
                            # print(hex_string)
                    except:
                        data = b""
                    if data != b"":
                        if isinstance(data, str):
                            pattern = re.compile(rb'###(.*?)###', re.DOTALL)
                            matches = pattern.findall(data)
                            for match in matches:
                                if self.check_python_version() == 2:
                                    command_log = ""
                                    for d in match:
                                        command_log += hex(ord(d))[2:] + " "
                                    self.log.debug(
                                        "_read : {}".format(command_log))
                                    # self.log.debug("_read: {}".format([hex(ord(d)) for d in data]))
                                else:
                                    command_log = ""
                                    for d in match:
                                        command_log += hex(d)[2:] + " "
                                    self.log.debug(
                                        "_read : {}".format(command_log))
                                res = self._process_received(match)
                                if res != []:
                                    with self.lock:
                                        self.read_command.append(
                                            [res, time.time()])
                        elif isinstance(data, bytes):
                            command_log = " ".join("{:02X}".format(b) for b in data)
                            self.log.debug("_read : {}".format(command_log))
                            res = self._process_received(data)
                            if res != []:
                                with self.lock:
                                    self.read_command.append(
                                        [res, time.time()])
                else:
                    while True and time.time() - t < wait_time:
                        if self._serial_port.isOpen() and self._serial_port.inWaiting() > 0:
                            data = self._serial_port.read()
                            if self.save_serial_log:
                                all_data += data
                            # self.log.info(all_read_data)
                            k += 1
                            # print(datas, flush=True)
                            if data_len == 3:
                                datas += data
                                crc = self._serial_port.read(2)
                                self.all_read_data += data
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
                        else:
                            time.sleep(0.001)
                    else:
                        datas = b''
                    if self.save_serial_log:
                        if all_data != b'':
                            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            with open("all_log.txt", "a") as f:
                                f.write(str(current_time) +
                                        str(all_data)[2:-1] + "\n")
                            all_data = b''
                    if datas != b'':
                        res = self._process_received(datas)
                        if self.check_python_version() == 2:
                            command_log = ""
                            for d in datas:
                                data = hex(d)[2:]
                                if len(data) != 2:
                                    data = "0" + data
                                command_log += data + " "
                            self.log.debug("_read : {}".format(command_log))
                        else:
                            command_log = ""
                            for d in datas:
                                data = hex(d)[2:]
                                if len(data) != 2:
                                    data = "0" + data
                                command_log += data + " "
                            self.log.debug("_read : {}".format(command_log))
                        if datas[3] == 0x5D:
                            debug_data = []
                            for i in range(4, 32, 4):
                                byte_value = int.from_bytes(
                                    datas[i:i + 4], byteorder='big', signed=True)
                                debug_data.append(byte_value)
                            # self.log.info("SERVO_COMMAND : {}".format(debug_data))
                            with open("plotPos.txt", "a") as f:
                                f.write(str(debug_data) + "\n")
                            continue
                        elif datas[3] == 0x8E and datas[2] == 0x3B:
                            debug_data = []

                            for i in range(4, 60, 2):
                                if i < 40:
                                    data = self._decode_int16(
                                        datas[i:i + 2]) / 1000
                                else:
                                    data = self._decode_int16(datas[i:i + 2])
                                debug_data.append(data)
                            self.all_debug_data.append(debug_data)

                            continue
                        if res == []:
                            # print("res is empty")
                            continue
                        # if datas[3] == 0x5b:
                        #     print("等待加入到读取队列")
                        with self.lock:
                            self.read_command.append([res, time.time()])
                            # if datas[3] == 0x5b:
                            # print("加入到读取队列成功")

            except Exception as e:
                # self.log.error("read error: {}".format(traceback.format_exc()))
                pass
            time.sleep(0.001)

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

    def bytes4_to_int(self, bytes4):
        i = 0
        res = []
        data_len = len(bytes4)
        while i < data_len:
            if self.check_python_version() == 2:
                byte_value = 0
                data = bytes4[i:i+4]
                for b in data:
                    byte_value = byte_value * 256 + b

                if byte_value & (1 << 31):
                    byte_value -= 1 << (32)
            else:
                byte_value = int.from_bytes(
                    bytes4[i:i+4], byteorder='big', signed=True)
            i += 4
            res.append(byte_value)
        return res

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
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
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

    def go_home(self, robot=1, speed=20, _async=False):
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
            return self.send_angles([0, 0, 0, 0, 0, 90, 0], speed, _async=_async)
        else:
            self.send_angle(11, 0, speed)
            self.send_angle(12, 0, speed)
            self.send_angle(13, 0, speed)
            return self.send_angles([0, 0, 0, 0, 0, 90, 0], speed, _async=_async)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 7 or 11 ~ 13.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
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

    def drag_teach_save(self):
        """Start recording the dragging teaching point. In order to show the best sports effect, the recording time should not exceed 90 seconds."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_SAVE)

    def drag_teach_execute(self):
        """Start dragging the teaching point and only execute it once."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE, has_reply=True)

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

    def tool_serial_restore(self):
        """485 factory reset
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_RESTORE)

    def tool_serial_ready(self):
        """Set up 485 communication

        Return:
            0 : not set
            1 : Setup completed
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_READY)

    def tool_serial_available(self):
        """Read 485 buffer length

        Return:
            485 buffer length available for reading
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_AVAILABLE)

    def tool_serial_read_data(self, data_len):
        """Read fixed length data. Before reading, read the buffer length first. After reading, the data will be cleared

        Args:
            data_len (int): The number of bytes to be read, range 1 ~ 45
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, data_len=data_len)
        return self._mesg(ProtocolCode.TOOL_SERIAL_READ_DATA, data_len)

    def tool_serial_write_data(self, command):
        """End 485 sends data， Data length range is 1 ~ 45 bytes

        Args:
            command (list) : data instructions

        Return:
            number of bytes received
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_WRITE_DATA, command)

    def tool_serial_flush(self):
        """Clear 485 buffer
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_FLUSH)

    def tool_serial_peek(self):
        """View the first data in the buffer, the data will not be cleared

        Return:
            1 byte data
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_PEEK)

    def tool_serial_set_baud(self, baud=115200):
        """Set 485 baud rate, default 115200

        Args:
            baud (int): baud rate
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_SET_BAUD, baud)

    def tool_serial_set_timeout(self, max_time):
        """Set 485 timeout in milliseconds, default 30ms

        Args:
            max_time (int): timeout
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, max_time=max_time)
        return self._mesg(ProtocolCode.TOOL_SERIAL_SET_TIME_OUT, max_time)

    def get_robot_status(self):
        return self._mesg(ProtocolCode.MERCURY_ROBOT_STATUS)

    def power_on(self, mode=1):
        """Power on the robot

        Args:
            mode (int, optional): 1 - Motor enable, 0 - The motor is not enabled, only initialized.

        Return:
            1: success
            2: failed
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
        res = self._mesg(ProtocolCode.POWER_ON, mode)
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

    def focus_servo(self, joint_id):
        """Power on designated servo

        Args:
            joint_id: int. joint id 1 - 7
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, joint_id)

    def release_servo(self, joint_id):
        """Power off designated servo

        Args:
            joint_id: int. joint id 1 - 7
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.RELEASE_SERVO, joint_id)

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
            class_name=self.__class__.__name__, joint_id=joint_id, value=value)
        return self._mesg(ProtocolCode.SET_BREAK, joint_id, value)

    def over_limit_return_zero(self):
        """Return to zero when the joint is over the limit
        """
        return self._mesg(ProtocolCode.OVER_LIMIT_RETURN_ZERO, has_reply=True)

    def jog_increment_angle(self, joint_id, increment, speed, _async=False):
        """Single angle incremental motion control. 

        Args:
            joint_id: Joint id 1 - 7.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, speed=speed)

        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed, has_reply=True,
                          _async=_async)

    def jog_increment_coord(self, coord_id, increment, speed, _async=False):
        """Single coordinate incremental motion control. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the job_base_increment_coord interface

        Args:
            joint_id: axis id 1 - 6.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, speed=speed)
        value = self._coord2int(
            increment) if coord_id <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed, has_reply=True, _async=_async)

    def get_quick_move_info(self):
        return self._mesg(ProtocolCode.GET_QUICK_INFO)

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
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, _type=_type)
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, joint_id, _type)

    def set_pos_over_shoot(self, shoot_value):
        self.calibration_parameters(
            class_name=self.__class__.__name__, shoot_value=shoot_value)
        return self._mesg(ProtocolCode.MERCURY_SET_POS_OVER_SHOOT, [shoot_value * 100])

    def get_pos_over_shoot(self):
        return self._mesg(ProtocolCode.MERCURY_GET_POS_OVER_SHOOT)

    def stop(self, deceleration=0, _async=False):
        """Robot stops moving

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - Stop completion
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, deceleration=deceleration)
        # self.is_stop = time.time()
        if deceleration == 1:
            return self._mesg(ProtocolCode.STOP, 1, _async=_async)
        else:
            return self._mesg(ProtocolCode.STOP, _async=_async)

    def pause(self, deceleration=0):
        """Robot pauses movement

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - pause completion
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, deceleration=deceleration)

        # self.is_stop = time.time()
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_COLLISION_MODE, mode)

    def set_collision_threshold(self, joint_id, threshold_value=100):
        """Set joint collision threshold

        Args:
            joint_id (int): joint ID， range 1 ~ 6
            threshold_value (int): Collision threshold, range is 50 ~ 250, default is 100, the smaller the value, the easier it is to trigger a collision
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id,
                                    threshold_value=threshold_value)
        return self._mesg(ProtocolCode.SET_COLLISION_THRESHOLD, joint_id, threshold_value)

    def get_collision_threshold(self):
        """Get joint collision threshold
        """
        return self._mesg(ProtocolCode.GET_COLLISION_THRESHOLD)

    def set_torque_comp(self, joint_id, comp_value=100):
        """Set joint torque compensation

        Args:
            joint_id (int): joint ID， range 1 ~ 6
            comp_value (int): Compensation value, range is 0 ~ 250, default is 100, The smaller the value, the harder it is to drag the joint
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, comp_value=comp_value)
        return self._mesg(ProtocolCode.SET_TORQUE_COMP, joint_id, comp_value)

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
        self.calibration_parameters(class_name=self.__class__.__name__, vr_mode=mode)
        return self._mesg(ProtocolCode.SET_VR_MODE, mode)

    def get_model_direction(self):
        """Get the direction of the robot model
        """
        return self._mesg(ProtocolCode.GET_MODEL_DIRECTION)

    def set_model_direction(self, joint_id, direction):
        """Set the direction of the robot model

        Args:
            joint_id (int): joint ID, 1 ~ 7.
            direction (int): 0 - forward, 1 - backward
        """
        return self._mesg(ProtocolCode.SET_MODEL_DIRECTION, joint_id, direction)

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
        self.calibration_parameters(class_name=self.__class__.__name__, rank=rank)
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
        self.calibration_parameters(class_name=self.__class__.__name__, rank=rank, rank_value=value)
        return self._mesg(ProtocolCode.SET_FILTER_LEN, rank, value)

    def clear_zero_pos(self):
        return self._mesg(ProtocolCode.CLEAR_ZERO_POS)

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

    def send_angles(self, angles, speed, _async=False):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 7.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True, _async=_async)

    def send_angle(self, joint_id, angle, speed, _async=False):
        """Send one angle of joint to robot arm.

        Args:
            joint_id : Joint id(genre.Angle)， int 1-7.
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
                The coord range of `X` is -351.11 ~ 566.92.
                The coord range of `Y` is -645.91 ~ 272.12.
                The coord range of `Y` is -262.91 ~ 655.13.
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
        return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True, _async=_async)

    def resume(self):
        """Recovery movement"""
        return self._mesg(ProtocolCode.RESUME)

    def set_servo_calibration(self, joint_id):
        """The current position of the calibration joint actuator is the angle zero point, 

        Args:
            joint_id: Serial number of articulated steering gear. Joint id 1 - 6
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, joint_id)

    def set_servos_calibration(self):
        for id in range(1, 8):
            self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, id)

    def is_in_position(self, data, mode=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords. angles len 6, coords len 6.
            mode: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        if mode == 1:
            self.calibration_parameters(
                class_name=self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif mode == 0:
            self.calibration_parameters(
                class_name=self.__class__.__name__, angles=data)
            data_list = [self._angle2int(i) for i in data]

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, mode)

    def is_moving(self):
        """Detect if the robot is moving

        Return:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_MOVING)

    def jog_angle(self, joint_id, direction, speed, _async=True):
        """Jog control angle.

        Args:
            joint_id (int): Joint id 1 - 7.
            direction (int): 0 - decrease, 1 - increase
            speed (int): int range 1 - 100
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.

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
            class_name=self.__class__.__name__, joint_id=joint_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed, _async=_async, has_reply=True)

    def jog_coord(self, coord_id, direction, speed, _async=True):
        """Jog control coord. This interface is based on a single arm 1-axis coordinate system. If you are using a dual arm robot, it is recommended to use the jog_base_coord interface

        Args:
            coord_id (int): int 1-6
            direction (int): 0 - decrease, 1 - increase
            speed (int): 1 - 100
            _async (bool, optional): Whether to execute asynchronous control. Defaults to True.

        Returns:
            1: End of the Movement

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed, _async=_async, has_reply=True)

    def get_max_speed(self, mode):
        """Get maximum speed

        Args:
            mode (int): 0 - angle speed. 1 - coord speed.

        Return: 
            angle speed range 1 ~ 150°/s. coord speed range 1 ~ 200mm/s
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.GET_SPEED, mode)

    def set_max_speed(self, mode, max_speed):
        """Set maximum speed

        Args:
            mode (int): 0 - angle speed. 1 - coord speed.
            max_speed (int): angle speed range 1 ~ 150°/s. coord speed range 1 ~ 200mm/s

        Returns:
            1: _description_
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode, max_speed=max_speed)
        return self._mesg(ProtocolCode.SET_SPEED, mode, [max_speed])

    def set_max_acc(self, mode, max_acc):
        """Set maximum acceleration

        Args:
            mode (int): 0 - angle acceleration. 1 - coord acceleration.
            max_acc (int): maximum acceleration value. Angular acceleration range is 1 ~ 150°/s. Coordinate acceleration range is 1 ~ 400mm/s
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode, max_acc=max_acc)
        return self._mesg(ProtocolCode.SET_MAX_ACC, mode, [max_acc])

    def get_max_acc(self, mode):
        """Get maximum acceleration

        Args:
            mode (int): 0 - angle acceleration. 1 - coord acceleration.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.GET_MAX_ACC, mode)

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args: 
            joint_id: Joint id 1 - 6 or 11 ~ 12

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: Joint id 1 - 6 or 11 ~ 12

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id)

    def set_joint_max_angle(self, joint_id, degree):
        """Set the maximum angle of the joint (must not exceed the maximum angle specified for the joint)

        Args:
            joint_id (int): Joint id 1 - 6  or 11 ~ 12
            degree: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -55 ~ 95. The angle range of joint 3 is -173 ~ 5. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -20 ~ 265. The angle range of joint 6 is -180 ~ 180. The angle range of joint 11 is -60 ~ 0. The angle range of joint 12 is -138 ~ 188.

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
            degree: The angle range of joint 1 is -165 ~ 165. The angle range of joint 2 is -55 ~ 95. The angle range of joint 3 is -173 ~ 5. The angle range of joint 4 is -165 ~ 165. The angle range of joint 5 is -20 ~ 265. The angle range of joint 6 is -180 ~ 180. The angle range of joint 11 is -60 ~ 0. The angle range of joint 12 is -138 ~ 188.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, degree=degree)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, joint_id, degree)

    def is_servo_enable(self, joint_id):
        """To detect the connection state of a single joint

        Args:
            joint_id: Joint id 1 - 7

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, joint_id)

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
        self.calibration_parameters(
            class_name=self.__class__.__name__, servo_id=servo_id)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, servo_id=servo_id)
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
            self.calibration_parameters(
                class_name=self.__class__.__name__, flag=flag, speed=speed)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, flag=flag, speed=speed,
                                        gripper_type=gripper_type)
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
            self.calibration_parameters(
                class_name=self.__class__.__name__, gripper_value=gripper_value, speed=speed)
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed)

    def set_gripper_calibration(self):
        """Set the current position to zero, set current position value is `2048`."""
        return self._mesg(ProtocolCode.SET_GRIPPER_CALIBRATION)

    def set_gripper_mode(self, mode):
        """Set gripper mode

        Args:
            mode: 0 - transparent transmission. 1 - Port Mode.

        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def set_digital_output(self, pin_no, pin_signal):
        """Set the end-of-arm IO status

        Args:
            pin_no (int): 1 or 2
            pin_signal (int): 0 / 1
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, pin_no=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """Read the end-of-arm IO status

        Args:
            pin_no (int): 1 or 2

        Returns:
            int: 0 or 1
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, pin_no=pin_no)
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no)

    def get_digital_inputs(self):
        """Read the end-of-arm IO status, IN1/IN2/ATOM Button"""
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUTS)

    def get_world_reference(self):
        """Get the world coordinate system"""
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE)

    def set_world_reference(self, coords):
        """Set the world coordinate system

        Args:
            coords: a list of coords value(List[float]). [x(mm), y, z, rx(angle), ry, rz]
        """
        # self.calibration_parameters(class_name = self.__class__.__name__, coords=coords)
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
        # self.calibration_parameters(class_name = self.__class__.__name__, coords=coords)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, rftype=rftype)
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
            move_type: 0 - moveJ, 1 - moveL, 2 - moveF, 3 - moveS, 4 - CP
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, move_type=move_type)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, end=end)
        return self._mesg(ProtocolCode.SET_END_TYPE, end)

    def get_end_type(self):
        """Get end coordinate system

        Return: 
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE)

    def get_monitor_mode(self):
        return self._mesg(ProtocolCode.GET_MONITOR_MODE)

    def set_monitor_mode(self, mode):
        return self._mesg(ProtocolCode.SET_MONITOR_MODE, mode)

    def set_limit_switch(self, limit_mode, state):
        """Set the limit switches. No save after power off

        Args:
            limit_mode (int): 1 - Location out of tolerance. 2 - Synchronous control
            state (int): 0 - close. 1 - open
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, limit_mode=limit_mode, state=state)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=new_coords, angles=old_angles)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(new_coords[idx]))
        for angle in new_coords[3:]:
            coord_list.append(self._angle2int(angle))
        angles = [self._angle2int(angle) for angle in old_angles]
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)

    def get_drag_fifo(self):
        return self._mesg(ProtocolCode.GET_DRAG_FIFO)

    def set_drag_fifo(self, angles):
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SET_DRAG_FIFO, angles)

    def get_drag_fifo_len(self):
        return self._mesg(ProtocolCode.GET_DRAG_FIFO_LEN)

    def jog_rpy(self, axis, direction, speed, _async=True):
        """Rotate the end point around the fixed axis of the base coordinate system

        Args:
            axis (int): 1 ~ 3. 1 - Roll, 2 - Pitch, 3 - Yaw
            direction (int): 1 - Forward. 0 - Reverse.
            speed (int): 1 ~ 100.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, axis=axis, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_RPY, axis, direction, speed, _async=_async, has_reply=True)

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
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode)
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
        self.calibration_parameters(
            class_name=self.__class__.__name__, pin_no=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_base_io_input(self, pin_no):
        """Get the input IO status of the base

        Args:
            pin_no: (int) pin port number. range 1 ~ 6
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, pin_no=pin_no)
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)

    def set_world_reference(self, coords):
        """Set the world coordinate system

        Args:
            coords: a list of coords value(List[float])
                for mycobot / mecharm / myArm: [x(mm), y, z, rx(angle), ry, rz]\n
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, coord_list)

    def set_identify_mode(self, mode):
        """Set the kinetic parameter identification mode

        Args:
            mode (int): 0 - open. 1 - close
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_IDENTIFY_MODE, mode)

    def get_identify_mode(self):
        """Obtaining kinetic parameter identification mode"""
        return self._mesg(ProtocolCode.GET_IDENTIFY_MODE)

    def write_move_c_r(self, coords, r, speed, rank=0):
        """_summary_

        Args:
            coords (_type_): _description_
            r (_type_): _description_
            speed (_type_): _description_
            rank (_type_): _description_
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coords=coords, r=r, speed=speed, rank=rank)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.WRITE_MOVE_C_R, coord_list, [r * 100], speed, rank, has_reply=True)

    def fourier_trajectories(self, trajectory):
        """Execute dynamic identification trajectory

        Args:
            trajectory (int): 0 ~ 4
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, trajectory=trajectory)
        return self._mesg(ProtocolCode.FOURIER_TRAJECTORIES, trajectory)

    def get_dynamic_parameters(self, add):
        """Setting the dynamics parameters

        Args:
            add (int): 0 ~ 62

        Returns: data * 0.001
        """
        self.calibration_parameters(class_name=self.__class__.__name__, add=add)
        return self._mesg(ProtocolCode.GET_DYNAMIC_PARAMETERS, add)

    def set_dynamic_parameters(self, add, data):
        """Setting the dynamics parameters

        Args:
            add (int): 0 ~ 62
            data : data * 1000

        """
        self.calibration_parameters(class_name=self.__class__.__name__, add=add)
        return self._mesg(ProtocolCode.SET_DYNAMIC_PARAMETERS, add, [data * 1000])

    def identify_print(self):
        res = self.all_debug_data
        self.all_debug_data = []
        return res

    def clear_encoder_error(self, joint_id):
        return self._mesg(ProtocolCode.CLEAR_ENCODER_ERROR, joint_id)

    def get_motors_run_err(self):
        return self._mesg(ProtocolCode.GET_MOTORS_RUN_ERR)

    def get_fusion_parameters(self, rank_mode):
        """Get speed fusion planning parameters
        Args:
            rank_mode: 1 ~ 4
                1: Fusion joint velocity
                2: Fusion joint acceleration
                3: Fusion coordinate velocity
                4: Fusion coordinate acceleration
        Returns:
            1: Fusion joint velocity
            2: Fusion joint acceleration
            3: Fusion coordinate velocity
            4: Fusion coordinate acceleration
        """
        self.calibration_parameters(class_name=self.__class__.__name__, get_rank_mode=rank_mode)
        return self._mesg(ProtocolCode.GET_FUSION_PARAMETERS, rank_mode)

    def set_fusion_parameters(self, rank_mode, value):
        """
        rank_mode: 1 ~ 4
        value: 0 ~ 10000
        """
        return self._mesg(ProtocolCode.SET_FUSION_PARAMETERS, rank_mode, [value])

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION)
