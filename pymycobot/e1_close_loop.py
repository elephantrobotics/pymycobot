# coding=utf-8
import threading
import time
import struct
from datetime import datetime
import re

from pymycobot.common import ProtocolCode, write, read, DataProcessor, FingerGripper
from pymycobot.error import calibration_parameters


class E1CloseLoop(DataProcessor):
    _write = write
    _read = read

    def __init__(self, debug=False):
        super(E1CloseLoop, self).__init__(debug)
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
        self.event = threading.Event()

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
            E1CloseLoop, self)._mesg(genre, *args, **kwargs)
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
        elif genre == ProtocolCode.SET_FRESH_MODE:
            wait_time = 4

        need_break = False
        data = None
        timeout = 0.5
        if genre == ProtocolCode.SET_FRESH_MODE:
            timeout = 4
        elif genre == ProtocolCode.SET_BASE_EXTERNAL_CONTROL:
            timeout = 5
            wait_time = 4
        elif genre == ProtocolCode.TOOL_SERIAL_WRITE_DATA:
            if real_command[7] in [36, 13]:
                timeout = 3
                wait_time = 10
            else:
                wait_time = 0.25
        else:
            timeout = 0.5

        interval_time = time.time()
        is_moving = 0
        check_is_moving_t = 1

        while True and time.time() - t < wait_time:
            # self.event.wait(0.05)
            # self.event.clear()
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
                if isinstance(moving, int) and moving == 0:
                    # print("停止运动，退出")
                    is_moving += 1
                    if is_moving == 1:
                        # 第一次检测到停止，只是标记，不退出
                        # 由于is_moving接口比到位反馈更快，所以第一次收到停止运动后，将下一次的检测时间更改为0.25s，防止此处先退出，返回-2
                        check_is_moving_t = 0.25
                    elif is_moving > 1:
                        # 第二次检测到停止才真正退出
                        with self.lock:
                            if genre in self.write_command:
                                self.write_command.remove(genre)
                        return -2
            time.sleep(0.0001)
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
                        command_log = " ".join("{:02X}".format(b) for b in data)
                        self.log.debug("_read : {}".format(command_log))
                        res = self._process_received(data)
                        if res != []:
                            with self.lock:
                                self.read_command.append(
                                    [res, time.time()])
                            self.event.set()
                else:
                    start = time.time()
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
                        if res == []:
                            # print("res is empty")
                            continue
                        # if datas[3] == 0x5b:
                        #     print("等待加入到读取队列")

                        with self.lock:
                            self.read_command.append([res, time.time()])
                    # print('time-->', time.time() - start)
                            # if datas[3] == 0x5b:
                            # print("加入到读取队列成功")
            except Exception as e:
                # self.log.error("read error: {}".format(traceback.format_exc()))
                pass
            time.sleep(0.0001)

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

    # def write_move_c(self, transpoint, endpoint, speed):
    #     """Circular trajectory motion
    #
    #     Args:
    #         transpoint (list): Arc passing point coordinates
    #         endpoint (list): Arc end point coordinates
    #         speed (int): 1 ~ 100
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
    #     start = []
    #     end = []
    #     for index in range(6):
    #         if index < 3:
    #             start.append(self._coord2int(transpoint[index]))
    #             end.append(self._coord2int(endpoint[index]))
    #         else:
    #             start.append(self._angle2int(transpoint[index]))
    #             end.append(self._angle2int(endpoint[index]))
    #     return self._mesg(ProtocolCode.WRITE_MOVE_C, start, end, speed, has_reply=True)

    def get_angles(self):
        """ Get the angle of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES)

    # def drag_teach_save(self):
    #     """Start recording the dragging teaching point. In order to show the best sports effect, the recording time should not exceed 120 seconds."""
    #     return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_SAVE)
    #
    # def drag_teach_execute(self):
    #     """Start dragging the teaching point and only execute it once."""
    #     return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE, has_reply=True)
    #
    # def drag_teach_pause(self):
    #     """Pause recording of dragging teaching point"""
    #     return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_PAUSE)

    def tool_serial_write_data(self, command):
        """End 485 sends data， Data length range is 1 ~ 45 bytes

        Args:
            command (list) : data instructions

        Return:
            number of bytes received
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_WRITE_DATA, command)

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
        return res

    def power_off(self):
        """Robot power outage"""
        with self.lock:
            self.read_command.clear()
        return self._mesg(ProtocolCode.POWER_OFF)

    def get_robot_type(self):
        """Get robot type
        """
        return self._mesg(ProtocolCode.GET_ROBOT_ID)

    # def get_zero_pos(self):
    #     """Read the zero encoder value
    #
    #     Returns:
    #         list: The values of the zero encoders of the seven joints
    #     """
    #     return self._mesg(ProtocolCode.GET_ZERO_POS)

    def is_init_calibration(self):
        """Check if the robot is initialized for calibration

        Returns:
            bool: True if the robot is initialized for calibration, False otherwise
        """
        return self._mesg(ProtocolCode.IS_INIT_CALIBRATION)

    def over_limit_return_zero(self):
        """Return to zero when the joint is over the limit
        """
        return self._mesg(ProtocolCode.OVER_LIMIT_RETURN_ZERO, has_reply=True)

    # def drag_teach_clean(self):
    #     """clear sample
    #     """
    #     return self._mesg(ProtocolCode.MERCURY_DRAG_TEACH_CLEAN)

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
        return self._mesg(ProtocolCode.MODIFY_VERSION)

    # def set_control_mode(self, mode):
    #     """Set robot motion mode
    #
    #     Args:
    #         mode (int): 0 - location mode, 1 - torque mode
    #
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, mode=mode)
    #     return self._mesg(ProtocolCode.SET_CONTROL_MODE, mode)

    # def get_control_mode(self):
    #     """Get robot motion mode
    #
    #     Returns:
    #         int: 0 - location mode, 1 - torque mode
    #     """
    #     return self._mesg(ProtocolCode.GET_CONTROL_MODE)

    # def set_collision_mode(self, mode):
    #     """Set collision detection mode
    #
    #     Args:
    #         mode (int): 0 - disable, 1 - enable
    #
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, mode=mode)
    #     return self._mesg(ProtocolCode.SET_COLLISION_MODE, mode)

    # def set_collision_threshold(self, joint_id, threshold_value=100):
    #     """Set joint collision threshold
    #
    #     Args:
    #         joint_id (int): joint ID， range 1 ~ 6
    #         threshold_value (int): Collision threshold, range is 50 ~ 250, default is 100, the smaller the value, the easier it is to trigger a collision
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id,
    #                                 threshold_value=threshold_value)
    #     return self._mesg(ProtocolCode.SET_COLLISION_THRESHOLD, joint_id, threshold_value)
    #
    # def get_collision_threshold(self):
    #     """Get joint collision threshold
    #     """
    #     return self._mesg(ProtocolCode.GET_COLLISION_THRESHOLD)

    # def get_vr_mode(self):
    #     """Check if the robot is in VR mode
    #     """
    #     return self._mesg(ProtocolCode.GET_VR_MODE)
    #
    # def set_vr_mode(self, mode):
    #     """Set VR mode
    #
    #     Args:
    #         mode (int): 0 - open, 1 - close
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, vr_mode=mode)
    #     return self._mesg(ProtocolCode.SET_VR_MODE, mode)
    #
    # def get_model_direction(self):
    #     """Get the direction of the robot model
    #     """
    #     return self._mesg(ProtocolCode.GET_MODEL_DIRECTION)
    #
    # def set_model_direction(self, joint_id, direction):
    #     """Set the direction of the robot model
    #
    #     Args:
    #         joint_id (int): joint ID, 1 ~ 7.
    #         direction (int): 1 - forward, 0 - backward
    #     """
    #     return self._mesg(ProtocolCode.SET_MODEL_DIRECTION, joint_id, direction)

    # def get_filter_len(self, rank):
    #     """Get the filter length
    #
    #     Args:
    #         rank (int):
    #             1 : Drag teaching sampling filter
    #             2 : Drag teaching execution filter
    #             3 : Joint velocity fusion filter
    #             4 : Coordinate velocity fusion filter
    #             5 : Drag teaching sampling period
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, rank=rank)
    #     return self._mesg(ProtocolCode.GET_FILTER_LEN, rank)
    #
    # def set_filter_len(self, rank, value=120):
    #     """Set the filter length
    #
    #     Args:
    #         rank (int):
    #             1 : Drag teaching sampling filter
    #             2 : Drag teaching execution filter
    #             3 : Joint velocity fusion filter
    #             4 : Coordinate velocity fusion filter
    #             5 : Drag teaching sampling period
    #         value (int): Filter length, range is 1 ~ 255
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, rank=rank, rank_value=value)
    #     return self._mesg(ProtocolCode.SET_FILTER_LEN, rank, value)

    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO)

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
            joint_id: Serial number of articulated steering gear. Joint id 1 - 7
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
            max_acc (int): maximum acceleration value. Angular acceleration range is 1 ~ 200°/s. Coordinate acceleration range is 1 ~ 400mm/s
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
            joint_id: Joint id 1 - 7

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: Joint id 1 - 7

        Return:
            angle value(float)
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id)

    def set_joint_max_angle(self, joint_id, degree):
        """Set the maximum angle of the joint (must not exceed the maximum angle specified for the joint)

        Args:
            joint_id (int): Joint id 1 - 7
            degree: The angle range of
                joint 1 is -165 ~ 165.
                joint 2 is -55 ~ 95.
                joint 3 is -173 ~ 5.
                joint 4 is -165 ~ 165.
                joint 5 is -20 ~ 265.
                joint 6 is -180 ~ 180.
                joint 6 is -180 ~ 180.

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
            degree: The angle range of
                joint 1 is -165 ~ 165.
                joint 2 is -55 ~ 95.
                joint 3 is -173 ~ 5.
                joint 4 is -165 ~ 165.
                joint 5 is -20 ~ 265.
                joint 6 is -180 ~ 180.
                joint 6 is -180 ~ 180.

        Return:
            1 - success
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, joint_id=joint_id, degree=degree)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, joint_id, degree)

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

    def get_world_reference(self):
        """Get the world coordinate system"""
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE)

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

    # def solve_inv_kinematics(self, new_coords, old_angles):
    #     """_summary_
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, coords=new_coords, angles=old_angles)
    #     coord_list = []
    #     for idx in range(3):
    #         coord_list.append(self._coord2int(new_coords[idx]))
    #     for angle in new_coords[3:]:
    #         coord_list.append(self._angle2int(angle))
    #     angles = [self._angle2int(angle) for angle in old_angles]
    #     return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)

    # def get_collision_mode(self):
    #     """Get collision detection status
    #
    #     Return:
    #         1 - open
    #         0 - close
    #     """
    #     return self._mesg(ProtocolCode.GET_COLLISION_MODE)

    # def get_servo_encoders(self):
    #     return self._mesg(ProtocolCode.GET_ENCODERS)

    # def set_base_io_output(self, pin_no, pin_signal):
    #     """Set the base output IO status
    #
    #     Args:
    #         pin_no: pin port number. range 1 ~ 6
    #         pin_signal: 0 - low. 1 - high.
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, pin_no=pin_no, pin_signal=pin_signal)
    #     return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)
    #
    # def get_base_io_input(self, pin_no):
    #     """Get the input IO status of the base
    #
    #     Args:
    #         pin_no: (int) pin port number. range 1 ~ 6
    #     """
    #     self.calibration_parameters(
    #         class_name=self.__class__.__name__, pin_no=pin_no)
    #     return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)

    # def identify_print(self):
    #     res = self.all_debug_data
    #     self.all_debug_data = []
    #     return res

    def get_motors_run_err(self):
        return self._mesg(ProtocolCode.GET_MOTORS_RUN_ERR)

    # def get_fusion_parameters(self, rank_mode):
    #     """Get speed fusion planning parameters
    #     Args:
    #         rank_mode: 1 ~ 4
    #             1: Fusion joint velocity
    #             2: Fusion joint acceleration
    #             3: Fusion coordinate velocity
    #             4: Fusion coordinate acceleration
    #     Returns:
    #         1: Fusion joint velocity
    #         2: Fusion joint acceleration
    #         3: Fusion coordinate velocity
    #         4: Fusion coordinate acceleration
    #     """
    #     self.calibration_parameters(class_name=self.__class__.__name__, get_rank_mode=rank_mode)
    #     return self._mesg(ProtocolCode.GET_FUSION_PARAMETERS, rank_mode)

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION)

    def get_limit_switch(self):
        """Get the limit switches

        Return:
            list : [Location out of tolerance state, sync state], 0 - close. 1 - open
        """
        res = self._mesg(ProtocolCode.GET_LIMIT_SWITCH)
        if isinstance(res, list) and res[1] == 0:
            self.sync_mode = False
        return res
