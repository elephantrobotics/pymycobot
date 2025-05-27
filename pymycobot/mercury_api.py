# coding=utf-8

import locale

from pymycobot.error import restrict_serial_port
from pymycobot.common import ProtocolCode, FingerGripper
from pymycobot.robot_info import _interpret_status_code
from pymycobot.close_loop import CloseLoop


class MercuryCommandGenerator(CloseLoop):
    def __init__(self, debug=False):
        super(MercuryCommandGenerator, self).__init__(debug)
        try:
            import numpy as np
        except ImportError:
            raise ImportError("Please install numpy")
        # 同步模式
        self.language, _ = locale.getdefaultlocale()
        if self.language not in ["zh_CN", "en_US"]:
            self.language = "en_US"
        self.max_joint, self.min_joint = 0, 0

    def _joint_limit_init(self):
        max_joint = np.zeros(7)
        min_joint = np.zeros(7)
        for i in range(7):
            max_joint[i] = self.get_joint_max_angle(i + 1)
            min_joint[i] = self.get_joint_min_angle(i + 1)
        return max_joint, min_joint

    def _joint_limit_judge(self, angles):
        offset = 3
        try:
            for i in range(7):
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
        return_data = super(MercuryCommandGenerator, self)._mesg(
            genre, *args, **kwargs)
        if isinstance(return_data, tuple):
            valid_data, data_len = return_data
        elif isinstance(return_data, int):
            return return_data
        else:
            return None
        res = []
        hand_address = None
        if genre == ProtocolCode.MERCURY_GET_TOQUE_GRIPPER:
            if len(valid_data) > 1:
                hand_address = self._decode_int16(valid_data[1:3])
                valid_data = valid_data[3:]
                data_len -= 3
            else:
                return valid_data[0]
        # print(data_len, valid_data)
        if genre == ProtocolCode.TOOL_SERIAL_READ_DATA:
            for i in range(data_len):
                res.append(valid_data[i])
        elif data_len in [6, 8, 12, 14, 16, 20, 24, 26, 60]:
            if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1, n):
                    res.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                i = 0
                while i < data_len:
                    byte_value = int.from_bytes(
                        valid_data[i:i + 4], byteorder='big', signed=True)
                    i += 4
                    res.append(byte_value)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES,
                                             ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    res.append(valid_data[i])
            else:
                
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i: header_i + 2]
                    res.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return self._decode_int8(valid_data[1:2])
            elif genre in [ProtocolCode.GET_LIMIT_SWITCH]:
                for i in valid_data:
                    res.append(i)
            elif genre in [ProtocolCode.GET_ERROR_INFO]:
                res.append(self._decode_int8(valid_data[1:]))
            else:
                res.append(self._decode_int16(valid_data))
                if hand_address in [FingerGripper.GET_HAND_MAJOR_FIRMWARE_VERSION]:
                    res[0] /=10
        elif data_len == 3:
            if genre in [ProtocolCode.GET_DIGITAL_INPUTS]:
                for i in valid_data:
                    res.append(i)
            else:
                res.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.GET_ENCODER, ProtocolCode.GET_DYNAMIC_PARAMETERS]:
                byte_value = int.from_bytes(
                    valid_data, byteorder='big', signed=True)
                if genre in [ProtocolCode.GET_DYNAMIC_PARAMETERS]:
                    byte_value /= 1000
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
        elif data_len in [28, 32]:  # 28 left get_zero_pos, 32 right get_zero_pos
            for i in range(0, data_len, 4):
                byte_value = int.from_bytes(
                    valid_data[i:i + 4], byteorder='big', signed=True)
                res.append(byte_value)
        elif data_len == 40 and genre == ProtocolCode.GET_ANGLES_COORDS:
            i = 0
            while i < data_len:
                if i < 28:
                    byte_value = int.from_bytes(
                        valid_data[i:i + 4], byteorder='big', signed=True)
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
        elif data_len == 41 and genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            # 图灵右臂上位机错误
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
        elif data_len in [37, 46]:
            # X1 left status
            if data_len == 37:
                add_index = 9
            else:
                add_index = 10
            i = 0
            res = []
            while i < data_len:
                if i < add_index:
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
            i = 0
            while i < data_len:
                byte_value_send = int.from_bytes(
                    valid_data[i:i + 4], byteorder='big', signed=True)
                i += 4
                byte_value_current = int.from_bytes(
                    valid_data[i:i + 4], byteorder='big', signed=True)
                res.append([byte_value_send, byte_value_current])
                i += 4
        elif data_len == 48:
            i = 0
            j = 0
            res = [0, 0]
            angles = []
            coords = []
            while i < data_len:
                if i < 28:
                    byte_value_send = int.from_bytes(
                        valid_data[i:i + 4], byteorder='big', signed=True)
                    angles.append(self._int2angle(byte_value_send))
                    i += 4
                elif i < 40:

                    one = valid_data[i: i + 2]
                    one = self._decode_int16(one)
                    if j < 3:
                        one = self._int2coord(one)
                    else:
                        one = self._int2angle(one)
                    coords.append(one)
                    j += 1
                    i += 2
                else:
                    res.append(valid_data[i])
                    i += 1
            res[0] = angles
            res[1] = coords
            res[2] = 0
            res[3] = 0
        # elif genre == ProtocolCode.MERCURY_GET_TOQUE_GRIPPER:
        #     res = self._decode_int16(valid_data[-2:])
        #     address = self._decode_int16(valid_data[1:3])
        #     if address == 1:
        #         res /= 10
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
            ProtocolCode.GET_COLLISION_MODE,
            ProtocolCode.GET_DYNAMIC_PARAMETERS,
            ProtocolCode.GET_ERROR_INFO
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
        elif genre in [ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, ProtocolCode.MERCURY_GET_POS_OVER_SHOOT,
                       ProtocolCode.GET_SERVO_CW]:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.GET_ANGLES:
            return [self._int3angle(angle) for angle in res]
        elif genre == ProtocolCode.SOLVE_INV_KINEMATICS:
            return [self._int2angle(angle) for angle in res]
        elif genre == ProtocolCode.COBOTX_GET_ANGLE:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 23:
                index = 9
            else:
                index = 10
            for i in range(index, len(res)):
                if res[i] != 0:
                    data = bin(res[i])[2:]
                    res[i] = []
                    while len(data) != 16:
                        data = "0" + data
                    for j in range(16):
                        if data[j] != "0":
                            error_id = 15 - j
                            res[i].append(error_id)
                    if res[i] == []:
                        res[i] = 0
            return res
        elif genre == ProtocolCode.GET_MOTORS_RUN_ERR:
            for i in range(len(res)):
                if res[i] != 0:
                    data = bin(res[i])[2:]
                    res[i] = []
                    while len(data) != 16:
                        data = "0" + data
                    for j in range(16):
                        if data[j] != "0":
                            error_id = 15 - j
                            res[i].append(error_id)
                    if res[i] == []:
                        res[i] = 0
            return res
        else:
            if isinstance(res, list) and len(res) == 1:
                return res[0]
            return res

    @restrict_serial_port
    def get_base_coords(self):
        """get base coords"""
        return self._mesg(ProtocolCode.MERCURY_GET_BASE_COORDS)

    @restrict_serial_port
    def send_base_coord(self, coord_id, base_coord, speed, _async=False):
        """Single coordinate control with the torso base as the coordinate system

        Args:
            coord_id (int): 1 to 6 correspond to x, y, z, rx, ry, rz
            base_coord (float): coord value.
                The coord range of `X` is -351.11 ~ 566.92.
                The coord range of `Y` is -645.91 ~ 272.12.
                The coord range of `Y` is -262.91 ~ 655.13.
                The coord range of `RX` is -180 ~ 180.
                The coord range of `RY` is -180 ~ 180.
                The coord range of `RZ` is -180 ~ 180.
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, coord_id=coord_id, base_coord=base_coord, speed=speed, serial_port=self._serial_port.port)
        if coord_id < 4:
            coord = self._coord2int(base_coord)
        else:
            coord = self._angle2int(base_coord)
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORD, coord_id, [coord], speed, _async=_async, has_reply=True)

    @restrict_serial_port
    def send_base_coords(self, base_coords, speed, _async=False):
        """Full coordinate control

        Args:
            base_coords (list): coordinate value, [x, y, z, rx, ry, rz]
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, base_coords=base_coords, speed=speed, serial_port=self._serial_port.port)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(base_coords[idx]))
        for angle in base_coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORDS, coord_list, speed, _async=_async, has_reply=True)

    @restrict_serial_port
    def jog_base_coord(self, axis, direction, speed, _async=True):
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
        return self._mesg(ProtocolCode.MERCURY_JOG_BASE_COORD, axis, direction, speed, _async=_async, has_reply=True)

    @restrict_serial_port
    def set_servo_cw(self, head_id, err_angle):
        """Set the joint in-place feedback error angle

        Args:
            head_id (int): Joint ID, 11 or 12.
            err_angle (float): Error range is 0 ~ 5.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, head_id=head_id, err_angle=err_angle)
        return self._mesg(ProtocolCode.SET_SERVO_CW, head_id, [self._angle2int(err_angle)])

    @restrict_serial_port
    def get_servo_cw(self, head_id):
        """Get the joint in-place feedback error angle

        Args:
            head_id (int): Joint ID, 11 or 12.

        Returns:
            float: Error angle, range is 0 ~ 5.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, head_id=head_id)
        return self._mesg(ProtocolCode.GET_SERVO_CW, head_id)

    @restrict_serial_port
    def clear_waist_queue(self):
        """Clear the cache points of the three motors in the torso
        """
        return self._mesg(ProtocolCode.CLEAR_WAIST_QUEUE)

    @restrict_serial_port
    def jog_base_increment_coord(self, axis_id, increment, speed, _async=True):
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
        return self._mesg(ProtocolCode.JOG_BASE_INCREMENT_COORD, axis_id, coord_list, speed, has_reply=True,
                          _async=_async)
        
    def is_in_position(self, data, mode=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords. angles len 6, coords len 6.
            mode: 0 - angles, 1 - coords, 2 - base coords

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if mode in [1,2]:
            if mode == 2:
                self.calibration_parameters(class_name=self.__class__.__name__, base_coords=data, serial_port=self._serial_port.port)
            else:
                self.calibration_parameters(class_name=self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif mode == 0:
            self.calibration_parameters(class_name=self.__class__.__name__, angles=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("mode is not right, please input 0 or 1 or 2")
        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, mode)
    
    def write_waist_sync(self, current_angle, target_angle, speed):
        """_summary_

        Args:
            current_angle (_type_): _description_
            target_angle (_type_): _description_
            speed (_type_): _description_
        """
        self.calibration_parameters(class_name=self.__class__.__name__,
                                    current_angle=current_angle, target_angle=target_angle, speed=speed)
        return self._mesg(ProtocolCode.WRITE_WAIST_SYNC, [self._angle2int(current_angle)], [self._angle2int(target_angle)], speed)

    @restrict_serial_port
    def jog_base_rpy(self, axis, direction, speed, _async=True):
        """Rotate the end point around the fixed axis of the base coordinate system

        Args:
            axis (int): 1 ~ 3. 1 - Roll, 2 - Pitch, 3 - Yaw
            direction (int): 1 - Forward. 0 - Reverse.
            speed (int): 1 ~ 100.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, axis=axis, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_BASE_RPY, axis, direction, speed, _async=_async, has_reply=True)

