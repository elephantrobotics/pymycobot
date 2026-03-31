# coding=utf-8
import time

from pymycobot.common import ProtocolCode, ProGripper, FingerGripper
    
class ForceGripper:
    def calibration_parameters(self, *args, **kwargs):
        pass
    def _mesg(self, *args, **kwargs):
        pass
    # 设置力矩手爪
    def set_pro_gripper(self, gripper_address, value=0, has_return=False, gripper_id=14):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    set_gripper_address=gripper_address, gripper_address_value=value)
        # 发送设置力矩手爪的指令
        return self._mesg(ProtocolCode.MERCURY_SET_TOQUE_GRIPPER, gripper_id, [gripper_address], [value],
                          has_return=has_return)

    # 获取力矩手爪
    def get_pro_gripper(self, gripper_address, gripper_id=14):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    get_gripper_address=gripper_address)
        # 发送获取力矩手爪的指令
        return self._mesg(ProtocolCode.MERCURY_GET_TOQUE_GRIPPER, gripper_id, [gripper_address])

    # 设置力矩手爪角度
    def set_pro_gripper_angle(self, gripper_angle, gripper_id=14):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    gripper_angle=gripper_angle)
        # 发送设置力矩手爪角度的指令
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_ANGLE, gripper_angle, gripper_id=gripper_id)

    # 设置力矩手爪打开
    def set_pro_gripper_open(self, gripper_id=14):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_ANGLE, 100, gripper_id=gripper_id)

    def set_pro_gripper_close(self, gripper_id=14):
    # 设置力矩手爪关闭
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_ANGLE, 0, gripper_id=gripper_id)
        # 发送设置力矩手爪关闭的指令

    def get_pro_gripper_angle(self, gripper_id=14):
    # 获取力矩手爪角度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(ProGripper.GET_GRIPPER_ANGLE, gripper_id=gripper_id)
        # 发送获取力矩手爪角度的指令

    def set_pro_gripper_calibration(self, gripper_id=14):
    # 设置力矩手爪校准
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_CALIBRATION, gripper_id=gripper_id)
        # 发送设置力矩手爪校准的指令

    def get_pro_gripper_status(self, gripper_id=14):
    # 获取力矩手爪状态
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(ProGripper.GET_GRIPPER_STATUS, gripper_id=gripper_id)
        # 发送获取力矩手爪状态的指令

    def set_pro_gripper_torque(self, torque, gripper_id=14):
    # 设置力矩手爪扭矩
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, torque=torque)
        # 调用校准参数函数
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_TORQUE, torque, gripper_id=gripper_id)
        # 发送设置力矩手爪扭矩的指令

    def get_pro_gripper_torque(self, gripper_id=14):
    # 获取力矩手爪扭矩
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(ProGripper.GET_GRIPPER_TORQUE, gripper_id=gripper_id)
        # 发送获取力矩手爪扭矩的指令

    def set_pro_gripper_speed(self, speed, gripper_id=14):
    # 设置力矩手爪速度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, speed=speed)
        # 调用校准参数函数
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_SPEED, speed, gripper_id=gripper_id)
        # 发送设置力矩手爪速度的指令

    def get_pro_gripper_speed(self, gripper_id=14):
    # 获取力矩手爪速度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(ProGripper.GET_GRIPPER_SPEED, gripper_id=gripper_id)
        # 发送获取力矩手爪速度的指令

    def set_pro_gripper_abs_angle(self, angle, gripper_id=14):
        # 设置力矩手爪绝对角度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_angle=angle)
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_ABS_ANGLE, angle, has_return=True, gripper_id=gripper_id)
        # 发送设置力矩手爪绝对角度的指令

    def set_pro_gripper_pause(self, gripper_id=14):
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_PAUSE, gripper_id=gripper_id)

    def set_pro_gripper_stop(self, gripper_id=14):
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_STOP, gripper_id=gripper_id)

    def set_pro_gripper_resume(self, gripper_id=14):
        return self.set_pro_gripper(ProGripper.SET_GRIPPER_RESUME, gripper_id=gripper_id)


class ThreeHand:
    def calibration_parameters(self, *args, **kwargs):
        pass
    def _mesg(self, *args, **kwargs):
        pass
    def __set_tool_fittings_value(self, addr, *args, gripper_id=14, **kwargs):
        kwargs["has_replay"] = True
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.MERCURY_SET_TOQUE_GRIPPER, gripper_id, [addr], *args or ([0x00],), **kwargs)

    def __get_tool_fittings_value(self, addr, *args, gripper_id=14, **kwargs):
        kwargs["has_replay"] = True
        return self._mesg(ProtocolCode.MERCURY_GET_TOQUE_GRIPPER, gripper_id, [addr], *args or ([0x00],), **kwargs)

    def get_hand_firmware_major_version(self, gripper_id=14):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_MAJOR_FIRMWARE_VERSION, gripper_id=gripper_id
        )

    def get_hand_firmware_minor_version(self, gripper_id=14):
        return self.__get_tool_fittings_value(FingerGripper.GET_HAND_MINOR_FIRMWARE_VERSION, gripper_id=gripper_id)

    def set_hand_gripper_id(self, new_hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, new_hand_id=new_hand_id)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ID, [new_hand_id], gripper_id=gripper_id
        )

    def get_hand_gripper_id(self, gripper_id=14):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_ID, gripper_id=gripper_id
        )

    def set_hand_gripper_angle(self, hand_id, gripper_angle, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            hand_id (int): 1 ~ 6
            gripper_angle (int): 0 ~ 100
            gripper_id (int) : 1 ~ 254
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_angle=gripper_angle)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ANGLE, [hand_id], [gripper_angle], gripper_id=gripper_id
        )

    def get_hand_gripper_angle(self, hand_id, gripper_id=14):
        """Get the angle of the single joint of the gripper

        Args:
            hand_id (int): 1 ~ 6
            gripper_id (int) : 1 ~ 254

        Return:
            gripper_angle (int): 0 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_ANGLE, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_angles(self, angles, speed, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angles=angles, speed=speed)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ANGLES, [angles], [speed], gripper_id=gripper_id
        )

    def get_hand_gripper_angles(self, gripper_id=14):
        return self.__get_tool_fittings_value(FingerGripper.GET_HAND_ALL_ANGLES, gripper_id)

    def set_hand_gripper_torque(self, hand_id, torque, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, torque=torque)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_TORQUE, [hand_id], [torque], gripper_id=gripper_id
        )

    def get_hand_gripper_torque(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_TORQUE, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_calibrate(self, hand_id, gripper_id=14):
        """ Setting the gripper jaw zero position

        Args:
            hand_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_CALIBRATION, [hand_id], gripper_id=gripper_id
        )

    def get_hand_gripper_status(self, gripper_id=14):
        """ Get the clamping status of the gripper

        Args:
            gripper_id (int): 1 ~ 254

        Return:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_STATUS, gripper_id=gripper_id
        )

    def set_hand_gripper_enabled(self, flag, gripper_id=14):
        """ Set the enable state of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            flag (int): 0 or 1

        """
        self.calibration_parameters(class_name=self.__class__.__name__, flag=flag)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ENABLED, [flag], gripper_id=gripper_id
        )

    def set_hand_gripper_speed(self, hand_id, speed, gripper_id=14):
        """ Set the speed of the gripper

        Args:
            hand_id (int): 1 ~ 6
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, speed=speed)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_SPEED, [hand_id], [speed], gripper_id=gripper_id
        )

    def get_hand_gripper_default_speed(self, hand_id, gripper_id=14):
        """ Get the default speed of the gripper

        Args:
            hand_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254

        Return:
            default speed (int): 1 ~ 100

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_DEFAULT_SPEED, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_p(self, hand_id, value, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_p=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_P, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_p(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_P, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_d(self, hand_id, value, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_d=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_D, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_d(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_D, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_i(self, hand_id, value, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_i=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_I, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_i(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_I, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_min_pressure(self, hand_id, value, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            hand_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, min_pressure=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_MIN_PRESSURE, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_min_pressure(self, hand_id, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            hand_id (int): 1 ~ 6

        Return:
            min pressure value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_MIN_PRESSURE, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_clockwise(self, hand_id, value, gripper_id=14):
        """
        state: 0 or 1, 0 - disable, 1 - enable
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, clockwise=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_CLOCKWISE, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_clockwise(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_CLOCKWISE, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_counterclockwise(self, hand_id, value, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, clockwise=value)
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_COUNTERCLOCKWISE, [hand_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_counterclockwise(self, hand_id, gripper_id=14):
        self.calibration_parameters(class_name=self.__class__.__name__, hand_id=hand_id, gripper_id=gripper_id)
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_COUNTERCLOCKWISE, [hand_id], gripper_id=gripper_id
        )

    def set_hand_gripper_pinch_action_speed_consort(self, pinch_pose, rank_mode, idle_flag=None, gripper_id=14):
        """ Setting the gripper pinching action-speed coordination

        Args:
            pinch_pose (int): 0 ~ 4
                0: All joints return to zero
                1: Index finger and thumb pinch together
                2: Middle finger and thumb pinch together
                3: Index finger and middle finger pinch together
                4: Three fingers together
            rank_mode (int): 0 ~ 5
                The degree of closure,the higher the level, the more closed
            idle_flag (int): default None or 1
                Idle flag. By default, there is no such byte. When this byte is 1, the idle finger can be freely manipulated.
            gripper_id (int): 1 ~ 254

        """

        if idle_flag is None:
            self.calibration_parameters(
                class_name=self.__class__.__name__, pinch_pose=pinch_pose, rank_mode=rank_mode
            )
            return self.__set_tool_fittings_value(
                FingerGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode
            )
        else:
            self.calibration_parameters(
                class_name=self.__class__.__name__, pinch_pose=pinch_pose, rank_mode=rank_mode, idle_flag=idle_flag
            )
            return self.__set_tool_fittings_value(
                FingerGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode, idle_flag,
                gripper_id=gripper_id
            )

    def get_hand_model(self, gripper_id=14):
        """ Get the model number of the three-finger gripper

        Args:
           gripper_id (int): 1 ~ 254

        Return:
            int: 0 ~ 1
            0 - left hand gripper
            1 - right hand gripper

        """
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_MODEL, gripper_id=gripper_id)

class FiveFingerGripper:
    """Pro Five-Finger Dexterous Hand Control Interface Class"""
    def calibration_parameters(self, *args, **kwargs):
        pass

    def tool_serial_write_data(self, arm_id, command):
        pass

    def _modbus_crc(self, data):
        pass

    def _five_fingers_modbus_read(self, arm_id, slave_id, start_reg, reg_count, retry=3):
        """Aoyi Five-Finger Dexterity Hand Universal Modbus Read Holding Register Interface

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            slave_id (int): Device ID (your dexterity hand is 0x02)
            start_reg (int): Starting register address
            reg_count (int): Number of registers to read
            retry (int): Number of retries

        Returns:
            list[int] or -1
        """
        for attempt in range(retry):
            cmd = [
                slave_id,
                0x03,  # Read register
                (start_reg >> 8) & 0xFF,
                start_reg & 0xFF,
                (reg_count >> 8) & 0xFF,
                reg_count & 0xFF
            ]

            # CRC
            cmd += list(self._modbus_crc(bytes(cmd)))
            recv = self.tool_serial_write_data(arm_id, cmd)
            if not recv or recv== -1 or len(recv) < 5:
                # print("Modbus no response:", recv)
                continue

            # Find the Modbus starting point
            start = None
            for i in range(len(recv) - 3):
                if recv[i] == slave_id and recv[i + 1] == 0x03:
                    start = i
                    break

            if start is None:
                # print("Modbus header not found:", recv)
                continue

            recv = recv[start:]

            # Length check
            if len(recv) < 3:
                continue
            byte_count = recv[2]
            expected_len = 3 + byte_count + 2

            if len(recv) < expected_len:
                # print("Incomplete frame:", recv)
                continue
            recv = recv[:expected_len]
            # CRC check
            data = recv[:-2]
            crc_recv = recv[-2:]

            if self._modbus_crc(bytes(data)) != bytes(crc_recv):
                continue

            # Data Analysis Format: [addr, func, byte_count, data..., crc_l, crc_h]
            data_bytes = recv[3:3 + byte_count]

            values = []
            for i in range(0, len(data_bytes), 2):
                val = (data_bytes[i] << 8) | data_bytes[i + 1]
                values.append(val)

            return values
        return -1

    def _five_fingers_modbus_write(self, arm_id, slave_id, start_reg, values):
        reg_count = len(values)
        byte_count = reg_count * 2

        cmd = [
            slave_id,
            0x10,
            (start_reg >> 8) & 0xFF,
            start_reg & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF,
            byte_count
        ]

        for v in values:
            high = (v >> 8) & 0xFF
            low = v & 0xFF
            cmd.extend([high, low])

        cmd += list(self._modbus_crc(bytes(cmd)))

        recv = self.tool_serial_write_data(arm_id, cmd)

        if recv is None or recv == -1:
            return -1

        # Find the starting address for verification
        start = None
        for i in range(len(recv) - 3):
            if recv[i] == slave_id and recv[i + 1] == 0x10:
                start = i
                break
        if start is None:

            return -1

        recv = recv[start:]

        if len(recv) < 8:
            return -1

        data = recv[:-2]
        crc_recv = recv[-2:]

        if self._modbus_crc(bytes(data)) != bytes(crc_recv):
            return -1

        return 1

    def get_five_fingers_angles(self, arm_id, hand_id=2):
        """Read the angle of the five fingers (unit: degrees)

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            hand_id (int): Hand ID, range 0 ~ 255, default 2

        Returns:
            list[float]: For example, [33.54, 173.83, 171.68, 172.1, 174.71, 1.0] represent
                        [thumb bending, index finger, middle finger, ring finger, little finger, and thumb rotation], respectively.
        """

        start_reg = 0x0483
        reg_count = 0x06
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, five_hand_id=hand_id)

        raw = self._five_fingers_modbus_read(arm_id, hand_id, start_reg, reg_count)

        if not raw or raw==-1:
            return -1
        if not isinstance(raw, list):
            return -1
        angles = [round(v / 100.0, 2) for v in raw]

        return angles

    def get_five_fingers_angle(self, arm_id, finger_id, hand_id=2):
        """Read the angle of a single joint of the five fingers

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            finger_id (int): 1 ~ 6
                    1 - thumb bending
                    2 - index finger
                    3 - middle finger
                    4 - ring finger
                    5 - little finger
                    6 - thumb rotation
            hand_id (int): Hand ID, range 0 ~ 255, default 2

        Returns:
            float: angle value
        """

        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, finger_id=finger_id, five_hand_id=hand_id)
        start_reg = 1155 + (finger_id - 1)
        raw = self._five_fingers_modbus_read(arm_id, hand_id, start_reg, 1)

        if not raw or raw==-1:
            return -1
        if not isinstance(raw, list):
            return -1

        angle = raw[0] / 100.0

        return angle

    def set_five_fingers_angles(self, arm_id, fingers_angles, hand_id=2):
        """Set all finger angles.

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            fingers_angles (list): A list of length 6, where J1-J6 represent [thumb bending, index finger, middle finger, ring finger, little finger, thumb rotation] respectively.
                           J1:2.26° ~ 36.76
                           J2:100.22°~178.37°
                           J3:97.81° ~ 176.06°
                           J4:101.38° ~ 176.54°
                           J5:98.84° ~ 174.86°
                           J6:0° ~ 90°
            hand_id (int): Five-finger device ID, range 0 ~ 255, default 2

        Returns:
            int
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, five_fingers_angles=fingers_angles, five_hand_id=hand_id)
        values = [int(a * 100 + 1e-8) for a in fingers_angles]

        return self._five_fingers_modbus_write(arm_id, hand_id, 0x0483, values)

    def set_five_fingers_angle(self, arm_id, finger_id, finger_angle, hand_id=2):
        """Set single finger angles

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            finger_id (int): 1 ~ 6
                    1 - thumb bending
                    2 - index finger
                    3 - middle finger
                    4 - ring finger
                    5 - little finger
                    6 - thumb rotation
            finger_angle (int or float) : angle value
                            J1:2.26° ~ 36.76
                            J2:100.22°~178.37°
                            J3:97.81° ~ 176.06°
                            J4:101.38° ~ 176.54°
                            J5:98.84° ~ 174.86°
                            J6:0° ~ 90°
            hand_id (int): Five-finger device ID, range 0 ~ 255, default 2
        """

        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, finger_id=finger_id,
                                    five_finger_angle=finger_angle, five_hand_id=hand_id)

        reg = 1155 + (finger_id - 1)

        value = int(finger_angle * 100)

        return self._five_fingers_modbus_write(arm_id, hand_id, reg, [value])

    def get_five_fingers_version(self, arm_id, hand_id=2):
        """Read the firmware major and minor version numbers of the five fingers

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            hand_id (int): Hand ID, range 0 ~ 255, default 2

        Returns:
            float: Major and minor version numbers, such as 3.1
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, five_hand_id=hand_id)
        raw = self._five_fingers_modbus_read(arm_id, hand_id, 1001, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1

        val = raw[0]

        major = (val >> 8) & 0xFF
        minor = val & 0xFF

        version = f"{major}.{minor}"
        return version

    def get_five_fingers_modified_version(self, arm_id, hand_id=2):
        """Read firmware and modify version number of the five fingers

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            hand_id (int): Hand ID, range 0 ~ 255, default 2

        Returns:
            int: modified version numbers, such as 79
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, five_hand_id=hand_id)
        raw = self._five_fingers_modbus_read(arm_id, hand_id, 1002, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1
        val = raw[0]
        return val

    def get_five_fingers_hand_id(self, arm_id, hand_id=2):
        """Read hand id of the five fingers

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            hand_id (int): Hand ID, range 0 ~ 255, default 2

        Returns:
            int: hand ID, such as 2
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id, five_hand_id=hand_id)
        raw = self._five_fingers_modbus_read(arm_id, hand_id, 1005, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1

        val = raw[0]

        roh_hand_id = val & 0xFF

        return roh_hand_id

    def set_five_fingers_hand_id(self, arm_id, target_hand_id, hand_id=2):
        """Set hand id of the five fingers

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            target_hand_id (int): Hand ID, range 0 ~ 255, default 2
            hand_id (int): Hand ID, range 0 ~ 255, default 2
        """
        self.calibration_parameters(class_name=self.__class__.__name__, tool_arm_id=arm_id,
                                    target_five_hand_id=target_hand_id, five_hand_id=hand_id)

        return self._five_fingers_modbus_write(arm_id, hand_id, 1005, [target_hand_id])

class L1ForceGripper:
    """L1 Pro Force-Controlled Gripper Control Interface Class"""
    def calibration_parameters(self, *args, **kwargs):
        pass

    def tool_serial_write_data(self, arm_id, command):
        pass

    def _modbus_crc(self, data):
        pass

    def set_tool_serial_timeout(self, arm_id, timeout=10000):
        pass

    def set_tool_serial_baud_rate(self, arm_id, baud_rate):
        pass

    def get_tool_config(self,arm_id):
        pass

    def _send_modbus_command(self, arm_id, gripper_id, func_code, reg_addr, value_high=None, value_low=None, custom_mode=False):
        """
        General Modbus command sending method

        Args:
            arm_id: 1-left arm, 2-right arm
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
        recv = self.tool_serial_write_data(arm_id, cmd)
        if not recv:
            return cmd, -1
        return cmd, recv

    def _check_gripper_id(self, gripper_id, arm_id):

        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, tool_arm_id=arm_id)

    def _write_and_check(self, arm_id, gripper_id, reg_addr, value, custom_mode=False):
        """Write register and verify response robustly (support calibration delay)"""
        self._check_gripper_id(gripper_id, arm_id)
        high, low = (value >> 8) & 0xFF, value & 0xFF
        # Continuously read the response packets, and send a read command to trigger feedback each time.
        _, recv = self._send_modbus_command(arm_id, gripper_id, 0x06, reg_addr, high, low, custom_mode)

        # Basic validity check
        if not isinstance(recv, (list, bytearray)) or len(recv) < 6:
            return -1

        # Two modes have different byte offsets
        # Modbus RTU standard: [id][cmd][regH][regL][valH][valL]...
        # Custom packet       : [fe][fe][len][id][cmd][regH][regL][valH][valL]...
        offset = 1 if not custom_mode else 4

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

    def _read_register(self, arm_id, gripper_id, reg_addr):
        """Reads a register with command verification"""
        self._check_gripper_id(gripper_id, arm_id)

        cmd, recv = self._send_modbus_command(arm_id, gripper_id, 0x03, reg_addr)
        if isinstance(recv, (list, bytearray)) and len(recv) >= 6:
            recv_func = recv[2]
            recv_addr = (recv[3] << 8) | recv[4]
            if recv_func == 0x03 and recv_addr == reg_addr:
                return (recv[5] << 8) | recv[6]
            else:
                return -1

        return -1

    def get_pro_gripper_firmware_version(self, arm_id, gripper_id=14):
        """ Read the firmware major and minor version numbers

        Args:
            arm_id (int):  1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (float): x.x
        """
        val = self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_FIRMWARE_VERSION)
        return val / 10.0 if val >= 0 else -1

    def get_pro_gripper_firmware_modified_version(self, arm_id, gripper_id=14):
        """ Read the firmware revision number

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (int)
        """
        val = self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_FIRMWARE_MODIFY_VERSION)
        return val if val >= 0 else -1

    def set_pro_gripper_id(self, arm_id, target_id, gripper_id=14):
        """ Set the gripper ID

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            target_id (int): Target ID, 1 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, target_id=target_id)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_ID, target_id)

    def get_pro_gripper_id(self, arm_id, gripper_id=14):
        """ Read the gripper ID

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_id (int): 1 ~ 254
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_ID)

    def set_pro_gripper_angle(self, arm_id, gripper_angle, gripper_id=14):
        """ Set the gripper angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_ANGLE, gripper_angle)

    def get_pro_gripper_angle(self, arm_id, gripper_id=14):
        """ Get the gripper angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_angle (int): 0 ~ 100
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_ANGLE)

    def set_pro_gripper_open(self, arm_id, gripper_id=14):
        """ Open the gripper

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self.set_pro_gripper_angle(arm_id, 100, gripper_id)

    def set_pro_gripper_close(self, arm_id, gripper_id=14):
        """ Close the gripper

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self.set_pro_gripper_angle(arm_id, 0, gripper_id)

    def set_pro_gripper_calibration(self, arm_id, gripper_id=14):
        """ Set the gripper zero position

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_CALIBRATION, 0)

    def get_pro_gripper_status(self, arm_id, gripper_id=14):
        """ Get the gripper status

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_STATUS)

    def set_pro_gripper_enabled(self, arm_id, state, gripper_id=14):
        """ Set the gripper enable state

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            state (bool): 0 or 1, 0 - Disable 1 - Enable
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_ENABLED, state)

    def set_pro_gripper_torque(self, arm_id, gripper_torque, gripper_id=14):
        """ Set the gripper torque

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_torque (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_torque=gripper_torque)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_TORQUE, gripper_torque)

    def get_pro_gripper_torque(self, arm_id, gripper_id=14):
        """ Set the gripper torque

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_torque (int): 0 ~ 100
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_TORQUE)

    def set_pro_gripper_speed(self, arm_id, speed, gripper_id=14):
        """ Set the gripper torque

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_SPEED, speed)

    def get_pro_gripper_speed(self, arm_id, gripper_id=14):
        """ Get the gripper speed

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            speed (int): 1 ~ 100
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_SPEED)

    def set_pro_gripper_abs_angle(self, arm_id, gripper_angle, gripper_id=14):
        """ Set the gripper absolute angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_ABS_ANGLE, gripper_angle)

    def set_pro_gripper_io_open_angle(self, arm_id, gripper_angle, gripper_id=14):
        """ Set the gripper IO open angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_IO_OPEN_ANGLE, gripper_angle)

    def get_pro_gripper_io_open_angle(self, arm_id, gripper_id=14):
        """ Get the gripper IO open angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_IO_OPEN_ANGLE)

    def set_pro_gripper_io_close_angle(self, arm_id, gripper_angle, gripper_id=14):
        """ Set the gripper IO close angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_IO_CLOSE_ANGLE, gripper_angle)

    def get_pro_gripper_io_close_angle(self, arm_id, gripper_id=14):
        """ Get the gripper IO close angle

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_IO_CLOSE_ANGLE)

    def set_pro_gripper_mini_pressure(self, arm_id, pressure_value, gripper_id=14):
        """ Set the gripper mini pressure

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            pressure_value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pressure_value=pressure_value)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_MINI_PRESSURE, pressure_value)

    def get_pro_gripper_mini_pressure(self, arm_id, gripper_id=14):
        """ Get the gripper mini pressure

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            mini pressure (int): 0 ~ 254
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_MINI_PRESSURE)

    def set_pro_gripper_protection_current(self, arm_id, current_value, gripper_id=14):
        """ Set the gripper protection current

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            current_value (int): 100 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, current_value=current_value)
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_PROTECTION_CURRENT, current_value)

    def get_pro_gripper_protection_current(self, arm_id, gripper_id=14):
        """ Get the gripper protection current

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            current_value (int): 100 ~ 300
        """
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_PROTECTION_CURRENT)

    def set_pro_gripper_modbus(self, arm_id, state, custom_mode=False, gripper_id=14):
        """ Set the gripper modbus mode

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
            state (int): 0 or 1, 0 - close modbus 1 - open modbus
            custom_mode (bool):
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, 0 -  failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        if custom_mode:
            return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_MODE, state, custom_mode=custom_mode)
        else:
            return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_MODE, state)

    def set_pro_gripper_baud(self, arm_id, baud_rate=0, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
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
        return self._write_and_check(arm_id, gripper_id, ProGripper.MODBUS_SET_BAUD_RATE, baud_rate)

    def get_pro_gripper_baud(self, arm_id, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            arm_id (int): 1 - left arm, 2 - right arm
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
        return self._read_register(arm_id, gripper_id, ProGripper.MODBUS_GET_BAUD_RATE)

    def set_pro_gripper_init(self, arm_id, gripper_id=14):
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
            arm_id (int): 1 - left arm, 2 - right arm
            gripper_id (int): Modbus ID of the gripper, range 1–254.
                Defaults to 14.

        Returns:
            bool: True if initialization succeeds, otherwise False.
        """

        try_bauds = [115200, 1000000]

        print("The gripper is initializing, please wait...")

        self.set_tool_serial_timeout(arm_id, 250)

        test = self.get_pro_gripper_angle(arm_id, gripper_id=gripper_id)
        if test != -1:
            self.set_pro_gripper_modbus(arm_id, 1, gripper_id=gripper_id)  # ensure normal modbus mode

            self.set_pro_gripper_baud(arm_id, 0, gripper_id=gripper_id)  # gripper -> 115200

            self.set_tool_serial_baud_rate(arm_id, 115200)  # end -> 115200
            self.set_tool_serial_timeout(arm_id, 10000)

            print("Gripper Initialization Successful!")
            return True

        for baud in try_bauds:
            # print(f"\n👉 Try the end baud rate: {baud}")
            self.set_tool_serial_baud_rate(arm_id, baud_rate=baud)

            cfg = self.get_tool_config(arm_id)
            # print(f"   485 current config: {cfg}")

            # Read the angle again, applicable to: Baud rate = Correct, Mode = Modbus
            test = self.get_pro_gripper_angle(arm_id, gripper_id=gripper_id)
            # print(f"🔁 Test Modbus to read angle return: {test}")

            if test != -1:
                self.set_pro_gripper_baud(arm_id, 0, gripper_id=gripper_id)
                self.set_tool_serial_baud_rate(arm_id, 115200)
                self.set_tool_serial_timeout(arm_id, 10000)
                print("Gripper Initialization Successful!")
                return True

            # print(f"\n👉 Try the end baud rate again: {baud}")
            self.set_tool_serial_baud_rate(arm_id, baud_rate=baud)
            cfg = self.get_tool_config(arm_id)
            # print(f"   485 current config: {cfg}")
            ret = self.set_pro_gripper_modbus(arm_id, 1, True, gripper_id=gripper_id)
            # print(f"   set_modbus(custom) ret={ret}")

            if ret == 1:
                for i in range(3):
                    t = self.get_pro_gripper_angle(arm_id, gripper_id=gripper_id)
                    # print(f" 🔧 Test angle read[{i}] -> {t}")
                    if t != -1:
                        break
                    time.sleep(0.1)

                if t != -1:
                    self.set_pro_gripper_baud(arm_id, 0, gripper_id=gripper_id)  # change gripper → 115200

                    self.set_tool_serial_baud_rate(arm_id, 115200)  # end back → 115200
                    self.set_tool_serial_timeout(arm_id, 10000)

                    print("Gripper Initialization Successful!")
                    return True

        print("Gripper Initialization Failed!")
        return False