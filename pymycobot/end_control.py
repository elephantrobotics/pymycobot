# coding=utf-8

from pymycobot.common import ProtocolCode, ProGripper, FingerGripper
    
class ForceGripper:
    def calibration_parameters(self, *args, **kwargs):
        pass
    def _mesg(self, *args, **kwargs):
        pass
    # 设置力矩手爪
    def set_pro_gripper(self, gripper_id, gripper_address, value=0, has_return=False):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    gripper_address=gripper_address)
        # 发送设置力矩手爪的指令
        return self._mesg(ProtocolCode.MERCURY_SET_TOQUE_GRIPPER, gripper_id, [gripper_address], [value],
                          has_return=has_return)

    # 获取力矩手爪
    def get_pro_gripper(self, gripper_id, gripper_address):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    gripper_address=gripper_address)
        # 发送获取力矩手爪的指令
        return self._mesg(ProtocolCode.MERCURY_GET_TOQUE_GRIPPER, gripper_id, [gripper_address])

    # 设置力矩手爪角度
    def set_pro_gripper_angle(self, gripper_id, gripper_angle):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id,
                                    gripper_angle=gripper_angle)
        # 发送设置力矩手爪角度的指令
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ANGLE, gripper_angle)

    # 设置力矩手爪打开
    def set_pro_gripper_open(self, gripper_id):
        # 调用校准参数函数
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ANGLE, 100)

    def set_pro_gripper_close(self, gripper_id):
    # 设置力矩手爪关闭
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ANGLE, 0)
        # 发送设置力矩手爪关闭的指令

    def get_pro_gripper_angle(self, gripper_id):
    # 获取力矩手爪角度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_ANGLE)
        # 发送获取力矩手爪角度的指令

    def set_pro_gripper_calibration(self, gripper_id):
    # 设置力矩手爪校准
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_CALIBRATION)
        # 发送设置力矩手爪校准的指令

    def get_pro_gripper_status(self, gripper_id):
    # 获取力矩手爪状态
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_STATUS)
        # 发送获取力矩手爪状态的指令

    def set_pro_gripper_torque(self, gripper_id, torque):
    # 设置力矩手爪扭矩
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, torque=torque)
        # 调用校准参数函数
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_TORQUE, torque)
        # 发送设置力矩手爪扭矩的指令

    def get_pro_gripper_torque(self, gripper_id):
    # 获取力矩手爪扭矩
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_TORQUE)
        # 发送获取力矩手爪扭矩的指令

    def set_pro_gripper_speed(self, gripper_id, speed):
    # 设置力矩手爪速度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, speed=speed)
        # 调用校准参数函数
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_SPEED, speed)
        # 发送设置力矩手爪速度的指令

    def get_pro_gripper_speed(self, gripper_id):
    # 获取力矩手爪速度
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        # 调用校准参数函数
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_SPEED)
        # 发送获取力矩手爪速度的指令

    def set_pro_gripper_abs_angle(self, gripper_id, angle):
    # 设置力矩手爪绝对角度
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ABS_ANGLE, angle, has_return=True)
        # 发送设置力矩手爪绝对角度的指令

    def set_pro_gripper_pause(self, gripper_id):
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_PAUSE)

    def set_pro_gripper_stop(self, gripper_id):
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_STOP)

    def set_pro_gripper_resume(self, gripper_id):
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_RESUME)
    

class ThreeHand:
    def _mesg(self, *args, **kwargs):
        pass
    def __set_tool_fittings_value(self, addr, *args, gripper_id=14, **kwargs):
        kwargs["has_replay"] = True
        return self._mesg(ProtocolCode.MERCURY_SET_TOQUE_GRIPPER, gripper_id, [addr], *args or ([0x00],), **kwargs)

    def __get_tool_fittings_value(self, addr, *args, gripper_id=14, **kwargs):
        kwargs["has_replay"] = True
        return self._mesg(ProtocolCode.MERCURY_GET_TOQUE_GRIPPER, gripper_id, [addr], *args or ([0x00],), **kwargs)

    def get_hand_firmware_major_version(self, gripper_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_MAJOR_FIRMWARE_VERSION, gripper_id=gripper_id
        )

    def get_hand_firmware_minor_version(self, gripper_id):
        return self.__get_tool_fittings_value(FingerGripper.GET_HAND_MINOR_FIRMWARE_VERSION, gripper_id=gripper_id)

    def set_hand_gripper_id(self, gripper_id, hand_id):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ID, [hand_id], gripper_id=gripper_id
        )

    def get_hand_gripper_id(self, gripper_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_ID, gripper_id=gripper_id
        )

    def set_hand_gripper_angle(self, gripper_id, joint_id, gripper_angle):
        """Set the angle of the single joint of the gripper

        Args:
            gripper_id (int) : 1 ~ 254
            joint_id (int): 1 ~ 6
            gripper_angle (int): 0 ~ 100
        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ANGLE, [joint_id], [gripper_angle], gripper_id=gripper_id
        )

    def get_hand_gripper_angle(self, gripper_id, joint_id):
        """Get the angle of the single joint of the gripper

        Args:
            gripper_id (int) : 1 ~ 254
            joint_id (int): 1 ~ 6

        Return:
            gripper_angle (int): 0 ~ 100
        """
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_ANGLE, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_angles(self, gripper_id, angles, speed):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ANGLES, [angles], [speed], gripper_id=gripper_id
        )

    def get_hand_gripper_angles(self, gripper_id):
        return self.__get_tool_fittings_value(FingerGripper.GET_HAND_ALL_ANGLES, gripper_id)

    def set_hand_gripper_torque(self, gripper_id, joint_id, value):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_TORQUE, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_torque(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_TORQUE, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_calibrate(self, gripper_id, joint_id):
        """ Setting the gripper jaw zero position

        Args:
            gripper_id (int): 1 ~ 254
            joint_id (int): 1 ~ 6
        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_CALIBRATION, [joint_id], gripper_id=gripper_id
        )

    def get_hand_gripper_status(self, gripper_id):
        """ Get the clamping status of the gripper

        Args:
            gripper_id (int): 1 ~ 254

        Return:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        return self.__set_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_STATUS, gripper_id=gripper_id
        )

    def set_hand_gripper_enabled(self, gripper_id, flag):
        """ Set the enable state of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            flag (int): 1 ~ 6

        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_ENABLED, [flag], gripper_id=gripper_id
        )

    def set_hand_gripper_speed(self, gripper_id, joint_id, speed):
        """ Set the speed of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            joint_id (int): 1 ~ 6
            speed (int): 1 ~ 100

        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_SPEED, [joint_id], [speed], gripper_id=gripper_id
        )

    def get_hand_gripper_default_speed(self, gripper_id, joint_id):
        """ Get the default speed of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            joint_id (int): 1 ~ 6

        Return:
            default speed (int): 1 ~ 100

        """
        return self.__set_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_DEFAULT_SPEED, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_pinch_action(self, gripper_id, pinch_mode):
        """ Set the pinching action of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            pinch_mode (int):
                0 - Index finger and thumb pinch
                1 - Middle finger and thumb pinch
                2 - Three-finger grip
                3 - Two-finger grip
        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_PINCH_ACTION, pinch_mode, gripper_id=gripper_id
        )

    def set_hand_gripper_p(self, gripper_id, joint_id, value):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_P, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_p(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_P, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_d(self, gripper_id, joint_id, value):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_D, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_d(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_D, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_i(self, gripper_id, joint_id, value):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_I, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_i(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_I, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_min_pressure(self, gripper_id, joint_id, value):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254

        """
        return self.__get_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_MIN_PRESSURE, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_min_pressure(self, gripper_id, joint_id):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            gripper_id (int): 1 ~ 254
            joint_id (int): 1 ~ 6

        Return:
            min pressure value (int): 0 ~ 254

        """
        return self.__get_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_MIN_PRESSURE, [joint_id], gripper_id=gripper_id
        )

    def set_hand_gripper_clockwise(self, gripper_id, joint_id, value):
        """
        state: 0 or 1, 0 - disable, 1 - enable
        """
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_CLOCKWISE, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_clockwise(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_CLOCKWISE, joint_id, gripper_id=gripper_id
        )

    def set_hand_gripper_counterclockwise(self, gripper_id, joint_id, value):
        return self.__set_tool_fittings_value(
            FingerGripper.SET_HAND_GRIPPER_COUNTERCLOCKWISE, [joint_id], [value], gripper_id=gripper_id
        )

    def get_hand_gripper_counterclockwise(self, gripper_id, joint_id):
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_GRIPPER_COUNTERCLOCKWISE, [joint_id], gripper_id=gripper_id
        )

    def get_hand_single_pressure_sensor(self, gripper_id, finger_id):
        """ Get the counterclockwise runnable error of the single joint of the gripper

       Args:
           gripper_id (int): 1 ~ 254
           finger_id (int): 1 ~ 5

       Return:
           int: 0 ~ 4096

       """
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_SINGLE_PRESSURE_SENSOR, [finger_id], gripper_id=gripper_id
        )

    def get_hand_all_pressure_sensor(self, gripper_id):
        """ Get the counterclockwise runnable error of the single joint of the gripper

        Args:
           gripper_id (int): 1 ~ 254

        Return:
            int: 0 ~ 4096

        """
        return self.__get_tool_fittings_value(
            FingerGripper.GET_HAND_ALL_PRESSURE_SENSOR, gripper_id=gripper_id
        )

    def set_hand_gripper_pinch_action_speed_consort(self, gripper_id, pinch_pose, rank_mode, idle_flag=None):
        """ Setting the gripper pinching action-speed coordination

        Args:
            gripper_id (int): 1 ~ 254
            pinch_pose (int): 0 ~ 4
                0: All joints return to zero
                1: Index finger and thumb pinch together
                2: Middle finger and thumb pinch together
                3: Index finger and middle finger pinch together
                4: Three fingers together
            rank_mode (int): 0 ~ 5
                The degree of closure,the higher the level, the more closed
            idle_flag (int): default None, 0 ~ 4
                Idle flag. By default, there is no such byte. When this byte is 1, the idle finger can be freely manipulated.

        """
        if idle_flag is None:
            return self.__set_tool_fittings_value(
                FingerGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode, gripper_id=gripper_id
            )
        else:
            return self.__set_tool_fittings_value(
                FingerGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode, idle_flag,
                gripper_id=gripper_id
            )
