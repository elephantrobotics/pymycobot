# coding=utf-8

import sys
import logging
import time

from pymycobot.log import setup_logging
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode, DataProcessor


class CommandGenerator(DataProcessor):
    """MyCobot Python API
    Annotations:
        * = Chain operation
        x = ??? 

    Supported methods:

        # Overall status
            version()
            power_on()
            power_off()
            is_power_on()
            release_all_servos()
            is_controller_connected()

        # MDI mode and operation
            get_angles()
            send_angle()
            send_angles()
            get_coords()
            send_coords()
            is_in_position()
            is_moving() x

        # JOG mode and operation
            jog_angle()
            jog_coord()
            jog_stop()
            set_encoder()
            get_encoder()
            set_encoders()
            pause()
            resume()
            stop()
            is_paused()

        # Running status and Settings
            get_encoder()
            set_encoder()
            get_encoders()
            set_encoders()
            get_speed()
            set_speed() *
            get_joint_min_angle()
            get_joint_max_angle()

        # Servo control
            is_servo_enable()
            is_all_servo_enable()
            set_servo_data()
            get_servo_data()
            set_servo_calibration()
            release_servo()
            focus_servo()

        # Atom IO
            set_color() *
            set_digital_output()
            get_digital_input()
            set_pwm_mode() x
            set_pwm_output()
            get_gripper_value()
            set_gripper_state() *
            set_gripper_value()
            set_gripper_ini()
            is_gripper_moving()

        # Basic
            set_basic_output()
            get_basic_input()
    """

    def __init__(self, debug=False):
        """
        Args:
            debug    : whether show debug info
        """
        self._version = sys.version_info[:2][0]
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self.calibration_parameters = calibration_parameters

    # System status
    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION, has_reply=True)

    # Overall status
    def power_on(self):
        """Open communication with Atom."""
        return self._mesg(ProtocolCode.POWER_ON)

    def power_off(self):
        """Close communication with Atom."""
        return self._mesg(ProtocolCode.POWER_OFF)

    def is_power_on(self):
        """Adjust robot arm status

        Return:
            1 - power on
            0 - power off
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_POWER_ON, has_reply=True)

    def release_all_servos(self, data=None):
        """Relax all joints
        
        Args:
            data: 1 - Undamping (The default is damping)
        """
        if data is None:
            return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS)
        else:
            return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS, data)

    def is_controller_connected(self):
        """Wether connected with Atom.
        
        Return:
            1 - succeed
            0 - failed
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_CONTROLLER_CONNECTED, has_reply=True)

    # MDI mode and operation
    def get_angles(self):
        """ Get the angle of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)

    def send_angle(self, id, degree, speed):
        """Send one angle of joint to robot arm.

        Args:
            id : Joint id(genre.Angle) int 1-6.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed, has_reply=True)

    def send_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 6.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True)

    def get_coords(self):
        """Get the coords from robot arm, coordinate system based on base.

        Return:
            list : A float list of coord .[x, y, z, rx, ry, rz]

        
        """
        return self._mesg(ProtocolCode.GET_COORDS, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one coord to robot arm. 

        Args:
            id(int) : coord id(genre.Coord) int 1-6.
            coord(float) : coord value, mm
            speed(int) : 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, coord=coord, speed=speed)
        value = self._coord2int(coord) if id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, id, [value], speed, has_reply=True)

    def send_coords(self, coords, speed, mode=None):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]).[x(mm), y, z, rx(angle), ry, rz]\n
            speed : (int) 1 ~ 100
            mode : (int) 0 - angluar, 1 - linear
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        if mode is not None:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, mode)
        else:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True)

    def is_in_position(self, data, id=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords.len 6.
            id: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if id == 1:
            self.calibration_parameters(class_name=self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            self.calibration_parameters(class_name=self.__class__.__name__, angles=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, id, has_reply=True)

    def is_moving(self):
        """Detect if the robot is moving

        Return:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_MOVING, has_reply=True)

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Jog control angle.

        Args:
            joint_id: int 1-6.
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int 1-6
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coord_id=coord_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_increment(self, joint_id, increment, speed):
        """step mode

        Args:
            joint_id: int 1-6.
            increment:
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def pause(self):
        """Pause movement"""
        return self._mesg(ProtocolCode.PAUSE)

    def is_paused(self):
        """Judge whether the manipulator pauses or not.

        Return:
            1 - paused
            0 - not paused
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_PAUSED, has_reply=True)

    def resume(self):
        """Recovery movement"""
        return self._mesg(ProtocolCode.RESUME)

    def stop(self):
        """Stop moving"""
        return self._mesg(ProtocolCode.STOP)

    def set_encoder(self, joint_id, encoder, speed):
        """Set a single joint rotation to the specified potential value.

        Args:
            joint_id: int  1 - 6
                for gripper: Joint id 7
            encoder: The value of the set encoder.
            speed : 1 - 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id, encoder=encoder,
                                    speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder], speed)

    def get_encoder(self, joint_id):
        """Obtain the specified joint potential value.

        Args:
            joint_id: (int) 1 - 6
                for gripper: Joint id 7
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id)
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id, has_reply=True)

    def set_encoders(self, encoders, sp):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            encoders: A encoder list. len 6.
            sp: speed 1 ~ 100
        """
        return self._mesg(ProtocolCode.SET_ENCODERS, encoders, sp)

    def get_encoders(self):
        """Get the six joints of the manipulator

        Return:
            The list of encoders
        """
        return self._mesg(ProtocolCode.GET_ENCODERS, has_reply=True)

    # Running status and Settings

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args: 
            joint_id: 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint
        
        Args:
            joint_id: 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    # Servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id: 1 - 6
        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id, has_reply=True)

    def is_all_servo_enable(self):
        """Detect the connection status of all joints

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE, has_reply=True)

    def set_servo_data(self, servo_id, data_id, value, mode=None):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Serial number of articulated steering gear. 1 - 6
            data_id: Data address.
            value: 0 - 4096
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
        """
        if mode is None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, value=value)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, value)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, value=value,
                                        mode=mode)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, [value], mode)

    def get_servo_data(self, servo_id, data_id, mode=None):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Serial number of articulated steering gear.1 - 6
            data_id: Data address.
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode is not None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, mode=mode)
            return self._mesg(
                ProtocolCode.GET_SERVO_DATA, servo_id, data_id, mode, has_reply=True
            )
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id)
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, servo_id, data_id, has_reply=True
        )

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point, 
            and the corresponding potential value is 2048.

        Args:
            servo_id: Serial number of articulated steering gear. 1 - 6
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def joint_brake(self, joint_id):
        """Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed
        
        Args:
            joint_id:  1 - 6
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.JOINT_BRAKE, joint_id)

    def release_servo(self, servo_id, mode=None):
        """Power off designated servo

        Args:
            servo_id: int 1 - 6

            mode: Default damping, set to 1, cancel damping
        """
        if mode is None:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id, mode)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int 1 - 6
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    def focus_all_servos(self):
        """Power on all servo

        """
        return self._mesg(ProtocolCode.FOCUS_SERVO, 0)

    # Atom IO
    def set_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot arm.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(class_name=self.__class__.__name__, rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def set_digital_output(self, pin_no, pin_signal):
        """Set the terminal atom io status

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """singal value"""
        # TODO pin_no范围未知
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no, has_reply=True)

    def set_gripper_state(self, flag, speed, _type_1=None):
        """Set gripper switch state

        Args:
            flag  (int): 0 - open, 1 - close, 254 - release
            speed (int): 1 ~ 100
            _type_1 (int): default 1
                1 : Adaptive gripper. default to adaptive gripper
                2 : 5 finger dexterous hand
                3 : Parallel gripper, this parameter can be omitted
                4 : Flexible gripper
        """
        if _type_1 is None:
            self.calibration_parameters(class_name=self.__class__.__name__, flag=flag, speed=speed)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, flag=flag, speed=speed, _type_1=_type_1)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed, _type_1)

    def set_gripper_value(self, gripper_value, speed, gripper_type=None):
        """Set gripper value

        Args:
            gripper_value (int): 0 ~ 100
            speed (int): 1 ~ 100
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper, this parameter can be omitted
                4: Flexible gripper
        """
        if gripper_type is not None:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_value=gripper_value, speed=speed,
                                        gripper_type=gripper_type)
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed, gripper_type, has_reply=True)
        else:
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed, has_reply=True)

    def set_gripper_calibration(self):
        """Set the current position to zero, set current position value is `2048`."""
        return self._mesg(ProtocolCode.SET_GRIPPER_CALIBRATION)

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output for M5 version.

        Args:
            pin_no: pin port number.
            pin_signal: 0 / 1
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_basic_input(self, pin_no):
        """Get basic input for M5 version.

        Args:
            pin_no: (int) pin port number.
        """
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no, has_reply=True)

    def set_ssid_pwd(self, account, password):
        """Change connected wifi. (Apply to m5 or seeed)

        Args:
            account: (str) new wifi account.
            password: (str) new wifi password.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, account=account, password=password)

        self._mesg(ProtocolCode.SET_SSID_PWD)  # 先发指令，再发设置的账号密码
        time.sleep(0.02)
        return self._mesg(ProtocolCode.SET_SSID_PWD, account, password, has_reply=True)

    def get_ssid_pwd(self):
        """Get connected wifi account and password. (Apply to m5 or seeed)

        Return: 
            (account, password)
        """
        return self._mesg(ProtocolCode.GET_SSID_PWD, has_reply=True)

    def set_server_port(self, port):
        """Change the connection port of the server.

        Args:
            port: (int) The new connection port of the server.
        """
        return self._mesg(ProtocolCode.SET_SERVER_PORT, port)

    def get_tof_distance(self):
        """ Get the detected distance (Requires external distance detector).

        Return:
            (int) The unit is mm.
        """
        return self._mesg(ProtocolCode.GET_TOF_DISTANCE, has_reply=True)

    def get_error_information(self):
        """Obtaining robot error information
        
        Return:
            0: No error message.
            1 ~ 6: The corresponding joint exceeds the limit position.
            16 ~ 19: Collision protection.
            32: Kinematics inverse solution has no solution.
            33 ~ 34: Linear motion has no adjacent solution.
        """
        return self._mesg(ProtocolCode.GET_ERROR_INFO, has_reply=True)

    def get_basic_version(self):
        """Get basic firmware version"""
        return self._mesg(ProtocolCode.GET_BASIC_VERSION, has_reply=True)



