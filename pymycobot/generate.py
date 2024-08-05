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
            set_pin_mode()
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
    def get_robot_version(self):  # TODO: test method <2021-03-11, yourname> #
        """Get cobot version

        Return:
            mycobot   : 1
            mycobotPro: 101
        """
        return self._mesg(ProtocolCode.ROBOT_VERSION, has_reply=True)

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION, has_reply=True)

    def get_robot_id(self):
        """get robot id"""
        return self._mesg(ProtocolCode.GET_ROBOT_ID, has_reply=True)

    def set_robot_id(self, id):
        """set robot id
        
        Args:
            id(int): 0 ~ 255
        """
        return self._mesg(ProtocolCode.SET_ROBOT_ID, id)

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

    def read_next_error(self):
        """Robot Error Detection
        
        Return:
            list len 7.
            0 : No abnormality
            1 : Communication disconnected
            2 : Unstable communication
            3 : Servo abnormality
        """
        return self._mesg(ProtocolCode.READ_NEXT_ERROR, has_reply=True)    
    
    def set_free_mode(self, flag):
        """set to free mode
        
        Args:
            flag: 0/1
        """
        self.calibration_parameters(class_name = self.__class__.__name__, flag = flag)
        return self._mesg(ProtocolCode.SET_FREE_MODE, flag)

    def is_free_mode(self):
        """Check if it is free mode
        
        Return:
            0/1
        """
        return self._process_single(self._mesg(ProtocolCode.IS_FREE_MODE, has_reply=True))

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
            id : Joint id(genre.Angle)
                    for mycobot / mecharm: int 1-6.
                    for mypalletizer: int 1-4.
                    for myArm or Mercury: Joint id 1 - 7.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed)

    # @check_parameters(Command.SEND_ANGLES)
    def send_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]).
                        for mycobot / mecharm: len 6.
                        for mypalletizer: len 4.
                        for myArm: len 7.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name = self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed)

    def get_coords(self):
        """Get the coords from robot arm, coordinate system based on base.

        Return:
            list : A float list of coord .
                for mycobot / mecharm: [x, y, z, rx, ry, rz].
                for mypalletizer: [x, y, z, θ].
                for myArm: [x, y, z, rx, ry, rz].
        
        """
        return self._mesg(ProtocolCode.GET_COORDS, has_reply=True)

    def send_coord(self, id, coord, speed):
        """Send one coord to robot arm. 

        Args:
            id(int) : coord id(genre.Coord)\n
                        for mycobot / mecharm / myArm: int 1-6.\n
                        for mypalletizer: int 1-4.
            coord(float) : coord value, mm
            speed(int) : 1 ~ 100
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=id, coord = coord, speed=speed)
        value = self._coord2int(coord) if id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, id, [value], speed)

    def send_coords(self, coords, speed, mode=None):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]).
                        for mycobot / mecharm / myArm: [x(mm), y, z, rx(angle), ry, rz]\n
                        for mypalletizer: [x, y, z, θ]
            speed : (int) 1 ~ 100
            mode : (int) 0 - angluar, 1 - linear (mypalletizer 340 does not require this parameter)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        if mode is not None:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, mode)
        else:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed)

    def is_in_position(self, data, id=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords.
                    for mycobot / mecharm: len 6.
                    for mypalletizer: len 4
                    for myArm: angles len 7, coords len 6.
            id: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if id == 1:
            self.calibration_parameters(class_name = self.__class__.__name__, coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            self.calibration_parameters(class_name = self.__class__.__name__, angles=data)
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
            joint_id: int
                    for mycobot / mecharm: int 1-6.\n
                    for mypalletizer: int 1-4.
                    for myArm: Joint id 1 - 7.
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id = joint_id, direction = direction)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int
                    for mycobot / mecharm / myArm: int 1-6.\n
                    for mypalletizer: int 1-4.
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, coord_id = coord_id, direction = direction)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)
    
    def jog_absolute(self, joint_id, angle, speed):
        """Jog absolute angle

        Args:
            joint_id: int
                    for mycobot / mecharm: int 1-6.\n
                    for mypalletizer: int 1-4.
                    for myArm: Joint id 1 - 7.
            angle: -180 ~ 180
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id = joint_id, angle = angle, speed = speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, joint_id, [self._angle2int(angle)], speed)
    
    def jog_increment(self, joint_id, increment, speed):
        """step mode

        Args:
            joint_id:
                for mycobot / mecharm: int 1-6.
                for mypalletizer: int 1-4.
                for myArm: Joint id 1 - 7.
            increment: 
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id = joint_id, speed = speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def jog_stop(self):
        """Stop jog moving (Atom 7.0 Delete this interface)"""
        return self._mesg(ProtocolCode.JOG_STOP)

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
            joint_id: int
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for mycobot gripper: Joint id 7
                for myArm: Joint id 1 - 7.
            encoder: The value of the set encoder.
            speed : 1 - 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id, encoder=encoder,
                                    speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder], speed)

    def get_encoder(self, joint_id):
        """Obtain the specified joint potential value.

        Args:
            joint_id: (int)
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for mycobot gripper: Joint id 7
                for myArm: Joint id 1 - 7.
        """
        self.calibration_parameters(class_name = self.__class__.__name__, encode_id = joint_id)
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id, has_reply=True)

    def set_encoders(self, encoders, sp):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            encoders: A encoder list.
                for mycobot / mecharm: len 6.
                for mypalletizer: len 4
                for myArm: len 7.
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
    def get_speed(self):
        """Get speed

        Return: 
            int
        """
        return self._mesg(ProtocolCode.GET_SPEED, has_reply=True)

    def set_speed(self, speed):
        """Set speed value

        Args:
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(class_name = self.__class__.__name__, speed=speed)
        return self._mesg(ProtocolCode.SET_SPEED, speed)

    """
    def get_feed_override(self):
        return self._process_single(
            self._mesg(Command.GET_FEED_OVERRIDE, has_reply=True)
        )
    """


    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args: 
            joint_id:
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: Joint id 1 - 7.

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint
        
        Args:
            joint_id:
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: Joint id 1 - 7.

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    # Servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id:
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: Joint id 1 - 7.

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
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
            servo_id: Serial number of articulated steering gear.
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
            data_id: Data address.
            value: 0 - 4096
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
        """
        if mode is None:
            self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id, address=data_id, value=value)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, value)
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id, address=data_id, value=value,
                                        mode=mode)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, [value], mode)

    def get_servo_data(self, servo_id, data_id, mode=None):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Serial number of articulated steering gear.
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
            data_id: Data address.
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode is not None:
            self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id, address=data_id, mode=mode)
            return self._mesg(
                ProtocolCode.GET_SERVO_DATA, servo_id, data_id, mode, has_reply=True
            )
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id, address=data_id)
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, servo_id, data_id, has_reply=True
        )

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point, 
            and the corresponding potential value is 2048.

        Args:
            servo_id: Serial number of articulated steering gear.
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)
    
    def joint_brake(self, joint_id):
        """Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed
        
        Args:
            joint_id: 
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.JOINT_BRAKE, joint_id)

    def release_servo(self, servo_id, mode=None):
        """Power off designated servo

        Args:
            servo_id: int
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
            mode: Default damping, set to 1, cancel damping
        """
        if mode is None:
            self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

        else:
            self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id, mode)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for myArm: joint id 1 - 7
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    # Atom IO
    def set_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot arm.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(class_name = self.__class__.__name__, rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int): pin number.
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_mode=pin_mode)
        return self._mesg(ProtocolCode.SET_PIN_MODE, pin_no, pin_mode)

    def set_digital_output(self, pin_no, pin_signal):
        """Set the terminal atom io status

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """singal value"""
        # TODO pin_no范围未知
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no, has_reply=True)

    # def set_pwm_mode(self, mode):
    #     # TODO 280协议中无
    #     return self._mesg(ProtocolCode.SET_PWM_MODE, mode)

    def set_pwm_output(self, channel, frequency, pin_val):
        """ PWM control 

        Args:
            channel (int): IO number.
            frequency (int): clock frequency
            pin_val (int): Duty cycle 0 ~ 256; 128 means 50%
        """
        return self._mesg(ProtocolCode.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    def get_gripper_value(self, gripper_type=None):
        """Get the value of gripper.
        
        Args:
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper
                4: Flexible gripper

        Return: 
            gripper value (int)
        """
        if gripper_type is None:
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, has_reply=True)
        else:
            self.calibration_parameters(class_name = self.__class__.__name__, gripper_type=gripper_type)
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, gripper_type, has_reply=True)
            

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
            self.calibration_parameters(class_name = self.__class__.__name__, flag=flag, speed=speed)
            return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)
        else:
            self.calibration_parameters(class_name = self.__class__.__name__, flag=flag, speed=speed, _type_1=_type_1)
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
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed, gripper_type)
        else:
            return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, gripper_value, speed)
            

    def set_gripper_calibration(self):
        """Set the current position to zero, set current position value is `2048`."""
        return self._mesg(ProtocolCode.SET_GRIPPER_CALIBRATION)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 - not moving
            1 - is moving
            -1- error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output for M5 version.

        Args:
            pin_no: pin port number.
            pin_signal: 0 / 1
        """
        self.calibration_parameters(class_name = self.__class__.__name__, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal, has_reply=True)

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
        self.calibration_parameters(class_name = self.__class__.__name__, account=account, password=password)
        
        self._mesg(ProtocolCode.SET_SSID_PWD) # 先发指令，再发设置的账号密码
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
    
    def set_tool_reference(self, coords):
        """Set tool coordinate system
        
        Args:
            coords: a list of coords value(List[float])
                for mycobot / mecharm:[x(mm), y, z, rx(angle), ry, rz]
                for mypalletizer 340: [x, y, z]
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
        return self._mesg(ProtocolCode.GET_TOOL_REFERENCE, has_reply=True)
    
    def set_world_reference(self, coords):
        """Set the world coordinate system
        
        Args:
            coords: a list of coords value(List[float])
                for mycobot / mecharm / myArm: [x(mm), y, z, rx(angle), ry, rz]\n
        """
        self.calibration_parameters(class_name = self.__class__.__name__, coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, coord_list)
    
    def get_world_reference(self):
        """Get the world coordinate system"""
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE, has_reply=True)
    
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
        return self._mesg(ProtocolCode.GET_REFERENCE_FRAME, has_reply=True)
    
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
        return self._mesg(ProtocolCode.GET_MOVEMENT_TYPE, has_reply=True)
    
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
        return self._mesg(ProtocolCode.GET_END_TYPE, has_reply=True)
        
    
    def get_plan_speed(self):
        """Get planning speed
        
        Args:
            return: 
                [movel planning speed, movej planning speed].
        """
        return self._mesg(ProtocolCode.GET_PLAN_SPEED, has_reply=True)
    
    def get_plan_acceleration(self):
        """Get planning acceleration
        
        Args:
            return: 
                [movel planning acceleration, movej planning acceleration].
        """
        return self._mesg(ProtocolCode.GET_PLAN_ACCELERATION, has_reply=True)
    
    def set_plan_speed(self, speed, is_linear):
        """Set planning speed
        
        Args:
            speed (int): (1 ~ 100).
            is_linear: 0 -> joint 1 -> straight line
        """
        return self._mesg(ProtocolCode.SET_PLAN_SPEED, speed, is_linear)
    
    def set_plan_acceleration(self, acceleration, is_linear):
        """Set planning acceleration
        
        Args:
            acceleration (int): (1 ~ 100).
            is_linear: 0 -> joint 1 -> straight line
        """
        self.calibration_parameters(class_name = self.__class__.__name__, acceleration=acceleration, is_linear=is_linear)
        return self._mesg(ProtocolCode.SET_PLAN_ACCELERATION, acceleration, is_linear)
    
    def get_servo_speeds(self):
        """Get joint speed (Only for mycobot 320)
        
        Return: 
            unit step/s
        """
        return self._mesg(ProtocolCode.GET_SERVO_SPEED, has_reply=True)
    
    def get_servo_currents(self):
        """Get joint current (Only for mycobot 320)
        
        Return: 
            0 ~ 3250 mA
        """
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS, has_reply=True)
    
    def get_servo_voltages(self):
        """Get joint voltages (Only for mycobot 320)
        
        Return: 
            volts < 24 V
        """
        return self._mesg(ProtocolCode.GET_SERVO_VOLTAGES, has_reply=True)

    def get_servo_status(self):
        """Get joint status (Only for mycobot 320)
        
        Return: 
            [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error, a value of 1 indicates an error
        """
        return self._mesg(ProtocolCode.GET_SERVO_STATUS, has_reply=True)

    def get_servo_temps(self):
        """Get joint temperature (Only for mycobot 320)"""
        return self._mesg(ProtocolCode.GET_SERVO_TEMPS, has_reply=True)
    
    def set_joint_max(self, id, angle):
        """Set the joint maximum angle
        
        Args:
            id: int.
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for mycobot gripper: Joint id 7
                for myArm: Joint id 1 - 7.
            angle: 0 ~ 180 
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)
    
    def set_joint_min(self, id, angle):
        """Set the joint minimum angle
        
        Args:
            id: int.
                for mycobot / mecharm: Joint id 1 - 6
                for mypalletizer: Joint id 1 - 4
                for mycobot gripper: Joint id 7
                for myArm: Joint id 1 - 7.
            angle: 0 ~ 180 
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)
    
    def init_eletric_gripper(self): # TODO 22-5-19 need test
        """Electric gripper initialization (it needs to be initialized once after inserting and removing the gripper) (only for 320)"""
        return self._mesg(ProtocolCode.INIT_ELETRIC_GRIPPER)
    
    def set_eletric_gripper(self, status):  # TODO 22-5-19 need test
        """Set Electric Gripper Mode (only for 320)
        
        Args:
            status: 0 - open, 1 - close.
        """
        self.calibration_parameters(class_name = self.__class__.__name__, status=status)
        return self._mesg(ProtocolCode.SET_ELETRIC_GRIPPER, status)
    
    def set_encoders_drag(self, encoders, speeds):  # TODO 22-5-19 need test
        """Send all encoders and speeds
        
        Args:
            encoders: encoders list.
            speeds: Obtained by the get_servo_speeds() method 
        """
        self.calibration_parameters(class_name = self.__class__.__name__, encoders=encoders, speeds=speeds)
        return self._mesg(ProtocolCode.SET_ENCODERS_DRAG, encoders, speeds)
    
    def set_fresh_mode(self, mode):   # TODO 22-5-19 need test
        """Set command refresh mode
        
        Args:
            mode: int.
                1 - Always execute the latest command first.
                0 - Execute instructions sequentially in the form of a queue.
        """
        self.calibration_parameters(class_name = self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, mode)
        
    def get_fresh_mode(self):
        """Query sports mode"""
        return self._mesg(ProtocolCode.GET_FRESH_MODE, has_reply = True)
    
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
        return self._mesg(ProtocolCode.GET_GRIPPER_MODE, has_reply = True)
    
    def get_servo_last_pdi(self, id):
        """Obtain the pdi of a single steering gear before modification
        
        Args:
            id: 1 - 6
        """
        self.calibration_parameters(class_name = self.__class__.__name__, servo_id_pdi=id)
        return self._mesg(ProtocolCode.GET_SERVO_LASTPDI, id, has_reply = True)
    
    def get_error_information(self):
        """Obtaining robot error information
        
        Return:
            0: No error message.
            1 ~ 6: The corresponding joint exceeds the limit position.
            16 ~ 19: Collision protection.
            32: Kinematics inverse solution has no solution.
            33 ~ 34: Linear motion has no adjacent solution.
        """
        return self._mesg(ProtocolCode.GET_ERROR_INFO, has_reply = True)
    
    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO, has_reply = True)
    
    def move_round(self):
        """Drive the 9g steering gear clockwise for one revolution
        """
        return self._mesg(ProtocolCode.move_round)


    def get_basic_version(self):
        """Get basic firmware version"""
        return self._mesg(ProtocolCode.GET_BASIC_VERSION, has_reply = True)
    
    def set_transponder_mode(self, mode):
        """Set basic communication mode
        
        Args:
            mode: 0 - Turn off transparent transmission，1 - Open transparent transmission
        """
        self.calibration_parameters(class_name = self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_COMMUNICATE_MODE, mode, has_reply = True)
    
    def get_transponder_mode(self):
        return self._mesg(ProtocolCode.GET_COMMUNICATE_MODE, has_reply = True)
    
    def get_angles_coords(self):
        """Get basic communication mode"""
        return self._mesg(ProtocolCode.GET_ANGLES_COORDS, has_reply = True)
    
    def get_atom_version(self):
        """Get atom firmware version.

        Returns:
            float: version number.
        """
        return self._mesg(ProtocolCode.GET_ATOM_VERSION, has_reply = True)
    
    def set_HTS_gripper_torque(self, torque):
        """Set new adaptive gripper torque

        Args:
            torque (int): 150 ~ 980
            
        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(class_name = self.__class__.__name__, torque=torque)
        return  self._mesg(ProtocolCode.SetHTSGripperTorque, [torque], has_reply = True)
    
    def get_HTS_gripper_torque(self):
        """Get gripper torque

        Returns:
            int: 150 ~ 980
        """
        return self._mesg(ProtocolCode.GetHTSGripperTorque, has_reply = True)
    
    def get_gripper_protect_current(self):
        """Get the gripper protection current

        Returns:
            int: 1 ~ 500
        """
        return self._mesg(ProtocolCode.GetGripperProtectCurrent, has_reply = True)
    
    def init_gripper(self):
        """Initialize gripper

        Returns:
            int: 0 or 1 (1 - success)
        """
        return self._mesg(ProtocolCode.InitGripper, has_reply = True)
    
    def set_gripper_protect_current(self, current):
        """Set the gripper protection current

        Args:
            current (int): 1 ~ 500
        """
        self.calibration_parameters(class_name = self.__class__.__name__, current=current)
        
        return self._mesg(ProtocolCode.SetGripperProtectCurrent, [current])
    
    def set_four_pieces_zero(self):
        """Set the zero position of the four-piece motor

        Returns:
            int: 0 or 1 (1 - success)
        """
        return self._mesg(ProtocolCode.SET_FOUR_PIECES_ZERO, has_reply = True)
    
    def jog_rpy(self, end_direction, direction, speed):
        """Rotate the end around a fixed axis in the base coordinate system

        Args:
            end_direction (int):  Roll, Pitch, Yaw (1-3)
            direction (int): 1 - forward rotation, 0 - reverse rotation
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(class_name = self.__class__.__name__, end_direction=end_direction)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, end_direction, direction, speed)
    
    def set_void_compensate(self, mode):
        """Set void compensation mode

        Args:
            mode (int): 0 - close, 1 - open
        """
        self.calibration_parameters(class_name = self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_VOID_COMPENSATE, mode)
    
    def get_robot_status(self):
        """Get robot status
        """
        return self._mesg(ProtocolCode.GET_ROBOT_STATUS, has_reply = True)
    
    def angles_to_coords(self, angles):
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.GET_COORDS, angles, has_reply=True)
    
    def solve_inv_kinematics(self, target_coords, current_angles):
        angles = [self._angle2int(angle) for angle in current_angles]
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(target_coords[idx]))
        for angle in target_coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)