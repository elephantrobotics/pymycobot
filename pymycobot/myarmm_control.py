# coding=utf-8
import functools
import sys
import logging
import time
import threading
import serial
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode, DataProcessor, write, read


class RobotProtocolCode:
    GET_ROBOT_MODIFY_VERSION = 0x01
    GET_ROBOT_SYSTEM_VERSION = 0x02
    GET_ROBOT_TOOL_MODIFY_VERSION = 0x04
    GET_ROBOT_TOOL_SYSTEM_VERSION = 0x09
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWERED_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    GET_ROBOT_STATUS = 0x19
    SET_GRIPPER_ENABLED = 0x63


def setup_logging(debug: bool = False):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO if debug is False else logging.DEBUG)
    debug_formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(levelname).4s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger_handle = logging.StreamHandler()
    logger_handle.setFormatter(debug_formatter)
    logger.addHandler(logger_handle)
    return logger


def setup_serial_port(port, baudrate, timeout=0.1):
    serial_api = serial.Serial()
    serial_api.port = port
    serial_api.baudrate = baudrate
    serial_api.timeout = timeout
    serial_api.rts = False
    serial_api.open()
    return serial_api


class MyArmMProcessor(DataProcessor):

    def __init__(self, port, baudrate, timeout=0.1, debug=False):
        super().__init__(debug)
        self.lock = threading.Lock()
        self._serial_port = setup_serial_port(port, baudrate, timeout)
        self.calibration_parameters = functools.partial(calibration_parameters, class_name=self.__class__.__name__)
        self._write = functools.partial(write, self, method=None)
        self._read = functools.partial(read, self, _class=self.__class__.__name__)

    # def _write(self, command):
    #     write(self, command, method=None)
    #
    # def _read(self, genre, command=None, _class=None, timeout=None):
    #     return read(self, genre, command, timeout, _class)

    @classmethod
    def __is_return(cls, genre, command):
        """
        Check if the command is a return command.
        """
        return len(command) == 6 and command[3] == genre
    
    def _mesg(self, genre, *args, **kwargs):
        kwargs["has_reply"] = True
        real_command, has_reply, _ = super(MyArmMProcessor, self)._mesg(genre, *args, **kwargs)
        real_command = self._flatten(real_command)
        self._write(real_command)

        with self.lock:
            return self._read_genre_result(genre)

    def _read_genre_result(self, genre):
        data = self._read(genre, timeout=None)
        if genre == ProtocolCode.SET_SSID_PWD:
            if len(data) == 5 and data[3] == genre:
                return 1

        if genre == ProtocolCode.SET_BASIC_OUTPUT:
            if self.__is_return(genre, data):
                return data[-2]

        if genre == ProtocolCode.GET_ROBOT_STATUS:
            res = []
            valid_data = data[4:-1]
            for header_i in range(0, len(valid_data), 2):
                d = self._decode_int16(valid_data[header_i: header_i + 2])
                if d == 0:
                    res.append(d)
                    continue
                reverse_bins = reversed(bin(d)[2:])
                rank = [i for i, e in enumerate(reverse_bins) if e != '0']
                res.append(rank)

        else:
            res = self._process_received(data, genre)
        if not res:
            return -1

        if genre in [
            ProtocolCode.ROBOT_VERSION,
            ProtocolCode.GET_ROBOT_ID,
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_CONTROLLER_CONNECTED,
            ProtocolCode.IS_PAUSED,
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
            ProtocolCode.GET_ERROR_INFO,
            ProtocolCode.GET_COMMUNICATE_MODE,
            ProtocolCode.SET_COMMUNICATE_MODE,
            ProtocolCode.SetHTSGripperTorque,
            ProtocolCode.GetHTSGripperTorque,
            ProtocolCode.GetGripperProtectCurrent,
            ProtocolCode.InitGripper,
            ProtocolCode.SET_FOUR_PIECES_ZERO,
            ProtocolCode.STOP,
            ProtocolCode.IS_TOOL_BTN_CLICKED,
            ProtocolCode.GET_ROBOT_TOOL_MODIFY_VERSION
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_ANGLES, ProtocolCode.SOLVE_INV_KINEMATICS]:
            return [self._int2angle(angle) for angle in res]
        elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE]:
            r = []
            for idx in range(3):
                r.append(self._int2coord(res[idx]))
            for idx in range(3, 6):
                r.append(self._int2angle(res[idx]))
            return r
        elif genre in [ProtocolCode.GET_SERVO_VOLTAGES]:
            return [self._int2coord(angle) for angle in res]
        elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
            return self._int2coord(res[0])
        elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION,
                       ProtocolCode.GET_ATOM_VERSION]:
            return self._int2coord(self._process_single(res))
        elif genre == ProtocolCode.GET_ANGLES_COORDS:
            r = []
            for index in range(len(res)):
                if index < 6:
                    r.append(self._int2angle(res[index]))
                elif index < 9:
                    r.append(self._int2coord(res[index]))
                else:
                    r.append(self._int2angle(res[index]))
            return r
        elif self.__is_return(genre, data):
            return self._process_single(res)
        else:
            return res


class MyArmMControl(MyArmMProcessor):

    def __init__(self, port, baudrate=115200, timeout=0.1, debug=False):
        super(MyArmMControl, self).__init__(port=port, baudrate=baudrate, timeout=timeout, debug=debug)

    # System status
    def get_robot_modify_version(self):
        """Get the bot correction version number"""
        return self._mesg(RobotProtocolCode.GET_ROBOT_MODIFY_VERSION)

    def get_robot_system_version(self):
        """Obtaining the Robot Firmware Version (Major and Minor Versions)"""
        return self._mesg(RobotProtocolCode.GET_ROBOT_SYSTEM_VERSION)

    def get_robot_tool_modify_version(self):
        """Get the remediation version of the bot tool"""
        return self._mesg(ProtocolCode.GET_ROBOT_TOOL_MODIFY_VERSION)

    def get_robot_tool_system_version(self):
        """Get the Robot Tool Firmware Version (End Atom)"""
        return self._mesg(RobotProtocolCode.GET_ROBOT_TOOL_SYSTEM_VERSION)

    def power_on(self):
        """The robotic arm turns on the power"""
        return self._mesg(RobotProtocolCode.POWER_ON)

    def power_off(self):
        """The robotic arm turns off the power"""
        return self._mesg(RobotProtocolCode.POWER_OFF)

    def is_powered_on(self):
        """Control core connection status queries

        Return:
            1 - power on
            0 - power off
            -1 - error data
        """
        return self._mesg(RobotProtocolCode.IS_POWERED_ON)

    def release_all_servos(self, data=None):
        """The robot turns off the torque output

        Args:
            data: 1 - Un damping (The default is damping)
        """
        if data is None:
            return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS)
        else:
            return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS, data)

    def set_fresh_mode(self, mode):  # TODO 22-5-19 need test
        """Set command refresh mode

        Args:
            mode: int.
                1 - Always execute the latest command first.
                0 - Execute instructions sequentially in the form of a queue.
        """
        self.calibration_parameters(mode=mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, mode)

    def get_fresh_mode(self):
        """Query sports mode"""
        return self._mesg(ProtocolCode.GET_FRESH_MODE)

    def get_robot_status(self):
        """Get robot status"""
        return self._mesg(ProtocolCode.GET_ROBOT_STATUS)

    def get_angles(self):
        """ Get the angle of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES)

    def write_angle(self, joint_id, degree, speed):
        """Send the angle of a joint to robot arm.

        Args:
            joint_id: (int) 1 ~ 6
            degree: (float) -150 ~ 150
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(joint_id=joint_id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, joint_id, [self._angle2int(degree)], speed)

    def write_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: (list) A float list of all angle.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed)

    def get_coords(self):
        """Get the coordinates of all joints.

        Return:
            list: A float list of all coordinates.
        """
        return self._mesg(ProtocolCode.GET_COORDS)

    def write_coord(self, coord_id, coord, speed):
        """Send the coordinates of a joint to robot arm.

        Args:
            coord_id: (int) 1 ~ 6
            coord: (float) -150 ~ 150
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(coord_id=coord_id, coord=coord, speed=speed)
        value = self._coord2int(coord) if coord_id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, coord_id, [value], speed)

    def write_coords(self, coords, speed, mode=None):
        """Send the coordinates of all joints to robot arm.

        Args:
            coords: (list) A float list of all coordinates.
            speed : (int) 1 ~ 100
            mode: (int) 0 - normal, 1 - low, 2 - high
        """
        self.calibration_parameters(coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        if mode is not None:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, mode)
        else:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed)

    def is_in_position(self, data, mode=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords.
                    for myArm: angles len 7, coords len 6.
            mode: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if mode == 1:
            self.calibration_parameters(coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif mode == 0:
            self.calibration_parameters(angles=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, mode)

    def is_moving(self):
        """Detect if the robot is moving

        Return:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_MOVING)

    def jog_rpy(self, end_direction, direction, speed):
        """Rotate the end around a fixed axis in the base coordinate system

        Args:
            end_direction (int):  Roll, Pitch, Yaw (1-3)
            direction (int): 1 - forward rotation, 0 - reverse rotation
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(direction=direction, speed=speed, end_direction=end_direction
        )
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, end_direction, direction, speed)

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Jog control angle.

        Args:
            joint_id: int
                    for myArm: Joint id 1 - 7.
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        self.calibration_parameters(joint_id=joint_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int 1 ~ 6
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(coord_id=coord_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_increment(self, joint_id, increment, speed):
        """step mode

        Args:
            joint_id(int):
                for myArm: Joint id 1 - 6.
            increment(int): incremental
            speed(int): int (0 - 100)
        """
        if not isinstance(increment, (int, float)):
            raise ValueError("increment must be int or float")

        self.calibration_parameters(joint_id=joint_id, angle=increment, speed=speed)
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
        return self._mesg(ProtocolCode.IS_PAUSED)

    def resume(self):
        """Recovery movement"""
        return self._mesg(ProtocolCode.RESUME)

    def stop(self):
        """Stop moving"""
        return self._mesg(ProtocolCode.STOP)

    def get_encoder(self, joint_id):
        """Obtain the specified joint potential value.

        Args:
            joint_id: (int)
                for myArm: Joint id 1 - 7.
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id)

    def get_encoders(self):
        """Get the six joints of the manipulator

        Returns:
            The list of encoders
        """
        return self._mesg(ProtocolCode.GET_ENCODERS)

    # Running status and Settings
    def get_speed(self):
        """Get speed

        Returns:
            int
        """
        return self._mesg(ProtocolCode.GET_SPEED)

    def set_speed(self, speed):
        """Set speed value

        Args:
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(speed=speed)
        return self._mesg(ProtocolCode.SET_SPEED, speed)

    def get_joint_min(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            joint_id:
                for myArm: Joint id 1 - 7.

        Returns:
            angle value(float)
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id)

    def get_joint_max(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id:
                for myArm: Joint id 1 - 7.

        Return:
            angle value(float)
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id)

    def set_joint_max(self, joint_id, angle):
        """Set the joint maximum angle

        Args:
            joint_id: int.
                for myArm: Joint id 1 - 7.
            angle: 0 ~ 180
        """
        self.calibration_parameters(joint_id=joint_id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, joint_id, angle)

    def set_joint_min(self, joint_id, angle):
        """Set the joint minimum angle

        Args:
            joint_id: int.
                for myArm: Joint id 1 - 7.
            angle: 0 ~ 180
        """
        self.calibration_parameters(joint_id=joint_id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, joint_id, angle)

    # Servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id:
                for myArm: Joint id 1 - 8.

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id)

    def is_all_servo_enable(self):
        """Detect the connection status of all joints

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE)

    def set_servo_data(self, servo_id, data_id, value, mode=None):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Serial number of articulated steering gear.
                for myArm: joint id 1 - 8
            data_id: Data address.
            value: 0 - 4096
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
        """
        if not isinstance(value, int):
            raise TypeError("value must be int")

        if mode is None:
            self.calibration_parameters(servo_id=servo_id, servo_addr=data_id, value=value)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, value)
        else:
            self.calibration_parameters(servo_id=servo_id, servo_addr=data_id, value=value, mode=mode)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, data_id, [value], mode)

    def get_servo_data(self, servo_id, data_id, mode=None):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Serial number of articulated steering gear. 1 ~ 7
            data_id: Data address.
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode is not None:
            self.calibration_parameters(servo_id=servo_id, mode=mode)
            return self._mesg(ProtocolCode.GET_SERVO_DATA, servo_id, data_id, mode)

        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.GET_SERVO_DATA, servo_id, data_id)

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point,
            and the corresponding potential value is 2048.

        Args:
            servo_id: 1 ~ 8
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def release_servo(self, servo_id, mode=None):
        """Power off designated servo

        Args:
            servo_id: int 1 ~ 8
            mode: Default damping, set to 1, cancel damping
        """
        if mode is None:
            self.calibration_parameters(servo_id=servo_id)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

        else:
            self.calibration_parameters(servo_id=servo_id, mode=mode)
            return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id, mode)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int 1 ~ 7
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    def set_digital_output(self, pin_no, pin_signal):
        """Set the terminal atom io status

        Args:
            pin_no     (int):
            pin_signal (int): 0 / 1
        """
        self.calibration_parameters(pin_number=pin_no, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """Get the terminal atom io status
        Returns: int 0/1
        """
        self.calibration_parameters(pin_number=pin_no)
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no)

    def set_gripper_enabled(self):
        """Enable gripper"""
        return self._mesg(RobotProtocolCode.SET_GRIPPER_ENABLED)

    def get_gripper_value(self):
        """Get the value of gripper.

        Return:
            gripper value (int)
        """
        return self._mesg(ProtocolCode.GET_GRIPPER_VALUE)

    def set_gripper_state(self, flag, speed):
        """Set gripper switch state

        Args:
            flag  (int): 0 - open, 1 - close, 254 - release
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(gripper_flag=flag, speed=speed)
        return self._mesg(ProtocolCode.SET_GRIPPER_STATE, flag, speed)

    def set_gripper_value(self, gripper_value, speed):
        """Set gripper value

        Args:
            gripper_value (int): 0 ~ 100
            speed (int): 1 ~ 100
        """

        self.calibration_parameters(gripper_value=gripper_value, speed=speed)
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
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING)

    # Atom IO
    def set_led_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot arm.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(rgb=(r, g, b))
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def is_tool_btn_clicked(self):
        """Judge whether the button on the top of the robot arm is clicked or not.

        Returns:
            0 - not clicked
            1 - is clicked
            -1- error data
        """
        return self._mesg(ProtocolCode.IS_TOOL_BTN_CLICKED)

    # Basic
    def set_basic_output(self, pin_no, pin_signal):
        """Set the base IO output

        Args:
            pin_no: pin port number.
            pin_signal: 0 / 1
        """
        self.calibration_parameters(pin_signal=pin_signal, basic_pin_number=pin_no)
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, pin_no, pin_signal)

    def get_basic_input(self, pin_no):
        """Read the base IO output

        Args:
            pin_no: (int) pin port number.
        """
        self.calibration_parameters(basic_pin_number=pin_no)
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, pin_no)

    def set_ssid_pwd(self, account: str, password: str):
        """Change connected wi-fi.

        Args:
            account: (str) new wi-fi account.
            password: (str) new wi-fi password.
        """
        self._mesg(ProtocolCode.SET_SSID_PWD)
        time.sleep(0.02)
        self.calibration_parameters(account=account, password=password)
        return self._mesg(ProtocolCode.SET_SSID_PWD, account, password)

    def get_ssid_pwd(self):
        """Get connected wi-fi account and password.

        Return:
            (account, password)
        """
        return self._mesg(ProtocolCode.GET_SSID_PWD)

    def set_server_port(self, port):
        """Change the connection port of the server.

        Args:
            port: (int) The new connection port of the server.
        """
        if not isinstance(port, int):
            raise TypeError("server port must be int")
        return self._mesg(ProtocolCode.SET_SERVER_PORT, port)

    def get_tof_distance(self):
        """ Get the detected distance (Requires external distance detector).

        Return:
            (int) The unit is mm.
        """
        return self._mesg(ProtocolCode.GET_TOF_DISTANCE)

    def set_tool_reference(self, coords):
        """Set tool coordinate system

        Args:
            coords: a list of coords value(List[float]), [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_TOOL_REFERENCE, coord_list)

    def get_tool_reference(self):
        """Get tool coordinate system """
        return self._mesg(ProtocolCode.GET_TOOL_REFERENCE)

    def set_world_reference(self, coords):
        """Set the world coordinate system

        Args:
            coords: a list of coords value(List[float]), [x(mm), y, z, rx(angle), ry, rz]\n
        """
        self.calibration_parameters(coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, coord_list)

    def get_world_reference(self):
        """Get the world coordinate system"""
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE)

    def set_reference_frame(self, rftype):
        """Set the base coordinate system

        Args:
            rftype: 0 - base 1 - tool.
        """
        self.calibration_parameters(rftype=rftype)
        return self._mesg(ProtocolCode.SET_REFERENCE_FRAME, rftype)

    def get_reference_frame(self):
        """Get the base coordinate system

        Returns:
            0 - base 1 - tool.
        """
        return self._mesg(ProtocolCode.GET_REFERENCE_FRAME)

    def set_movement_type(self, move_type):
        """Set movement type

        Args:
            move_type: 1 - movel, 0 - moveJ
        """
        self.calibration_parameters(move_type=move_type)
        return self._mesg(ProtocolCode.SET_MOVEMENT_TYPE, move_type)

    def get_movement_type(self):
        """Get movement type

        Returns:
            1 - movel, 0 - moveJ
        """
        return self._mesg(ProtocolCode.GET_MOVEMENT_TYPE)

    def set_end_type(self, mode):
        """Set end coordinate system

        Args:
            mode: int, 0 - flange, 1 - tool
        """
        self.calibration_parameters(mode=mode)
        return self._mesg(ProtocolCode.SET_END_TYPE, mode)

    def get_end_type(self):
        """Get end coordinate system

        Returns:
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE)

    def get_plan_speed(self):
        """Get planning speed

        Returns:
            [movel planning speed, movej planning speed].
        """
        return self._mesg(ProtocolCode.GET_PLAN_SPEED)

    def get_plan_acceleration(self):
        """Get planning acceleration

        Returns:
            [movel planning acceleration, movej planning acceleration].
        """
        return self._mesg(ProtocolCode.GET_PLAN_ACCELERATION)

    def set_plan_speed(self, speed, is_linear):
        """Set planning speed

        Args:
            speed (int): (1 ~ 100).
            is_linear: 0 -> joint 1 -> straight line
        """
        self.calibration_parameters(speed=speed, is_linear=is_linear)
        return self._mesg(ProtocolCode.SET_PLAN_SPEED, speed, is_linear)

    def set_plan_acceleration(self, acceleration, is_linear):
        """Set planning acceleration

        Args:
            acceleration (int): (1 ~ 100).
            is_linear(int): 0 -> joint 1 -> straight line
        """
        self.calibration_parameters(speed=acceleration, is_linear=is_linear)
        return self._mesg(ProtocolCode.SET_PLAN_ACCELERATION, acceleration, is_linear)

    def get_servo_speeds(self):
        """Get the joint speed .
        Returns:
             speeds: list[float * 8] +- 3000 step/s
        """
        return self._mesg(ProtocolCode.GET_SERVO_SPEED)

    def get_servo_currents(self):
        """Get the joint current
        Returns:
             currents: list[float * 8] 0 ~ 3250
        """
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS)

    def get_servo_voltages(self):
        """Get the joint voltages

        Returns:
             voltages: list[float] voltage 0 ~ 240
        """
        return self._mesg(ProtocolCode.GET_SERVO_VOLTAGES)

    def get_servo_status(self):
        """
        Get the joint status.
        Returns:
             status: list[int] 0 ~ 255
             0 - normal,
             other - error
        """
        return self._mesg(ProtocolCode.GET_SERVO_STATUS)

    def get_servo_temps(self):
        """
        Get joint temperature
        Returns:
            temperatures: list[float] 0 ~ 255
        """
        return self._mesg(ProtocolCode.GET_SERVO_TEMPS)

    def set_void_compensate(self, mode):
        """Set the virtual position compensation mode (
        after turning on, the virtual position will be automatically compensated after the exercise is over
        )

        Args:
            mode (int): 0 - close, 1 - open
        """
        self.calibration_parameters(mode=mode)
        return self._mesg(ProtocolCode.SET_VOID_COMPENSATE, mode)

