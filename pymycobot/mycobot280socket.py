# coding=utf-8

from __future__ import division
import time
import math
import socket
import threading

from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyCobot280Socket(CommandGenerator):
    """MyCobot Python API Serial communication class.
    Note: Please use this class under the same network

    Supported methods:

        # Overall status
            set_free_mode()
            is_free_mode()
            get_error_information()
            clear_error_information()
            set_fresh_mode()
            get_fresh_mode()
            read_next_error()
            Other at parent class: `CommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            get_angles_coords()
            Other look at parent class: `CommandGenerator`.

        # JOG mode and operation
            jog_rpy()
            set_HTS_gripper_torque()
            get_HTS_gripper_torque()
            get_gripper_protect_current()
            init_gripper()
            set_gripper_protect_current()
            Other at parent class: `CommandGenerator`.

        # Running status and Settings
            set_joint_max()
            set_joint_min()
            Look at parent class: `CommandGenerator`.

        # Servo control
            move_round()
            set_four_pieces_zero()
            Other at parent class: `CommandGenerator`.

        # Atom IO
            set_pin_mode()
            get_gripper_value()
            is_gripper_moving()
            set_pwm_output()
            Other at parent class: `CommandGenerator`.

        # Basic
            set_transponder_mode()
            get_transponder_mode()
            Other at parent class: `CommandGenerator`.

        # Servo state value
            get_servo_speeds()
            get_servo_voltages()
            get_servo_status()
            get_servo_temps()

        # Coordinate transformation
            set_tool_reference()
            get_tool_reference()
            set_world_reference()
            get_world_reference()
            set_reference_frame()
            get_reference_frame()
            set_movement_type()
            get_movement_type()
            set_end_type()
            get_end_type()

        # Other
            close()
            wait() *
    """
    _write = write
    _read = read

    def __init__(self, ip, netport=9000, debug=False):
        """
        Args:
            ip: Server ip
            netport: Server port
        """
        super(MyCobot280Socket, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock

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
            MyCobot280Socket, self)._mesg(genre, *args, **kwargs)
        # [254,...,255]
        with self.lock:
            # data = self._write(self._flatten(real_command), "socket")
            # # if has_reply:
            # data = self._read(genre, method='socket')
            try_count = 0
            while try_count < 3:
                self._write(self._flatten(real_command), "socket")
                data = self._read(genre, method='socket')
                if data is not None and data != b'':
                    break
                try_count += 1
            else:
                return -1
            if genre == ProtocolCode.SET_SSID_PWD:
                return None
            res = self._process_received(data, genre)
            if res == []:
                return None
            elif res is not None and isinstance(res, list) and len(res) == 1 and genre not in [
                ProtocolCode.GET_BASIC_VERSION,
                ProtocolCode.GET_JOINT_MIN_ANGLE,
                ProtocolCode.GET_JOINT_MAX_ANGLE,
                ProtocolCode.SOFTWARE_VERSION]:
                return res[0]
            if genre in [
                ProtocolCode.ROBOT_VERSION,
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
                ProtocolCode.GET_GPIO_IN,
                ProtocolCode.GET_COMMUNICATE_MODE,
                ProtocolCode.SET_COMMUNICATE_MODE,
                ProtocolCode.SetHTSGripperTorque,
                ProtocolCode.GetHTSGripperTorque,
                ProtocolCode.GetGripperProtectCurrent,
                ProtocolCode.InitGripper,
                ProtocolCode.SET_FOUR_PIECES_ZERO
            ]:
                return self._process_single(res)
            elif genre in [ProtocolCode.GET_ANGLES, ProtocolCode.SOLVE_INV_KINEMATICS]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE]:
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
            else:
                return res

    # System Status
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

    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO, has_reply=True)

    # Overall Status
    def set_free_mode(self, flag):
        """set to free mode

        Args:
            flag: 0/1
        """
        self.calibration_parameters(class_name=self.__class__.__name__, flag=flag)
        return self._mesg(ProtocolCode.SET_FREE_MODE, flag)

    def is_free_mode(self):
        """Check if it is free mode

        Return:
            0/1
        """
        return self._mesg(ProtocolCode.IS_FREE_MODE, has_reply=True)

    def set_fresh_mode(self, mode):
        """Set command refresh mode

        Args:
            mode: int.
                1 - Always execute the latest command first.
                0 - Execute instructions sequentially in the form of a queue.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, mode)

    def get_fresh_mode(self):
        """Query sports mode"""
        return self._mesg(ProtocolCode.GET_FRESH_MODE, has_reply=True)

    def read_next_error(self):
        """Robot Error Detection

        Return:
            list len 6.
            0 : No abnormality
            1 : Communication disconnected
            2 : Unstable communication
            3 : Servo abnormality
        """
        return self._mesg(ProtocolCode.READ_NEXT_ERROR, has_reply=True)

    # MDI mode and operation
    def get_radians(self):
        """Get the radians of all joints

        Return:
            list: A list of float radians [radian1, ...]
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, radians, speed):
        """Send the radians of all joints to robot arm

        Args:
            radians: a list of radian values( List[float]), length 6
            speed: (int )0 ~ 100
        """
        calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def sync_send_angles(self, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached

        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 15s.
        """
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f == 1:
                break
            time.sleep(0.1)
        return 1

    def sync_send_coords(self, coords, speed, mode=0, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached

        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular（default）, 1 - linear
            timeout: default 15s.
        """
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1) == 1:
                break
            time.sleep(0.1)
        return 1

    def get_angles_coords(self):
        """Get joint angles and coordinates"""
        return self._mesg(ProtocolCode.GET_ANGLES_COORDS, has_reply=True)

    # Basic for raspberry pi.
    def gpio_init(self):
        """Init GPIO module, and set BCM mode."""
        import RPi.GPIO as GPIO  # type: ignore

        GPIO.setmode(GPIO.BCM)
        self.gpio = GPIO

    def gpio_output(self, pin, v):
        """Set GPIO output value.

        Args:
            pin: (int)pin number.
            v: (int) 0 / 1
        """
        self.gpio.setup(pin, self.gpio.OUT)
        self.gpio.output(pin, v)

    # JOG mode and operation
    def jog_rpy(self, end_direction, direction, speed):
        """Rotate the end around a fixed axis in the base coordinate system

        Args:
            end_direction (int):  Roll, Pitch, Yaw (1-3)
            direction (int): 1 - forward rotation, 0 - reverse rotation
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, end_direction=end_direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, end_direction, direction, speed)

    def jog_increment_angle(self, joint_id, increment, speed):
        """ angle step mode

        Args:
            joint_id: int 1-6.
            increment: Angle increment value
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def jog_increment_coord(self, id, increment, speed):
        """coord step mode

        Args:
            id: axis id 1 - 6.
            increment: Coord increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, speed=speed)
        value = self._coord2int(increment) if id <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, id, [value], speed)

    def set_HTS_gripper_torque(self, torque):
        """Set new adaptive gripper torque

        Args:
            torque (int): 150 ~ 980

        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(class_name=self.__class__.__name__, torque=torque)
        return self._mesg(ProtocolCode.SetHTSGripperTorque, [torque], has_reply=True)

    def get_HTS_gripper_torque(self):
        """Get gripper torque

        Returns:
            int: 150 ~ 980
        """
        return self._mesg(ProtocolCode.GetHTSGripperTorque, has_reply=True)

    def get_gripper_protect_current(self):
        """Get the gripper protection current

        Returns:
            int: 1 ~ 500
        """
        return self._mesg(ProtocolCode.GetGripperProtectCurrent, has_reply=True)

    def init_gripper(self):
        """Initialize gripper

        Returns:
            int: 0 or 1 (1 - success)
        """
        return self._mesg(ProtocolCode.InitGripper, has_reply=True)

    def set_gripper_protect_current(self, current):
        """Set the gripper protection current

        Args:
            current (int): 1 ~ 500
        """
        self.calibration_parameters(class_name=self.__class__.__name__, current=current)

        return self._mesg(ProtocolCode.SetGripperProtectCurrent, [current])

    # Atom IO
    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int): pin number.
            pin_mode (int): 0 - input, 1 - output, 2 - input_pullup
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin_mode=pin_mode)
        return self._mesg(ProtocolCode.SET_PIN_MODE, pin_no, pin_mode)

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
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_type=gripper_type)
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, gripper_type, has_reply=True)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 - not moving
            1 - is moving
            -1- error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    def set_pwm_output(self, channel, frequency, pin_val):
        """ PWM control

        Args:
            channel (int): IO number.
            frequency (int): clock frequency
            pin_val (int): Duty cycle 0 ~ 256; 128 means 50%
        """
        return self._mesg(ProtocolCode.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    # communication mode
    def set_transponder_mode(self, mode):
        """Set basic communication mode

        Args:
            mode: 0 - Turn off transparent transmission，1 - Open transparent transmission
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_COMMUNICATE_MODE, mode, has_reply=True)

    def get_transponder_mode(self):
        return self._mesg(ProtocolCode.GET_COMMUNICATE_MODE, has_reply=True)

    # g9 servo
    def move_round(self):
        """Drive the 9g steering gear clockwise for one revolution
        """
        return self._mesg(ProtocolCode.move_round)

    def set_four_pieces_zero(self):
        """Set the zero position of the four-piece motor

        Returns:
            int: 0 or 1 (1 - success)
        """
        return self._mesg(ProtocolCode.SET_FOUR_PIECES_ZERO, has_reply=True)

    # Running Status and Settings
    def set_joint_max(self, id, angle):
        """Set the joint maximum angle

        Args:
            id: int.
                Joint id 1 - 6
                for gripper: Joint id 7
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    def set_joint_min(self, id, angle):
        """Set the joint minimum angle

        Args:
            id: int.
                Joint id 1 - 6
                for gripper: Joint id 7
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

    # servo state value
    def get_servo_speeds(self):
        """Get joint speed

        Return:
            A list unit step/s
        """
        return self._mesg(ProtocolCode.GET_SERVO_SPEED, has_reply=True)

    def get_servo_voltages(self):
        """Get joint voltages

        Return:
            A list volts < 24 V
        """
        return self._mesg(ProtocolCode.GET_SERVO_VOLTAGES, has_reply=True)

    def get_servo_status(self):
        """Get joint status

        Return:
            [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error, a value of 1 indicates an error
        """
        return self._mesg(ProtocolCode.GET_SERVO_STATUS, has_reply=True)

    def get_servo_temps(self):
        """Get joint temperature

        Return:
            A list unit ℃
        """
        return self._mesg(ProtocolCode.GET_SERVO_TEMPS, has_reply=True)

    # Coordinate transformation
    def set_tool_reference(self, coords):
        """Set tool coordinate system

        Args:
            coords: a list of coords value(List[float])
                    [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords)
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
                    [x(mm), y, z, rx(angle), ry, rz]\n
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords)
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
        self.calibration_parameters(class_name=self.__class__.__name__, rftype=rftype)
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
        self.calibration_parameters(class_name=self.__class__.__name__, move_type=move_type)
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
        self.calibration_parameters(class_name=self.__class__.__name__, end=end)
        return self._mesg(ProtocolCode.SET_END_TYPE, end)

    def get_end_type(self):
        """Get end coordinate system

        Return:
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE, has_reply=True)

    def set_gpio_mode(self, mode):
        """Set pin coding method
        Args:
            mode: (str) BCM or BOARD 
        """
        self.calibration_parameters(gpiomode=mode)
        if mode == "BCM":
            return self._mesg(ProtocolCode.SET_GPIO_MODE, 0)
        else:
            return self._mesg(ProtocolCode.SET_GPIO_MODE, 1)

    def set_gpio_out(self, pin_no, mode):
        """Set the pin as input or output
        Args:
            pin_no: (int) pin id
            mode: (str) "in" or "out"
        """
        if mode == "in":
            return self._mesg(ProtocolCode.SET_GPIO_UP, pin_no, 0)
        else:
            return self._mesg(ProtocolCode.SET_GPIO_UP, pin_no, 1)

    def set_gpio_output(self, pin_no, state):
        """Set the pin to high or low level
        Args:
            pin_no: (int) pin id.
            state: (int) 0 or 1
        """
        return self._mesg(ProtocolCode.SET_GPIO_OUTPUT, pin_no, state)

    def get_gpio_in(self, pin_no):
        """Get pin level status.
        Args:
            pin_no: (int) pin id.
        """
        return self._mesg(ProtocolCode.GET_GPIO_IN, pin_no, has_reply=True)

    def angles_to_coords(self, angles):
        """ Convert angles to coordinates

        Args:
            angles : A float list of all angle.

        Return:
            list: A float list of all coordinates.
        """
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.GET_COORDS, angles, has_reply=True)

    def solve_inv_kinematics(self, target_coords, current_angles):
        """ Convert target coordinates to angles

        Args:
            target_coords: A float list of all coordinates.
            current_angles : A float list of all angle.

        Return:
            list: A float list of all angle.
        """
        angles = [self._angle2int(angle) for angle in current_angles]
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(target_coords[idx]))
        for angle in target_coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)

    def set_vision_mode(self, flag):
        """Set the visual tracking mode to limit the posture flipping of send_coords in refresh mode.
        (Only for visual tracking function)

        Args:
            flag: 0/1; 0 - close; 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, flag=flag)
        return self._mesg(ProtocolCode.SET_VISION_MODE, flag)

    # Other
    def wait(self, t):
        time.sleep(t)
        return self

    def close(self):
        self.sock.close()
