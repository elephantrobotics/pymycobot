# coding=utf-8

from __future__ import division
import time
import math
import threading
import logging

from pymycobot.log import setup_logging
from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyArm(CommandGenerator):
    """MyCobot Python API Serial communication class.

    Supported methods:

        # Overall status
            Look at parent class: `CommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            Other look at parent class: `CommandGenerator`.

        # JOG mode and operation
            Look at parent class: `CommandGenerator`.

        # Running status and Settings
            Look at parent class: `CommandGenerator`.

        # Servo control
            Look at parent class: `CommandGenerator`.

        # Atom IO
            Look at parent class: `CommandGenerator`.

        # Basic
            Look at parent class: `CommandGenerator`.

        # Other
            wait() *
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyArm, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()

    _write = write
    _read = read

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
            MyArm, self)._mesg(genre, *args, **kwargs)
        command = self._flatten(real_command)
        with self.lock:
            self._write(command)

            if has_reply:
                data = self._read(genre, command=command)
                res = self._process_received(data, genre, arm=7)
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
                    ProtocolCode.GET_ERROR_INFO,
                    ProtocolCode.SET_SSID_PWD,
                    ProtocolCode.SetHTSGripperTorque,
                    ProtocolCode.GetHTSGripperTorque,
                    ProtocolCode.GetGripperProtectCurrent,
                    ProtocolCode.InitGripper,
                    ProtocolCode.SET_FOUR_PIECES_ZERO
                ]:
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE,
                               ProtocolCode.GET_WORLD_REFERENCE]:
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
                        if index < 7:
                            r.append(self._int2angle(res[index]))
                        elif index < 10:
                            r.append(self._int2coord(res[index]))
                        else:
                            r.append(self._int2angle(res[index]))
                    return r
                elif genre == ProtocolCode.GET_SOLUTION_ANGLES:
                    return self._int2angle(res[0])
                else:
                    return res
            return None

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
            radians: a list of radian values( List[float]), length 7
            speed: (int )0 ~ 100
        """
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, degrees, speed)

    def send_angle(self, id, degree, speed):
        """Send one angle of joint to robot arm.

        Args:
            id : Joint id(genre.Angle) 1 - 7.
            angle : angle value(float).
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=degree, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, id, [self._angle2int(degree)], speed, has_reply=True)

    # @check_parameters(Command.SEND_ANGLES)
    def send_angles(self, angles, speed):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]).len 7.
            speed : (int) 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True)

    def sync_send_angles(self, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached

        Args:
            degrees: a list of degree values(List[float]), length 7.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f == 1:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, coords, speed, mode=0, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached

        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1) == 1:
                break
            time.sleep(0.1)
        return self

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

    # Other
    def wait(self, t):
        time.sleep(t)
        return self

    def set_solution_angles(self, angle, speed):
        """Set zero space deflection angle value

        Args:
            angle: Angle of joint 1.
            speed: 1 - 100.
        """
        return self._mesg(ProtocolCode.SET_SOLUTION_ANGLES, [self._angle2int(angle)], speed)

    def get_solution_angles(self):
        """Get zero space deflection angle value"""
        return self._mesg(ProtocolCode.GET_SOLUTION_ANGLES, has_reply=True)

    def get_transponder_mode(self):
        """Obtain the configuration information of serial transmission mode

        Return:
            mode: 0 - 1 - 2
        """
        return self._mesg(ProtocolCode.SET_SSID_PWD, has_reply=True)

    def set_transponder_mode(self, mode):
        """Set serial port transmission mode

        Args:
            mode:
                0 - Turn off transparent transmission.\n
                1 - Turn on transparent transmission. verify all data.\n
                2 - Turn on transparent transmission, only verify the configuration information of communication forwarding mode (default is 0)
        """
        return self._mesg(ProtocolCode.GET_SSID_PWD, mode)

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()

    def set_color(self, r, g, b):
        """Set the color of the LED

        Args:
            r: Red
            g: Green
            b: Blue
        """
        return self._mesg(ProtocolCode.SET_COLOR_MYARM, r, g, b)

    def is_in_position(self, data, id=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords.
                    angles len 7, coords len 6.
            id: 1 - coords, 0 - angles

        Return:
            1 - True\n
            0 - False\n
            -1 - Error
        """
        if id == 1:
            # self.calibration_parameters(coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif id == 0:
            # self.calibration_parameters(degrees=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")
        return self._mesg(ProtocolCode.IS_IN_POSITION, id, data_list, has_reply=True)

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        """Jog control angle.

        Args:
            joint_id: int 1 - 7.
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int 1-6.\n
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coord_id=coord_id, direction=direction)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_absolute(self, joint_id, angle, speed):
        """Jog absolute angle

        Args:
            joint_id: int 1 - 7.
            angle: -180 ~ 180
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, angle=angle, speed=speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, joint_id, [self._angle2int(angle)], speed)

    def jog_increment(self, joint_id, increment, speed):
        """step mode

        Args:
            joint_id: int 1 - 7.
            increment:
            speed: int (0 - 100)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def set_encoder(self, joint_id, encoder, speed):
        """Set a single joint rotation to the specified potential value.

        Args:
            joint_id: int 1 - 7.
            encoder: The value of the set encoder.
            speed : 1 - 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id, encoder=encoder,
                                    speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder], speed)

    def get_encoder(self, joint_id):
        """Obtain the specified joint potential value.

        Args:
            joint_id: (int) 1 - 7.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encode_id=joint_id)
        return self._mesg(ProtocolCode.GET_ENCODER, joint_id, has_reply=True)

    def set_encoders(self, encoders, sp):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            encoders: A encoder list.len 7.
            sp: speed 1 ~ 100
        """
        return self._mesg(ProtocolCode.SET_ENCODERS, encoders, sp)

    def set_encoders_drag(self, encoders, speeds):  # TODO 22-5-19 need test
        """Send all encoders and speeds

        Args:
            encoders: encoders list.
            speeds: Obtained by the get_servo_speeds() method
        """
        self.calibration_parameters(class_name=self.__class__.__name__, encoders=encoders, speeds=speeds)
        return self._mesg(ProtocolCode.SET_ENCODERS_DRAG, encoders, speeds)

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            joint_id: int 1 - 7.

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: int 1 - 7.

        Return:
            angle value(float)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    # Servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id: Joint id 1 - 7.

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id, has_reply=True)

    def set_servo_data(self, servo_id, data_id, value, mode=None):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Serial number of articulated steering gear.joint id 1 - 7
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
            servo_id: Serial number of articulated steering gear.joint id 1 - 7
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
            servo_id: Serial number of articulated steering gear. joint id 1 - 7
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def joint_brake(self, joint_id):
        """Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

        Args:
            joint_id: int 1 - 7
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.JOINT_BRAKE, joint_id)

    def release_servo(self, servo_id, mode=None):
        """Power off designated servo

        Args:
            servo_id: int 1 - 7
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
            servo_id: int 1 - 7
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    def set_joint_max(self, id, angle):
        """Set the joint maximum angle

        Args:
            id: int.
                Joint id 1 - 7.
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    def set_joint_min(self, id, angle):
        """Set the joint minimum angle

        Args:
            id: int.
                Joint id 1 - 7.
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

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
            list len 7.
            0 : No abnormality
            1 : Communication disconnected
            2 : Unstable communication
            3 : Servo abnormality
        """
        return self._mesg(ProtocolCode.READ_NEXT_ERROR, has_reply=True)

    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO, has_reply=True)

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

    def get_angles_coords(self):
        """Get joint angles and coordinates"""
        return self._mesg(ProtocolCode.GET_ANGLES_COORDS, has_reply=True)

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

    def set_four_pieces_zero(self):
        """Set the zero position of the four-piece motor

        Returns:
            int: 0 or 1 (1 - success)
        """
        return self._mesg(ProtocolCode.SET_FOUR_PIECES_ZERO, has_reply=True)

