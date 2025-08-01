# coding=utf-8

from __future__ import division
import time
import math
import logging
import threading

from pymycobot.log import setup_logging
from pymycobot.generate import CommandGenerator
from pymycobot.public import PublicCommandGenerator
from pymycobot.common import ProtocolCode, write, read, ProGripper, MyHandGripper
from pymycobot.error import calibration_parameters


class MyCobot320(CommandGenerator):
    """MyCobot320 Python API Serial communication class.

    Supported methods:

        # Overall status
            set_fresh_mode()
            get_fresh_mode()
            read_next_error()
            clear_error_information()
            Other at parent class: `CommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            get_angles_coords()
            Other look at parent class: `CommandGenerator`.

        # JOG mode and operation
            Look at parent class: `CommandGenerator`.

        # Running status and Settings
            jog_rpy()
            Other at parent class: `CommandGenerator`.

        # Servo control
            Look at parent class: `CommandGenerator`.

        # Atom IO
            init_eletric_gripper()
            set_eletric_gripper()
            set_gripper_mode()
            get_gripper_mode()
            Other at parent class: `CommandGenerator`.

        # Basic
            Look at parent class: `CommandGenerator`.

        # Servo state value
            get_servo_currents()
            get_servo_speeds()
            get_servo_voltages()
            get_servo_status()
            get_servo_temps()
            get_servo_last_pdi()
            set_void_compensate()
            get_robot_status()

        # Coordinate transformation
            set_tool_reference()
            get_world_reference()
            set_world_reference()
            get_world_reference()
            set_reference_frame()
            get_reference_frame()
            set_movement_type()
            get_movement_type()
            set_end_type()
            get_end_type()

        # Force Control Gripper
            set_pro_gripper()
            get_pro_gripper()
            set_pro_gripper_angle()
            get_pro_gripper_angle()
            set_pro_gripper_open()
            set_pro_gripper_close()
            set_pro_gripper_calibration()
            get_pro_gripper_status()
            set_pro_gripper_torque()
            get_pro_gripper_torque()
            set_pro_gripper_speed()
            get_pro_gripper_default_speed()
            set_pro_gripper_abs_angle()
            set_pro_gripper_pause()
            set_pro_gripper_resume()
            set_pro_gripper_stop()

        # Atom
            get_atom_version()

        # Other
            wait() *
            close()
            open()
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False, thread_lock=True):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyCobot320, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.thread_lock = thread_lock
        if thread_lock:
            self.lock = threading.Lock()
        import serial
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()

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
        real_command, has_reply, _async = super(
            MyCobot320, self)._mesg(genre, *args, **kwargs)
        if self.thread_lock:
            with self.lock:
                return self._res(real_command, has_reply, genre)
        else:
            return self._res(real_command, has_reply, genre)

    def _res(self, real_command, has_reply, genre):
        if genre == ProtocolCode.SET_SSID_PWD or genre == ProtocolCode.GET_SSID_PWD:
            self._write(self._flatten(real_command))
            data = self._read(genre)
        else:
            try_count = 0
            while try_count < 3:
                self._write(self._flatten(real_command))
                data = self._read(genre, real_command=real_command)
                if data is not None and data != b'':
                    break
                try_count += 1
            else:
                return -1
        if genre == ProtocolCode.SET_SSID_PWD:
            return 1
        res = self._process_received(data, genre)
        if res is None:
            return None
        if genre == ProtocolCode.SET_TOQUE_GRIPPER:
            if res == [0]:
                self._write(self._flatten(real_command))
                data = self._read(genre, real_command=real_command)
                res = self._process_received(data, genre)

        if res is not None and isinstance(res, list) and len(res) == 1 and genre not in [ProtocolCode.GET_BASIC_VERSION,
                                                                                         ProtocolCode.GET_JOINT_MIN_ANGLE,
                                                                                         ProtocolCode.GET_JOINT_MAX_ANGLE,
                                                                                         ProtocolCode.SOFTWARE_VERSION,
                                                                                         ProtocolCode.GET_ATOM_VERSION]:
            return res[0]
        if genre in [
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_CONTROLLER_CONNECTED,
            ProtocolCode.IS_PAUSED,  # TODO have bug: return b''
            ProtocolCode.IS_IN_POSITION,
            ProtocolCode.IS_MOVING,
            ProtocolCode.IS_SERVO_ENABLE,
            ProtocolCode.IS_ALL_SERVO_ENABLE,
            ProtocolCode.GET_SERVO_DATA,
            ProtocolCode.GET_DIGITAL_INPUT,
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
            ProtocolCode.SET_TOQUE_GRIPPER,
        ]:
            return self._process_single(res)
        elif genre == ProtocolCode.GET_ROBOT_STATUS:
            return self._parse_high_low_bytes(res)
        elif genre in [ProtocolCode.GET_TOQUE_GRIPPER]:
            if res[-1] == 255 and res[-2] == 255:
                self._write(self._flatten(real_command))
                data = self._read(genre)
                res = self._process_received(data, genre)
            return self._process_high_low_bytes(res)
        elif genre in [ProtocolCode.GET_ANGLES, ProtocolCode.GET_ANGLES_PLAN]:
            return [self._int2angle(angle) for angle in res]
        elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE, ProtocolCode.GET_COORDS_PLAN]:
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
        elif genre in [ProtocolCode.SOLVE_INV_KINEMATICS]:
            if res == [8241, 8241, 8241, 8241, 8241, 8241]:
                return 'No solution for conversion'
            return [self._int2angle(angle) for angle in res]
        elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
            return self._int2coord(res[0])
        elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
            return self._int2coord(self._process_single(res))
        elif genre in [ProtocolCode.GET_REBOOT_COUNT]:
            return self._process_high_low_bytes(res)
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

    # Overall Status
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

    def clear_error_information(self):
        """Clear robot error message"""
        return self._mesg(ProtocolCode.CLEAR_ERROR_INFO, has_reply=True)

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

    # Joint limit
    def set_joint_max(self, id, angle):
        """Set the joint maximum angle

        Args:
            id: int 1 - 6
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    def set_joint_min(self, id, angle):
        """Set the joint minimum angle

        Args:
            id: int.
                Joint id 1 - 6
            angle: 0 ~ 180
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

    # Atom Gripper IO
    def init_electric_gripper(self):  # TODO 22-5-19 need test
        """Electric gripper initialization (it needs to be initialized once after inserting and removing the gripper"""
        return self._mesg(ProtocolCode.INIT_ELECTRIC_GRIPPER)

    def set_electric_gripper(self, status):  # TODO 22-5-19 need test
        """Set Electric Gripper Mode

        Args:
            status: 0 - open, 1 - close.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, status=status)
        return self._mesg(ProtocolCode.SET_ELECTRIC_GRIPPER, status)

    def set_gripper_mode(self, mode):
        """Set gripper mode

        Args:
            mode: 0 - transparent transmission. 1 - Port Mode.

        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_GRIPPER_MODE, mode)

    def get_gripper_mode(self):
        """Get gripper mode

        Return:
            mode: 0 - transparent transmission. 1 - Port Mode.
        """
        return self._mesg(ProtocolCode.GET_GRIPPER_MODE, has_reply=True)

    # servo state value
    def get_servo_currents(self):
        """Get joint current

        Return:
            0 ~ 3250 mA
        """
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS, has_reply=True)

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

    def get_servo_last_pdi(self, id):
        """Obtain the pdi of a single steering gear before modification

        Args:
            id: 1 - 6
        """
        self.calibration_parameters(class_name=self.__class__.__name__, servo_id_pdi=id)
        return self._mesg(ProtocolCode.GET_SERVO_LASTPDI, id, has_reply=True)

    def set_void_compensate(self, mode):
        """Set void compensation mode

        Args:
            mode (int): 0 - close, 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_VOID_COMPENSATE, mode)

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

    # Force Control Gripper
    def set_pro_gripper(self, address, value, gripper_id=14):
        """Setting the force-controlled gripper parameters

        Args:
            address (int): Corresponding to the command sequence number in the force-controlled gripper protocol
            value : Parameters in the force-controlled gripper protocol
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, set_gripper_args=[gripper_id, address, value])
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [address], [value])

    def get_pro_gripper(self, address, gripper_id=14):
        """Get the force-controlled gripper parameters

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
            address (int): Corresponding to the command sequence number in the force-controlled gripper protocol
        """
        self.calibration_parameters(class_name=self.__class__.__name__, get_gripper_args=[gripper_id, address])
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [address])

    def set_pro_gripper_angle(self, gripper_angle, gripper_id=14):
        """ Setting the angle of the force-controlled gripper

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=[gripper_id, gripper_angle])
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ANGLE, gripper_angle)

    def get_pro_gripper_angle(self, gripper_id=14):
        """ Setting the angle of the force-controlled gripper

        Return:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_ANGLE)

    def set_pro_gripper_open(self, gripper_id=14):
        """ Open force-controlled gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper_angle(gripper_id, 100)

    def set_pro_gripper_close(self, gripper_id=14):
        """ close force-controlled gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper_angle(gripper_id, 0)

    def set_pro_gripper_calibration(self, gripper_id=14):
        """ Setting the gripper jaw zero position

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_CALIBRATION, 0)

    def get_pro_gripper_status(self, gripper_id):
        """ Get the clamping status of the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_STATUS)

    def set_pro_gripper_torque(self, torque_value, gripper_id=14):
        """ Setting gripper torque

        Args:
            torque_value (int): 100 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(class_name=self.__class__.__name__, torque_value=[gripper_id, torque_value])
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_TORQUE, torque_value)

    def get_pro_gripper_torque(self, gripper_id=14):
        """ Setting gripper torque

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            torque_value (int): 100 ~ 300
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_TORQUE)

    def set_pro_gripper_speed(self, speed, gripper_id=14):
        """ Set the gripper speed

        Args:
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_speed=[gripper_id, speed])
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_SPEED, speed)

    def get_pro_gripper_speed(self, gripper_id=14):
        """ Get the gripper speed

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.get_pro_gripper(gripper_id, ProGripper.GET_GRIPPER_SPEED)

    def set_pro_gripper_abs_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper absolute angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=[gripper_id, gripper_angle])
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_ABS_ANGLE, gripper_angle)

    def set_pro_gripper_pause(self, gripper_id=14):
        """ Pause movement

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_PAUSE, 0)

    def set_pro_gripper_resume(self, gripper_id=14):
        """ Resume movement

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_RESUME, 0)

    def set_pro_gripper_stop(self, gripper_id=14):
        """ Stop movement

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self.set_pro_gripper(gripper_id, ProGripper.SET_GRIPPER_STOP, 0)

    def get_atom_version(self):
        """Get atom firmware version.

        Returns:
            float: version number.
        """
        return self._mesg(ProtocolCode.GET_ATOM_VERSION, has_reply=True)

    def get_robot_status(self):
        """Get robot status
        """
        return self._mesg(ProtocolCode.GET_ROBOT_STATUS, has_reply=True)

    def set_vision_mode(self, flag):
        """Set the visual tracking mode to limit the posture flipping of send_coords in refresh mode.
        (Only for visual tracking function)

        Args:
            flag: 0/1; 0 - close; 1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, flag=flag)
        return self._mesg(ProtocolCode.SET_VISION_MODE, flag)

    # myHand  Gripper Control
    def get_hand_firmware_major_version(self, gripper_id=14):
        """Read the firmware major version number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_MAJOR_FIRMWARE_VERSION])

    def get_hand_firmware_minor_version(self, gripper_id=14):
        """Read the firmware minor version number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_MINOR_FIRMWARE_VERSION])

    def set_hand_gripper_id(self, id_value, gripper_id=14):
        """Set the gripper ID

        Args:
            id_value (int): 1 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, set_id=id_value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_ID], [id_value])

    def get_hand_gripper_id(self, gripper_id=14):
        """Get the gripper ID

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper ID (int): 1 ~ 254
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_ID])

    def set_hand_gripper_angle(self, joint_id, gripper_angle, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=[gripper_id, gripper_angle], gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_ANGLE], [joint_id], [gripper_angle])

    def get_hand_gripper_angle(self, joint_id, gripper_id=14):
        """Get the angle of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper_angle (int): 0 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_ANGLE], [joint_id])

    def set_hand_gripper_angles(self, gripper_angles, speed, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            gripper_angles (list): A list of integers, length 6, each value range 0 ~ 100
            speed (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_speed=[gripper_id, speed], gripper_angles=gripper_angles)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_ANGLES], [gripper_angles], [speed])

    def get_hand_gripper_angles(self, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper_angles (list): A list of integers, length 6
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_ALL_ANGLES])

    def set_hand_gripper_torque(self, joint_id, torque_value, gripper_id=14):
        """ Setting gripper torque

        Args:
            joint_id (int): 1 ~ 6
            torque_value (int): 100 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(class_name=self.__class__.__name__, torque_value=[gripper_id, torque_value])
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_TORQUE], [joint_id], [torque_value])

    def get_hand_gripper_torque(self, joint_id, gripper_id=14):
        """ Setting gripper torque

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            torque_value (int): 100 ~ 300
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_TORQUE], [joint_id])

    def set_hand_gripper_calibrate(self, joint_id, gripper_id=14):
        """ Setting the gripper jaw zero position

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_CALIBRATION], [joint_id], [0])

    def get_hand_gripper_status(self, gripper_id=14):
        """ Get the clamping status of the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_STATUS])

    def set_hand_gripper_enabled(self, flag, gripper_id=14):
        """ Set the enable state of the gripper

        Args:
            flag (int): o or 1
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, flag=flag)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_ENABLED], [flag])

    def set_hand_gripper_speed(self, joint_id, speed, gripper_id=14):
        """ Set the speed of the gripper

        Args:
            joint_id (int): 1 ~ 6
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_speed=[gripper_id, speed], gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_SPEED], [joint_id], [speed])

    def get_hand_gripper_default_speed(self,joint_id, gripper_id=14):
        """ Get the default speed of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            default speed (int): 1 ~ 100

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_DEFAULT_SPEED], [joint_id])

    def set_hand_gripper_p(self, joint_id, value, gripper_id=14):
        """ Set the P value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 150
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, gripper_p=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_P], [joint_id], [value])

    def get_hand_gripper_p(self, joint_id, gripper_id=14):
        """ Get the P value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            P value (int): 0 ~ 150

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_P], [joint_id])

    def set_hand_gripper_d(self, joint_id, value, gripper_id=14):
        """ Set the D value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 150
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, gripper_d=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_D], [joint_id], [value])

    def get_hand_gripper_d(self, joint_id, gripper_id=14):
        """ Get the D value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            D value (int): 0 ~ 150

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_D], [joint_id])

    def set_hand_gripper_i(self, joint_id, value, gripper_id=14):
        """ Set the I value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, gripper_i=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_I], [joint_id], [value])

    def get_hand_gripper_i(self, joint_id, gripper_id=14):
        """ Get the I value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            I value (int): 0 ~ 150

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_I], [joint_id])

    def set_hand_gripper_min_pressure(self, joint_id, value, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, min_pressure=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_MIN_PRESSURE], [joint_id], [value])

    def get_hand_gripper_min_pressure(self, joint_id, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            min pressure value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_MIN_PRESSURE], [joint_id])

    def set_hand_gripper_clockwise(self, joint_id, value, gripper_id=14):
        """ Set the clockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 16
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, clockwise=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_CLOCKWISE], [joint_id], [value])

    def get_hand_gripper_clockwise(self, joint_id, gripper_id=14):
        """ Get the clockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            value (int): 0 ~ 16

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_CLOCKWISE], [joint_id])

    def set_hand_gripper_counterclockwise(self, joint_id, value, gripper_id=14):
        """ Set the counterclockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 16
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id, clockwise=value)
        return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_COUNTERCLOCKWISE], [joint_id], [value])

    def get_hand_gripper_counterclockwise(self, joint_id, gripper_id=14):
        """ Get the counterclockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            value (int): 0 ~ 16

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, gripper_joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_COUNTERCLOCKWISE], [joint_id])

    def get_reboot_count(self):
        """Read reboot count

        Return:
            reboot count
        """
        return self._mesg(ProtocolCode.GET_REBOOT_COUNT, has_reply=True)

    def set_hand_gripper_pinch_action(self, pinch_pose, rank_mode, idle_flag=False, gripper_id=14):
        """ Setting the gripper pinching action-speed coordination

        Args:
            pinch_pose (int): 0 ~ 4
                0: All joints return to zero
                1: Index finger and thumb pinch together
                2: Middle finger and thumb pinch together
                3: Index finger and middle finger pinch together
                4: Three fingers together - rank_mode (int): 1 ~ 20
            rank_mode (int): 0 ~ 5
                The degree of closure,the higher the level, the more closed
            idle_flag (bool): default False
                Idle flag. By default, there is no such byte. When this byte is 1, the idle finger can be freely manipulated.
            gripper_id (int): 1 ~ 254, default 14

        """
        if not idle_flag:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, pinch_pose=pinch_pose, rank_mode=rank_mode)
            return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT], pinch_pose, rank_mode)

        else:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, pinch_pose=pinch_pose, rank_mode=rank_mode, idle_flag=idle_flag)
            return self._mesg(ProtocolCode.SET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT], pinch_pose, rank_mode, idle_flag)

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

    def get_angles_plan(self):
        """ Get the angle plan of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES_PLAN, has_reply=True)

    def get_coords_plan(self):
        """Get the coords plan from robot arm, coordinate system based on base.

        Return:
            list : A float list of coord .[x, y, z, rx, ry, rz]
        """
        return self._mesg(ProtocolCode.GET_COORDS_PLAN, has_reply=True)

    def get_hand_gripper_type(self, gripper_id=14):
        """Get the machine model
        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return: int
            0 - left hand
            1 - right hand
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)
        return self._mesg(ProtocolCode.GET_TOQUE_GRIPPER, gripper_id, [MyHandGripper.GET_HAND_GRIPPER_ROBOT_MODEL])

    # Other
    def wait(self, t):
        time.sleep(t)
        return self

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()
        
    def go_home(self):
        return self.send_angles([0,0,0,0,0,0], 10)
        
