#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import division

import functools
import socket
import time
import threading
import serial

from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


def setup_serial_connect(port, baudrate, timeout):
    serial_api = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    serial_api.rts = False
    if not serial_api.is_open:
        serial_api.open()
    return serial_api


def setup_socket_connect(host, port, timeout):
    socket_api = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket_api.settimeout(timeout)
    socket_api.connect((host, port))
    return socket_api


class GPIOProtocolCode:
    SETUP_GPIO_MODE = 0xAA
    SETUP_GPIO_STATE = 0xAB
    SET_GPIO_OUTPUT = 0xAC
    GET_GPIO_INPUT = 0xAD


class MyCobot280X5Api(CommandGenerator):

    def __init__(self, debug=False, thread_lock=True):
        super(MyCobot280X5Api, self).__init__(debug)
        self.calibration_parameters = functools.partial(calibration_parameters, class_name="MyCobot280")
        self.thread_lock = thread_lock
        self.lock = threading.Lock()

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
        real_command, has_reply, _ = super(MyCobot280X5Api, self)._mesg(genre, *args, **kwargs)
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
                data = self._read(genre)
                if data is not None and data != b'':
                    break
                try_count += 1
            else:
                return -1
        if genre == ProtocolCode.SET_SSID_PWD:
            return 1

        if genre == ProtocolCode.GET_QUICK_INFO:
            res = []
            valid_data = data[4:-1]
            for header_i in range(0, len(valid_data), 2):
                if header_i < 26:
                    one = valid_data[header_i: header_i + 2]
                    res.append(self._decode_int16(one))
            res.extend(valid_data[25:])
        else:
            res = self._process_received(data, genre)

        if res is None:
            return None
        if genre in [ProtocolCode.SET_BASIC_OUTPUT]:
            return 1
        if res is not None and isinstance(res, list) and len(res) == 1 and genre not in [
            ProtocolCode.GET_BASIC_VERSION, ProtocolCode.GET_JOINT_MIN_ANGLE, ProtocolCode.GET_JOINT_MAX_ANGLE,
            ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION
        ]:
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
        elif genre in [
            ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION
        ]:
            return self._int2coord(self._process_single(res))
        elif genre in [ProtocolCode.GET_REBOOT_COUNT]:
            return self._process_high_low_bytes(res)
        elif genre in (ProtocolCode.GET_ANGLES_COORDS, ProtocolCode.GET_QUICK_INFO):
            r = []
            for index, el in enumerate(res):
                if index < 6:
                    r.append(self._int2angle(el))
                elif index < 9:
                    r.append(self._int2coord(el))
                elif index < 12:
                    r.append(self._int2angle(el))
                else:
                    r.append(el)
            return r
        else:
            return res

    def wait(self, t):
        time.sleep(t)
        return self


class MyCobot280X5PICommandGenerator(MyCobot280X5Api):

    # System Status
    def get_modify_version(self):
        """get modify version"""
        return self._mesg(ProtocolCode.ROBOT_VERSION)

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION)

    def clear_queue(self):
        """Clear the command queue"""
        return self._mesg(ProtocolCode.CLEAR_COMMAND_QUEUE)

    def async_or_sync(self):
        """Set the command execution mode
        Return:
            0: Asynchronous mode
            1: Synchronous mode
        """
        return self._mesg(ProtocolCode.CHECK_ASYNC_OR_SYNC)

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

    def set_fresh_mode(self, mode):
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
        return self._mesg(ProtocolCode.GET_FRESH_MODE, has_reply=True)

    def set_vision_mode(self, flag):
        """Set the visual tracking mode to limit the posture flipping of send_coords in refresh mode.
        (Only for visual tracking function)

        Args:
            flag: 0/1; 0 - close; 1 - open
        """
        self.calibration_parameters(flag=flag)
        return self._mesg(ProtocolCode.SET_VISION_MODE, flag)

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

    def get_quick_move_message(self):
        """Get the quick move message"""
        return self._mesg(ProtocolCode.GET_QUICK_INFO, has_reply=True)

    # JOG mode and operation
    def write_angles_time_control(self, angles, step_time):
        """Write the angle of a joint in time control mode
        Args:
            angles: Angle value
            step_time: Time unit: 30ms, range(1 ~ 255)
        """
        if step_time not in range(1, 256):
            raise ValueError("step_time must be in range(1 ~ 255)")
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.WRITE_ANGLE_TIME, angles, step_time)

    def jog_rpy(self, end_direction, direction, speed):
        """Rotate the end around a fixed axis in the base coordinate system

        Args:
            end_direction (int):  Roll, Pitch, Yaw (1-3)
            direction (int): 1 - forward rotation, 0 - reverse rotation
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(end_direction=end_direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, end_direction, direction, speed)

    def jog_increment_angle(self, joint_id, increment, speed):
        """ angle step mode

        Args:
            joint_id: int 1-6.
            increment: Angle increment value
            speed: int (0 - 100)
        """
        self.calibration_parameters(id=joint_id, speed=speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def jog_increment_coord(self, axis, increment, speed):
        """coord step mode

        Args:
            axis: axis id 1 - 6.
            increment: Coord increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(id=axis, speed=speed)
        value = self._coord2int(increment) if axis <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, axis, [value], speed)

    def set_HTS_gripper_torque(self, torque):
        """Set new adaptive gripper torque

        Args:
            torque (int): 150 ~ 980

        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(torque=torque)
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
        self.calibration_parameters(current=current)

        return self._mesg(ProtocolCode.SetGripperProtectCurrent, [current])

    # Atom IO
    def set_pin_mode(self, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            pin_no   (int): pin.
            pin_mode (int): 0 - input, 1 - output, 2 - input pull up
        """
        self.calibration_parameters(pin_mode=pin_mode)
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
            self.calibration_parameters(gripper_type=gripper_type)
            return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, gripper_type, has_reply=True)

    def is_gripper_moving(self):
        """Judge whether the gripper is moving or not

        Returns:
            0 - not moving
            1 - is moving
            -1- error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)

    def get_tool_system_version(self):
        """
        Read the terminal primary and minor version numbers
        """
        return self._mesg(ProtocolCode.GET_ATOM_VERSION, has_reply=True)

    def get_tool_modify_version(self):
        """
        Read the terminal modified version number
        """
        return self._mesg(ProtocolCode.OVER_LIMIT_RETURN_ZERO, has_reply=True)

    def is_tool_connected(self):
        """Check the end connection status"""
        return self._mesg(ProtocolCode.IS_CONTROLLER_CONNECTED, has_reply=True)

    def is_tool_button_click(self):
        """Check whether the button on the end is pressed"""
        return self._mesg(ProtocolCode.IS_CONTROLLER_BUTTON_PRESSED, has_reply=True)

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
        self.calibration_parameters(mode=mode)
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
        self.calibration_parameters(id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, angle)

    def set_joint_min(self, id, angle):
        """Set the joint minimum angle

        Args:
            id: int.
                Joint id 1 - 6
                for gripper: Joint id 7
            angle: 0 ~ 180
        """
        self.calibration_parameters(id=id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, angle)

    # servo state value
    def get_servo_speeds(self):
        """Get joint speed

        Return:
            A list unit step/s
        """
        return self._mesg(ProtocolCode.GET_SERVO_SPEED, has_reply=True)

    def get_servo_currents(self):
        """Get all joint current

        Return:
             A list unit mA
        """
        return self._mesg(ProtocolCode.GET_SERVO_CURRENTS, has_reply=True)

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
        self.calibration_parameters(coords=coords)
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
        self.calibration_parameters(coords=coords)
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
        self.calibration_parameters(rftype=rftype)
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
        self.calibration_parameters(move_type=move_type)
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
        self.calibration_parameters(end=end)
        return self._mesg(ProtocolCode.SET_END_TYPE, end)

    def get_end_type(self):
        """Get end coordinate system

        Return:
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE, has_reply=True)

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

    def is_torque_gripper(self):
        """Whether it is a force-controlled gripper

        Return:
            40 - Force control
            9 - Non-force control
        """
        return self.get_servo_data(7, 1)

    def set_gripper_state(self, flag, speed, _type_1=None, is_torque=None):
        """Set gripper switch state

        Args:
            flag  (int): 0 - open, 1 - close, 254 - release
            speed (int): 1 ~ 100
            _type_1 (int): default 1
                1 : Adaptive gripper. default to adaptive gripper
                2 : 5 finger dexterous hand
                3 : Parallel gripper, this parameter can be omitted
                4 : Flexible gripper
            is_torque (int): When there is no type parameter, this parameter can be omitted.
                1: Force control
                0: Non-force control
        """
        self.calibration_parameters(flag=flag, speed=speed, _type_1=_type_1, is_torque=is_torque)
        args = [flag, speed]
        if _type_1 is not None:
            args.append(_type_1)
        if is_torque is not None:
            args.append(is_torque)
        return self._mesg(ProtocolCode.SET_GRIPPER_STATE, *args)

    def set_gripper_value(self, gripper_value, speed, gripper_type=None, is_torque=None):
        """Set gripper value

        Args:
            gripper_value (int): 0 ~ 100
            speed (int): 1 ~ 100
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper, this parameter can be omitted
                4: Flexible gripper
            is_torque (int): When there is no type parameter, this parameter can be omitted.
                1: Force control
                0: Non-force control
        """
        self.calibration_parameters(gripper_value=gripper_value, speed=speed,
                                    gripper_type=gripper_type, is_torque=is_torque)
        args = [gripper_value, speed]
        if gripper_type is not None:
            args.append(gripper_type)
        if is_torque is not None:
            args.append(is_torque)
        return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, *args, has_reply=True)

    def drag_start_record(self):
        """Start track recording

        Return:
            Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_START_RECORD, has_reply=True)

    def drag_end_record(self):
        """End track recording

        Return:
             Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_END_RECORD, has_reply=True)

    def drag_get_record_data(self):
        """Get the recorded track

        Return:
            List of potential values (encoder values) and operating speeds of each joint
            eg: [J1_encoder, J1_run_speed,J2_encoder, J2_run_speed,J3_encoder, J3_run_speed,J4_encoder, J4_run_speed,J5_
            encoder, J5_run_speed,J6_encoder, J6_run_speed]
        """

        return self._mesg(ProtocolCode.DRAG_GET_RECORD_DATA, has_reply=True)

    def drag_get_record_len(self):
        """Get the total number of recorded points

        Return:
            Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_GET_RECORD_LEN, has_reply=True)

    def drag_clear_record_data(self):
        """Clear recording track

        Return:
            Recording queue length 0
        """

        return self._mesg(ProtocolCode.DRAG_CLEAR_RECORD_DATA, has_reply=True)


class MyCobot280X5PI(MyCobot280X5PICommandGenerator):

    def __init__(self, port, baudrate=100_0000, timeout=0.1, debug=False, thread_lock=True):
        """
        Args:
            port     : port string
            baudrate : baud rate int, default 100_0000
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super().__init__(debug, thread_lock)
        self._serial_port = setup_serial_connect(port=port, baudrate=baudrate, timeout=timeout)
        self._write = functools.partial(write, self)
        self._read = functools.partial(read, self)

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()


class MyCobot280X5PISocket(MyCobot280X5PICommandGenerator):
    """MyCobot 280 X5 PI Socket Control Class

    server file: https://github.com/elephantrobotics/pymycobot/demo/Server_280_X5PI.py
    """
    def __init__(self, ip, port=30002, timeout=0.1, debug=False, thread_lock=True):
        super().__init__(debug, thread_lock)
        self.sock = setup_socket_connect(ip, port, timeout)
        self._write = functools.partial(write, self, method="socket")
        self._read = functools.partial(read, self, method="socket")

    def set_gpio_mode(self, mode):
        """Set pin coding method
        Args:
            mode: (int) 0 - BCM, 1 - BOARD

        returns:
            (int) 1 - success, 255 - error
        """
        if mode not in (0, 1):
            raise ValueError("mode must be 0 or 1")
        return self._mesg(GPIOProtocolCode.SETUP_GPIO_MODE, mode)

    def setup_gpio_state(self, pin_no, mode, initial=1):
        """Set the pin as input or output
        Args:
            pin_no: (int) pin id
            mode: (int) 0 - input, 1 - output
            initial: (int) 0 - low, 1 - high
        returns:
            (int) 1 - success, 255 - error
        """
        if mode not in (0, 1):
            raise ValueError("mode must be 0 or 1")

        if initial not in (0, 1):
            raise ValueError("initial must be 0 or 1")

        return self._mesg(GPIOProtocolCode.SETUP_GPIO_STATE, pin_no, mode, initial)

    def set_gpio_output(self, pin_no, state):
        """Set the pin to high or low level
        Args:
            pin_no: (int) pin id.
            state: (int) 0 - low, 1 - high
        returns:
            (int) 1 - success, 255 - error
        """
        return self._mesg(GPIOProtocolCode.SET_GPIO_OUTPUT, pin_no, state)

    def get_gpio_input(self, pin_no):
        """Get pin level status.
        Args:
            pin_no: (int) pin id.
        Returns:
            (int) 0 - low, 1 - high, 255 - error
        """
        return self._mesg(GPIOProtocolCode.GET_GPIO_INPUT, pin_no, has_reply=True)

    def close(self):
        self.sock.close()


def main():
    mc_sock = MyCobot280X5PISocket('192.168.1.246', port=30002, debug=True)
    # print(mc_sock.send_angle(1, 100, 50))
    # print(mc_sock.get_quick_move_message())
    print(mc_sock.set_gpio_mode(0))
    print(mc_sock.setup_gpio_state(5, 1, initial=1))
    print(mc_sock.set_gpio_output(5, 0))
    # print(mc_sock.get_gpio_input(5))


if __name__ == '__main__':
    main()
