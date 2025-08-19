#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import division

import functools
import socket
import time
import threading
import serial

from pymycobot.generate import DataProcessor
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


def setup_serial_port(port, baudrate, timeout=None):
    serial_api = serial.Serial()
    serial_api.port = port
    serial_api.baudrate = baudrate
    serial_api.timeout = timeout
    serial_api.rts = False
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


class MyCobot280RDKX5Api(DataProcessor):

    def __init__(self, debug=False, thread_lock=True):
        super(MyCobot280RDKX5Api, self).__init__(debug)
        self.calibration_parameters = functools.partial(self.parametric_verification, class_name="MyCobot280")
        self.thread_lock = thread_lock
        self.lock = threading.Lock()

    @classmethod
    def parametric_verification(cls, **kwargs):
        kwargs["class_name"] = "MyCobot280RDK-X5"
        return calibration_parameters(**kwargs)

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
        real_command, has_reply, _ = super(MyCobot280RDKX5Api, self)._mesg(genre, *args, **kwargs)
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

        res = self._process_received(data, genre)

        if res is None:
            return -1
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
        elif genre in (ProtocolCode.GET_ANGLES_COORDS, ):
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


class MyCobot280RDKX5CommandGenerator(MyCobot280RDKX5Api):

    # Overall Status
    def get_modify_version(self):
        """get modify version"""
        return self._mesg(ProtocolCode.ROBOT_VERSION)

    def get_system_version(self):
        """get system version"""
        return self._mesg(ProtocolCode.SOFTWARE_VERSION)

    def clear_queue(self):
        """Clear the command queue"""
        return self._mesg(ProtocolCode.CLEAR_COMMAND_QUEUE)

    def get_queue_length(self):
        """Get current queue length"""
        return self._mesg(ProtocolCode.GET_COMMAND_QUEUE)

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

    def set_monitor_state(self, monitor_state):
        """
        Set the monitoring state
        Args:
            monitor_state: 0 - Disable monitoring
                   1 - Enable monitoring
        """
        self.calibration_parameters(monitor_state=monitor_state)
        return self._mesg(ProtocolCode.SET_MONITOR_STATE, monitor_state)

    def get_monitor_state(self):
        """
        Get the monitoring state
        """
        return self._mesg(ProtocolCode.GET_MONITOR_STATE, has_reply=True)

    def get_reboot_count(self):
        """Get the number of times the robot has been restarted"""
        return self._mesg(ProtocolCode.GET_REBOOT_COUNT, has_reply=True)

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

    def release_all_servos(self):
        """Relax all joints"""
        return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS)

    def focus_all_servos(self):
        """Damping all joints"""
        return self._mesg(ProtocolCode.FOCUS_ALL_SERVOS)

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

    def set_fresh_mode(self, fresh_mode):
        """Set command refresh mode

        Args:
            fresh_mode: int.
                1 - Always execute the latest command first.
                0 - Execute instructions sequentially in the form of a queue.
        """
        self.calibration_parameters(fresh_mode=fresh_mode)
        return self._mesg(ProtocolCode.SET_FRESH_MODE, fresh_mode)

    def get_fresh_mode(self):
        """Query sports mode"""
        return self._mesg(ProtocolCode.GET_FRESH_MODE, has_reply=True)

    def set_vision_mode(self, vision_mode):
        """Set the visual tracking mode to limit the posture flipping of send_coords in refresh mode.
        (Only for visual tracking function)

        Args:
            vision_mode: 0/1; 0 - close; 1 - open
        """
        self.calibration_parameters(vision_mode=vision_mode)
        return self._mesg(ProtocolCode.SET_VISION_MODE, vision_mode)

    # Motion interface
    def get_angles(self):
        """ Get the angle of all joints.

        Return:
            list: A float list of all angle.
        """
        return self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)

    def send_angle(self, joint_id, angle, speed, _async=False):
        """Send one angle of joint to robot arm.

        Args:
            joint_id : Joint id(genre.Angle) int 1-6.
            angle : angle value(float).
            speed : (int) 1 ~ 100
            _async: (default False) Whether to execute asynchronously, Currently invalid
        """
        self.calibration_parameters(joint_id=joint_id, angle=angle, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLE, joint_id, [self._angle2int(angle)], speed, has_reply=True, _async=_async)

    def send_angles(self, angles, speed, _async=False):
        """Send the angles of all joints to robot arm.

        Args:
            angles: a list of angle values(List[float]). len 6.
            speed : (int) 1 ~ 100
            _async: (default False) Whether to execute asynchronously, Currently invalid
        """
        self.calibration_parameters(angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, has_reply=True, _async=_async)

    def get_coords(self):
        """Get the coords from robot arm, coordinate system based on base.

        Return:
            list : A float list of coord .[x, y, z, rx, ry, rz]
        """
        return self._mesg(ProtocolCode.GET_COORDS, has_reply=True)

    def angles_to_coords(self, angles):
        """ Convert angles to coordinates

        Args:
            angles : A float list of all angle.

        Return:
            list: A float list of all coordinates.
        """
        self.calibration_parameters(angles=angles)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.GET_COORDS, angles, has_reply=True)

    def send_coord(self, coord_id, coord, speed, _async=False):
        """Send one coord to robot arm.

        Args:
            coord_id(int) : coord id(genre.Coord) int 1-6.
            coord(float) : coord value, mm
            speed(int) : 1 ~ 100
            _async: (default False) Whether to execute asynchronously, Currently invalid
        """
        self.calibration_parameters(coord_id=coord_id, coord=coord, speed=speed)
        value = self._coord2int(coord) if coord_id <= 3 else self._angle2int(coord)
        return self._mesg(ProtocolCode.SEND_COORD, coord_id, [value], speed, has_reply=True, _async=_async)

    def send_coords(self, coords, speed, coord_mode=None, _async=False):
        """Send all coords to robot arm.

        Args:
            coords: a list of coords value(List[float]).[x(mm), y, z, rx(angle), ry, rz]\n
            speed : (int) 1 ~ 100
            coord_mode : (int) 0 - angluar, 1 - linear
            _async: (default False) Whether to execute asynchronously, Currently invalid
        """
        self.calibration_parameters(coords=coords, speed=speed, coord_mode=coord_mode)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        if coord_mode is not None:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, coord_mode, _async=_async)
        else:
            return self._mesg(ProtocolCode.SEND_COORDS, coord_list, speed, has_reply=True, _async=_async)

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

    def is_in_position(self, data, mode=0):
        """Judge whether in the position.

        Args:
            data: A data list, angles or coords.len 6.
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
            raise Exception("mode is not right, please input 0 or 1")

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
        self.calibration_parameters(joint_id=joint_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ANGLE, joint_id, direction, speed)

    def jog_rpy(self, end_direction, direction, speed):
        """Rotate the end around a fixed axis in the base coordinate system

        Args:
            end_direction (int):  Roll, Pitch, Yaw (1-3)
            direction (int): 1 - forward rotation, 0 - reverse rotation
            speed (int): 1 ~ 100
        """
        self.calibration_parameters(end_direction=end_direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_ABSOLUTE, end_direction, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        """Jog control coord.

        Args:
            coord_id: int 1-6
            direction: 0 - decrease, 1 - increase
            speed: int (1 - 100)
        """
        self.calibration_parameters(coord_id=coord_id, direction=direction, speed=speed)
        return self._mesg(ProtocolCode.JOG_COORD, coord_id, direction, speed)

    def jog_increment_angle(self, joint_id, increment, speed):
        """ angle step mode

        Args:
            joint_id: int 1-6.
            increment: Angle increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(id=joint_id, speed=speed, angle=increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)

    def jog_increment_coord(self, coord_id, increment, speed):
        """coord step mode

        Args:
            coord_id: coord id 1 - 6.
            increment: Coord increment value
            speed: int (1 - 100)
        """
        self.calibration_parameters(coord_id=coord_id, speed=speed, coord=increment)
        value = self._coord2int(increment) if coord_id <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed)

    def set_HTS_gripper_torque(self, gripper_torque):
        """Set new adaptive gripper torque

        Args:
            gripper_torque (int): 150 ~ 980

        Return:
            0: Set failed
            1: Set successful
        """
        self.calibration_parameters(gripper_torque=gripper_torque)
        return self._mesg(ProtocolCode.SetHTSGripperTorque, [gripper_torque], has_reply=True)

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

    def set_gripper_protect_current(self, protect_current):
        """Set the gripper protection current

        Args:
            protect_current (int): 1 ~ 500
        """
        self.calibration_parameters(protect_current=protect_current)

        return self._mesg(ProtocolCode.SetGripperProtectCurrent, [protect_current])

    def set_encoder(self, servo_id, encoder, speed):
        """Set a single joint rotation to the specified potential value.

        Args:
            servo_id: int  1 - 7
            encoder: The value of the set encoder.
            speed : 1 - 100
        """
        self.calibration_parameters(servo_id=servo_id, encoder=encoder, speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODER, servo_id, [encoder], speed)

    def get_encoder(self, servo_id):
        """Obtain the specified joint potential value.

        Args:
            servo_id: (int) 1 - 6
                for gripper: Joint id 7
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.GET_ENCODER, servo_id, has_reply=True)

    def set_encoders(self, encoders, speed):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            encoders: A encoder list. len 6.
            speed: speed 1 ~ 100
        """
        self.calibration_parameters(encoders=encoders, speed=speed)
        return self._mesg(ProtocolCode.SET_ENCODERS, encoders, speed)

    def set_encoders_drag(self, encoders, speeds):
        """Send all encoders and speeds

        Args:
            encoders: encoders list.
            speeds: Obtained by the get_servo_speeds() method
        """
        self.calibration_parameters(encoders=encoders, drag_speeds=speeds)
        return self._mesg(ProtocolCode.SET_ENCODERS_DRAG, encoders, speeds)

    def get_joint_min_angle(self, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            joint_id: 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MIN_ANGLE, joint_id, has_reply=True)

    def get_joint_max_angle(self, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            joint_id: 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.GET_JOINT_MAX_ANGLE, joint_id, has_reply=True)

    def set_joint_max(self, joint_id, angle):
        """Set the joint maximum angle

        Args:
            joint_id: int.
                Joint id 1 - 6
                for gripper: Joint id 7
            angle: 0 ~ 180
        """
        self.calibration_parameters(joint_id=joint_id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MAX, joint_id, angle)

    def set_joint_min(self, joint_id, angle):
        """Set the joint minimum angle

        Args:
            joint_id: int.
                Joint id 1 - 6
                for gripper: Joint id 7
            angle: 0 ~ 180
        """
        self.calibration_parameters(joint_id=joint_id, angle=angle)
        return self._mesg(ProtocolCode.SET_JOINT_MIN, joint_id, angle)

    # servo control
    def is_servo_enable(self, servo_id):
        """To detect the connection state of a single joint

        Args:
            servo_id: 1 - 6
        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, servo_id, has_reply=True)

    def is_all_servo_enable(self):
        """Detect the connection status of all joints

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE, has_reply=True)

    def set_servo_data(self, servo_id, address, value, mode=None):
        """Set the data parameters of the specified address of the steering gear

        Args:
            servo_id: Serial number of articulated steering gear. 1 - 7
            address: Data address.
            value: 0 - 4096
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
        """

        unique_address = (0, 1, 2, 3, 4)
        if value in unique_address:
            raise ValueError(f"The address not right, Invalid address, address {unique_address} cannot be modified.")

        if mode is None:
            self.calibration_parameters(servo_id=servo_id, address=address, value=value)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, address, value)
        else:
            self.calibration_parameters(servo_id=servo_id, address=address, value=value, mode=mode)
            return self._mesg(ProtocolCode.SET_SERVO_DATA, servo_id, address, [value], mode)

    def get_servo_data(self, servo_id, address, mode=None):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            servo_id: Serial number of articulated steering gear.1 - 7
            address: Data address.
            mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

        Return:
            values 0 - 4096
        """
        if mode is not None:
            self.calibration_parameters(servo_id=servo_id, address=address, mode=mode)
            return self._mesg(ProtocolCode.GET_SERVO_DATA, servo_id, address, mode, has_reply=True)
        self.calibration_parameters(servo_id=servo_id, address=address)
        return self._mesg(ProtocolCode.GET_SERVO_DATA, servo_id, address, has_reply=True)

    def set_servo_calibration(self, servo_id):
        """The current position of the calibration joint actuator is the angle zero point,
            and the corresponding potential value is 2048.

        Args:
            servo_id: Serial number of articulated steering gear. 1 - 7
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, servo_id)

    def joint_brake(self, joint_id):
        """Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

        Args:
            joint_id:  1 - 6
        """
        self.calibration_parameters(joint_id=joint_id)
        return self._mesg(ProtocolCode.JOINT_BRAKE, joint_id)

    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: int 1 - 7
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id)

    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int 1 - 7
        """
        self.calibration_parameters(servo_id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id)

    # 夹爪控制
    def get_gripper_value(self, gripper_type=1):
        """Get the value of gripper.

        Args:
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper
                4: Flexible gripper

        Return:
            gripper value (int)
        """
        self.calibration_parameters(gripper_type=gripper_type)
        return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, gripper_type, has_reply=True)

    def set_gripper_state(self, gripper_state, gripper_speed, gripper_type=None, is_torque=None):
        """Set gripper switch state

        Args:
            gripper_state  (int): 0 - open, 1 - close, 254 - release
            gripper_speed (int): 1 ~ 100
            gripper_type (int): default 1
                1 : Adaptive gripper. default to adaptive gripper
                2 : 5 finger dexterous hand
                3 : Parallel gripper, this parameter can be omitted
                4 : Flexible gripper
            is_torque (int): When there is no type parameter, this parameter can be omitted.
                1: Force control
                0: Non-force control
        """
        self.calibration_parameters(
            gripper_state=gripper_state, gripper_speed=gripper_speed, gripper_type=gripper_type, is_torque=is_torque
        )
        args = [gripper_state, gripper_speed]
        if gripper_type is not None:
            args.append(gripper_type)
        if is_torque is not None:
            args.append(is_torque)
        return self._mesg(ProtocolCode.SET_GRIPPER_STATE, *args)

    def set_gripper_value(self, gripper_value, gripper_speed, gripper_type=None, is_torque=None):
        """Set gripper value

        Args:
            gripper_value (int): 0 ~ 100
            gripper_speed (int): 1 ~ 100
            gripper_type (int): default 1
                1: Adaptive gripper
                3: Parallel gripper, this parameter can be omitted
                4: Flexible gripper
            is_torque (int): When there is no type parameter, this parameter can be omitted.
                1: Force control
                0: Non-force control
        """
        self.calibration_parameters(
            gripper_value=gripper_value,
            speed=gripper_speed,
            gripper_type=gripper_type,
            is_torque=is_torque
        )
        args = [gripper_value, gripper_speed]
        if gripper_type is not None:
            args.append(gripper_type)
        if is_torque is not None:
            args.append(is_torque)
        return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, *args, has_reply=True)

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

    # Atom IO
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

    def set_tool_color(self, r=0, g=0, b=0):
        """Set the light color on the top of the robot arm.

        Args:
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(color=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, r, g, b)

    def is_tool_button_click(self):
        """Check whether the button on the end is pressed"""
        return self._mesg(ProtocolCode.IS_CONTROLLER_BUTTON_PRESSED, has_reply=True)

    def set_digital_output(self, pin_no, pin_signal):
        """Set the terminal atom io status

        Args:
            pin_no     (int): 1 or 2
            pin_signal (int): 0 / 1
        """

        if pin_no not in (1, 2):
            raise ValueError("pin_no must be 1 or 2")

        self.calibration_parameters(class_name=self.__class__.__name__, pin_signal=pin_signal)
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, pin_no, pin_signal)

    def get_digital_input(self, pin_no):
        """Get atom digital input value

        Args:
            pin_no (int): 1 or 2
        """
        if pin_no not in (1, 2):
            raise ValueError("pin_no must be 1 or 2")

        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, pin_no, has_reply=True)

    # Kinematic algorithm interface api
    def solve_inv_kinematics(self, target_coords, current_angles):
        """ Convert target coordinates to angles

        Args:
            target_coords: A float list of all coordinates.
            current_angles : A float list of all angle.

        Return:
            list: A float list of all angle.
        """
        self.calibration_parameters(coords=target_coords, angles=current_angles)
        angles = [self._angle2int(angle) for angle in current_angles]
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(target_coords[idx]))
        for angle in target_coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SOLVE_INV_KINEMATICS, coord_list, angles, has_reply=True)

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

    def set_reference_frame(self, reference_frame_type):
        """Set the base coordinate system

        Args:
            reference_frame_type: 0 - base 1 - tool.
        """
        self.calibration_parameters(reference_frame_type=reference_frame_type)
        return self._mesg(ProtocolCode.SET_REFERENCE_FRAME, reference_frame_type)

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

    def set_end_type(self, end_type):
        """Set end coordinate system

        Args:
            end_type: int
                0 - flange, 1 - tool
        """
        self.calibration_parameters(end_type=end_type)
        return self._mesg(ProtocolCode.SET_END_TYPE, end_type)

    def get_end_type(self):
        """Get end coordinate system

        Return:
            0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE, has_reply=True)

    # 9G Servo backgammon
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

    # Stdio api
    def get_angles_coords(self):
        """Get joint angles and coordinates"""
        return self._mesg(ProtocolCode.GET_ANGLES_COORDS, has_reply=True)

    # Servo state value
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

    # Drag the teach-in
    def start_drag_record(self):
        """Start track recording

        Return:
            Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_START_RECORD, has_reply=True)

    def end_drag_record(self):
        """End track recording

        Return:
             Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_END_RECORD, has_reply=True)

    def get_drag_record_data(self):
        """Get the recorded track

        Return:
            List of potential values (encoder values) and operating speeds of each joint
            eg: [J1_encoder, J1_run_speed,J2_encoder, J2_run_speed,J3_encoder, J3_run_speed,J4_encoder, J4_run_speed,J5_
            encoder, J5_run_speed,J6_encoder, J6_run_speed]
        """

        return self._mesg(ProtocolCode.DRAG_GET_RECORD_DATA, has_reply=True)

    def get_drag_record_len(self):
        """Get the total number of recorded points

        Return:
            Recording queue length
        """

        return self._mesg(ProtocolCode.DRAG_GET_RECORD_LEN, has_reply=True)

    def clear_drag_record_data(self):
        """Clear recording track

        Return:
            Recording queue length 0
        """

        return self._mesg(ProtocolCode.DRAG_CLEAR_RECORD_DATA, has_reply=True)


class MyCobot280RDKX5(MyCobot280RDKX5CommandGenerator):

    def __init__(self, port, baudrate=100_0000, timeout=0.1, debug=False, thread_lock=True):
        """
        Args:
            port     : port string
            baudrate : baud rate int, default 100_0000
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super().__init__(debug, thread_lock)
        self._serial_port = setup_serial_port(port=port, baudrate=baudrate, timeout=timeout)
        self._write = functools.partial(write, self)
        self._read = functools.partial(read, self)

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()


class MyCobot280RDKX5Socket(MyCobot280RDKX5CommandGenerator):
    """MyCobot 280 RDK X5 Socket Control Class

    server file: https://github.com/elephantrobotics/pymycobot/demo/Server_280_RDK_X5.py
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
    rdx_x5 = MyCobot280RDKX5("/dev/ttyS1", debug=True)
    # print(rdx_x5.get_angles())
    # rdx_x5.send_angle(6, 168, 100)
    rdx_x5.send_angles([0, 0, 0, 0, 0, 0], 100)
    time.sleep(3)
    # print(rdx_x5.get_coords())
    rdx_x5.send_coords([169.7, -65.8, 200.5, 178.14, -0.41, -89.98], 10)
    # mc_sock = MyCobot280RDKX5Socket('192.168.1.246', port=30002, debug=True)
    # print(mc_sock.send_angle(1, 100, 50))
    # print(mc_sock.get_quick_move_message())
    # print(mc_sock.set_gpio_mode(0))
    # print(mc_sock.setup_gpio_state(5, 1, initial=1))
    # print(mc_sock.set_gpio_output(5, 0))
    # print(mc_sock.get_gpio_input(5))


if __name__ == '__main__':
    main()
