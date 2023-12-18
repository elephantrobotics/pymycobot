# # -*- coding: utf-8 -*-
# v1.0.3
# 20230830
# https://linuxcnc.org/docs/2.7/html/config/python-interface.html
import platform
import time
import math
from enum import Enum

if platform.system() == "Linux" and platform.machine() == "aarch64":
    import linuxcnc as elerob
    import hal
    class JogMode(Enum):
        JOG_JOINT = elerob.ANGULAR - 1
        JOG_TELEOP = elerob.LINEAR - 1
from time import sleep

COORDS_EPSILON = 0.50
MAX_JOINTS = 6
MAX_CARTE = 3
jog_velocity = 1.0  # 100.0/60.0
angular_jog_velocity = 3600 / 60
MAX_PINS = 64




class RobotMoveState(Enum):
    IDLE_STATE = 0
    JOG_JOINT_STATE = 1
    JOG_AXIS_STATE = 2
    MOVE_AXIS_STATE = 3
    MOVE_JOINT_STATE = 4
    RUN_PROGRAM_STATE = 5


class Axis(Enum):
    X = 0
    Y = 1
    Z = 2
    RX = 3
    RY = 4
    RZ = 5


class Joint(Enum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3
    J5 = 4
    J6 = 5


class DI(Enum):
    """Available Digital Input Pins."""

    CAT_PHYSICAL_START = 15
    CAT_PHYSICAL_STOP = 16
    CAT_PHYSICAL_USER_DEFINE = 17

    J1_COMMUNICATION = 34
    J2_COMMUNICATION = 35
    J3_COMMUNICATION = 36
    J4_COMMUNICATION = 37
    J5_COMMUNICATION = 38
    J6_COMMUNICATION = 39

    IO_STOP_TRIGGERED = 40
    IO_RUN_TRIGGERED = 41
    HARDWARE_PAUSE_PRESSED = 43

    BRAKE_ACTIVATION_RUNNING = 44
    J1_SERVO_ENABLED = 45
    J2_SERVO_ENABLED = 46
    J3_SERVO_ENABLED = 47
    J4_SERVO_ENABLED = 48
    J5_SERVO_ENABLED = 49
    J6_SERVO_ENABLED = 50

    HARDWARE_FREE_MOVE = 51
    MOTION_ENABLE = 52

    J1_STATUS = 56
    J2_STATUS = 57
    J3_STATUS = 58
    J4_STATUS = 59
    J5_STATUS = 60
    J6_STATUS = 61

    EMERGENCY_STOP = 62
    POWER_ON_STATUS = 63


class DO(Enum):
    """Available Digital Output Pins."""

    BRAKE_ACTIVE_AUTO = 47

    BRAKE_MANUAL_MODE_ENABLE = 48
    J1_BRAKE_RELEASE = 49
    J2_BRAKE_RELEASE = 50
    J3_BRAKE_RELEASE = 51
    J4_BRAKE_RELEASE = 52
    J5_BRAKE_RELEASE = 53
    J6_BRAKE_RELEASE = 54

    SKIP_INIT_ERRORS = 55
    PROGRAM_AUTO_RUNNING = 56
    POWER_ON_RELAY_1 = 57
    POWER_ON_RELAY_2 = 58
    SOFTWARE_FREE_MOVE = 60


class AI(Enum):
    """Available Analog Input Pins."""

    J1_VOLTAGE = 31
    J2_VOLTAGE = 32
    J3_VOLTAGE = 33
    J4_VOLTAGE = 34
    J5_VOLTAGE = 35
    J6_VOLTAGE = 36

    J1_TEMPERATURE = 37
    J2_TEMPERATURE = 38
    J3_TEMPERATURE = 39
    J4_TEMPERATURE = 40
    J5_TEMPERATURE = 41
    J6_TEMPERATURE = 42

    J1_WINDING_A_CURRENT = 43
    J2_WINDING_A_CURRENT = 44
    J3_WINDING_A_CURRENT = 45
    J4_WINDING_A_CURRENT = 46
    J5_WINDING_A_CURRENT = 47
    J6_WINDING_A_CURRENT = 48

    J1_WINDING_B_CURRENT = 49
    J2_WINDING_B_CURRENT = 50
    J3_WINDING_B_CURRENT = 51
    J4_WINDING_B_CURRENT = 52
    J5_WINDING_B_CURRENT = 53
    J6_WINDING_B_CURRENT = 54

    ROBOT = 55

    ROBOT_AVG_POWER = 56
    CONTROLLER_TEMPERATURE = 57

    J1_CURRENT = 58
    J2_CURRENT = 59
    J3_CURRENT = 60
    J4_CURRENT = 61
    J5_CURRENT = 62
    J6_CURRENT = 63


class AO(Enum):
    """Available Analog Output Pins."""

    LED_LIGHT = 16

    ACCELERATION = 41

    TOOL_FORCE = 48
    TOOL_SPEED = 49
    STOPPING_DISTANCE = 50
    STOPPING_TIME = 51
    POWER_LIMIT = 52

    PAYLOAD = 53

    J1_TORQUE = 56
    J2_TORQUE = 57
    J3_TORQUE = 58
    J4_TORQUE = 59
    J5_TORQUE = 60
    J6_TORQUE = 61

    XY_AXIS_TORQUE = 62
    Z_AXIS_TORQUE = 63


class JointState(Enum):
    OK = 0
    ERROR = 1
    POWERED_OFF = 2


class Pro630:
    def __init__(self):
        self.c = elerob.command()
        self.s = elerob.stat()
        self.e = elerob.error_channel()
        self.robot_state = RobotMoveState.IDLE_STATE
        self.command_id = 0
        self.g = hal.component("halgpio_py", "halgpio")
        self.init_pins()
        self.g.ready()

    def init_pins(self):
        """Inits HAL pins in halgpio component."""
        for i in range(MAX_PINS):
            self.g.newpin("digital-in-" + str(i).zfill(2), hal.HAL_BIT, hal.HAL_IN)
            self.g.newpin("digital-out-" + str(i).zfill(2), hal.HAL_BIT, hal.HAL_OUT)
            self.g.newpin("analog-in-" + str(i).zfill(2), hal.HAL_FLOAT, hal.HAL_IN)
            self.g.newpin("analog-out-" + str(i).zfill(2), hal.HAL_FLOAT, hal.HAL_OUT)

        # 0 .. MAX_JOINTS: joint status word
        # MAX_JOINTS .. MAX_JOINTS * 2: joint error mask
        for i in range(MAX_JOINTS * 2):
            self.g.newpin("U32-in-" + str(i).zfill(2), hal.HAL_U32, hal.HAL_IN)

        for i in range(MAX_JOINTS):
            self.g.newpin(str(i) + ".min_limit", hal.HAL_FLOAT, hal.HAL_OUT)
            self.g.newpin(str(i) + ".max_limit", hal.HAL_FLOAT, hal.HAL_OUT)

        self.g.newpin("serial.0.xtorq", hal.HAL_FLOAT, hal.HAL_OUT)
        self.g.newpin("serial.0.ytorq", hal.HAL_FLOAT, hal.HAL_OUT)
        self.g.newpin("serial.0.ztorq", hal.HAL_FLOAT, hal.HAL_OUT)
        self.g.newpin("serial.0.atorq", hal.HAL_FLOAT, hal.HAL_OUT)
        self.g.newpin("serial.0.btorq", hal.HAL_FLOAT, hal.HAL_OUT)
        self.g.newpin("serial.0.ctorq", hal.HAL_FLOAT, hal.HAL_OUT)

        self.g.newpin("serial.0.sensor_torq_open", hal.HAL_U32, hal.HAL_OUT)
        self.g.newpin("serial.0.sensor_error_pkgs", hal.HAL_U32, hal.HAL_OUT)
        self.g.newpin("serial.0.sensor_recv_timeout", hal.HAL_U32, hal.HAL_OUT)
        self.g.newpin("serial.0.motion_flexible", hal.HAL_U32, hal.HAL_OUT)

    def power_on(self):
        """Powers on and prepares for operation the robot."""
        if self.s.task_state == elerob.STATE_ESTOP:
            self.send_estop_reset()
        self.set_digital_out(DO.POWER_ON_RELAY_1, 1)
        time.sleep(0.25)
        self.set_digital_out(DO.POWER_ON_RELAY_2, 1)
        time.sleep(0.25)
        self.set_digital_out(DO.POWER_ON_RELAY_2, 0)

        self.set_free_move(False)

        self.set_digital_out(DO.BRAKE_ACTIVE_AUTO, 0)
        time.sleep(0.1)
        self.set_digital_out(DO.BRAKE_ACTIVE_AUTO, 1)

    def power_on_only(self):
        """Only powers on the robot."""
        self.set_digital_out(DO.POWER_ON_RELAY_1, 1)
        time.sleep(0.25)
        self.set_digital_out(DO.POWER_ON_RELAY_2, 1)
        time.sleep(0.25)
        self.set_digital_out(DO.POWER_ON_RELAY_2, 0)
        self.send_estop()
        self.set_free_move(False)

    def power_off(self):
        """Powers off the robot."""
        self.set_digital_out(DO.POWER_ON_RELAY_1, 0)
        self.set_digital_out(DO.POWER_ON_RELAY_2, 0)
        self.set_digital_out(DO.BRAKE_ACTIVE_AUTO, 0)

    def get_coords(self):
        """Returns current robot coordinates.

        Returns:
            list[float]: list of 6 float values for each axis
        """
        c = self.get_actl_pos_float()
        return c

    def set_coords(self, coords, speed):
        """_summary_

        Args:
            coords (_type_): _description_
            speed (_type_): _description_
        """
        if self.is_in_position(coords, True):
            return
        self.set_robot_move_state(RobotMoveState.MOVE_AXIS_STATE, 0, 0)
        self.send_mdi("G01F" + str(speed) + self.coords_to_gcode(coords))

    def get_coord(self, axis):
        """_summary_

        Args:
            axis (_type_): _description_

        Returns:
            _type_: _description_
        """
        return self.get_coords()[axis]

    def set_coord(self, axis, coord, speed):
        """_summary_

        Args:
            axis (_type_): _description_
            coord (_type_): _description_
            speed (_type_): _description_
        """
        coords = self.get_coords()
        coords[axis] = coord
        self.set_coords(coords, speed)

    def get_angles(self):
        """_summary_"""
        return self.get_actl_joint_float()

    def set_angles(self, angles, speed):
        """_summary_

        Args:
            angles (_type_): _description_
            speed (_type_): _description_
        """
        if self.is_in_position(angles, False):
            return
        self.set_robot_move_state(RobotMoveState.MOVE_JOINT_STATE, 0, 0)
        self.send_mdi("G38.3F" + str(speed) + self.angles_to_gcode(angles))

    def get_angle(self, joint):
        """_summary_"""
        return self.get_angles()[joint]

    def set_angle(self, joint, angle, speed):
        """_summary_

        Args:
            joint (_type_): _description_
            angle (_type_): _description_
            speed (_type_): _description_
        """
        angles = self.get_angles()
        angles[joint] = angle
        self.set_angles(angles, speed)

    def get_speed(self):
        """_summary_"""
        return self.get_current_velocity()

    def state_on(self):
        """Sets robot state to ON.

        Returns:
            bool: True if success, False otherwise.
        """
        self.s.poll()
        if self.s.estop:  # FIXME: add errors check...
            return False
        if self.s.task_mode != elerob.MODE_MANUAL:
            self.c.mode(elerob.MODE_MANUAL)
            self.c.wait_complete()
            time.sleep(0.2)
        if self.s.motion_mode != elerob.TRAJ_MODE_FREE:
            self.c.teleop_enable(0)
            self.c.wait_complete()
        self.c.state(elerob.STATE_ON)
        self.c.wait_complete()
        if self.state_check():
            self.send_mdi("G64 P1")
            self.task_stop()
        self.set_robot_move_state(RobotMoveState.IDLE_STATE, 0, 0)
        return True

    def state_off(self):
        """_summary_"""
        self.c.state(elerob.STATE_OFF)
        self.c.wait_complete()

    def state_check(self):
        """Checks if robot's state is ready for operation.

        Currently only checks motion enable flag.

        Returns:
            bool: True if robot state is ready for operation, False otherwise.
        """
        self.s.poll()
        return self.get_motion_enabled()

    def jog_angle(self, joint, direction, speed):
        """_summary_

        Args:
            joint (Joint): _description_
            direction (_type_): _description_
            speed (_type_): _description_
        """
        self.set_robot_move_state(RobotMoveState.JOG_JOINT_STATE, joint, direction)
        self.command_id += 1
        self.jog_continuous(JogMode.JOG_JOINT, joint, direction, speed, self.command_id)

    def jog_coord(self, axis, direction, speed):
        """_summary_

        Args:
            axis (Axis): _description_
            direction (_type_): _description_
            speed (_type_): _description_
        """
        self.set_robot_move_state(RobotMoveState.JOG_AXIS_STATE, axis, direction)
        self.command_id += 1
        self.jog_continuous(JogMode.JOG_TELEOP, axis, direction, speed, self.command_id)

    def check_running(self):
        """_summary_

        Returns:
            bool: _description_
        """
        return not self.is_in_commanded_position()

    def is_in_position(self, coords, is_linear):
        """_summary_

        Args:
            coords (_type_): _description_
            is_linear (bool): _description_
        """
        if is_linear:
            return self.coords_equal(self.get_coords(), coords)
        else:
            return self.angles_equal(self.get_angles(), coords)

    def set_free_move(self, on):
        """_summary_

        Args:
            on (_type_): _description_
        """
        if on:
            self.set_jog_mode(JogMode.JOG_TELEOP)
        self.set_digital_out(DO.SOFTWARE_FREE_MOVE, on)

    def is_software_free_move(self):
        """_summary_

        Returns:
            bool: _description_
        """
        return self.get_digital_out(DO.SOFTWARE_FREE_MOVE)

    def is_hardware_free_move(self):
        """_summary_

        Returns:
            bool: _description_
        """
        return self.get_digital_in(DI.HARDWARE_FREE_MOVE)

    def set_payload(self, payload_weight):
        """_summary_

        Args:
            payload_weight (_type_): _description_
        """
        self.set_analog_out(AO.PAYLOAD, payload_weight)

    def is_program_run_finished(self):
        """_summary_"""
        return not self.is_program_running(True)

    def is_program_paused(self):
        """_summary_"""
        self.s.poll()
        return self.s.paused

    def read_next_error(self):
        """_summary_"""
        self.e.poll()
        raise NotImplementedError

    def is_power_on(self):
        """_summary_"""
        return self.get_digital_in(DI.POWER_ON_STATUS)

    def is_servo_enabled(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        return self.get_digital_in(DI(DI.J1_SERVO_ENABLED.value + joint.value))

    def is_all_servo_enabled(self):
        """_summary_"""
        return (
            self.is_servo_enabled(Joint.J1)
            and self.is_servo_enabled(Joint.J2)
            and self.is_servo_enabled(Joint.J3)
            and self.is_servo_enabled(Joint.J4)
            and self.is_servo_enabled(Joint.J5)
            and self.is_servo_enabled(Joint.J6)
        )

    def get_joint_min_pos_limit(self, joint):
        """_summary_

        Args:
            joint_number (Joint): _description_
        """
        return self.g[str(joint.value) + ".min_limit"]

    def set_joint_min_pos_limit(self, joint, limit):
        """_summary_

        Args:
            joint_number (Joint): _description_
            limit (float): _description_
        """
        self.g[str(joint.value) + ".min_limit"] = limit

    def get_joint_max_pos_limit(self, joint):
        """_summary_

        Args:
            joint_number (Joint): _description_
        """
        return self.g[str(joint.value) + ".max_limit"]

    def set_joint_max_pos_limit(self, joint, limit):
        """_summary_

        Args:
            joint_number (Joint): _description_
            limit (float): _description_
        """
        self.g[str(joint.value) + ".max_limit"] = limit

    def set_joint_max_velocity(self, joint, limit):
        """_summary_

        Args:
            joint (_type_): _description_
            limit (_type_): _description_
        """
        raise NotImplementedError

    def get_current_cnc_mode(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        self.s.poll()
        return self.s.task_mode

    def send_estop_reset(self):
        """_summary_"""
        self.c.state(elerob.STATE_ESTOP_RESET)
        self.c.wait_complete()

    def send_estop(self):
        """_summary_"""
        self.c.state(elerob.STATE_ESTOP)
        self.c.wait_complete()

    def get_acceleration(self):
        """_summary_"""
        return self.get_analog_out(AO.ACCELERATION)

    def send_mode(self, mode):
        """Sets mode to mode.

        Args:
            mode (_type_): one of elerob.MODE_MDI,
                                  elerob.MODE_MANUAL,
                                  elerob.MODE_AUTO.

        Returns:
            bool: always returns True
        """
        if self.s.task_mode == mode:
            return True
        self.c.mode(mode)
        self.c.wait_complete()
        return True

    def joint_brake(self, joint, release):
        """_summary_

        Args:
            joint (Joint): joint
            release (bool): True or False
        """
        self.set_digital_out(joint + DO.J1_BRAKE_RELEASE, release)

    def is_cnc_in_mdi_mode(self):
        """_summary_

        Returns:
            bool: _description_
        """
        self.s.poll()
        return self.s.task_mode == elerob.MODE_MDI

    def set_cnc_in_mdi_mode(self):
        """_summary_"""
        self.ensure_mode(elerob.MODE_MDI)

    def get_motion_line(self):
        """_summary_"""
        self.s.poll()
        return self.s.motion_line

    def send_teleop(self, enable):
        """_summary_

        Args:
            enable (int): _description_
        """
        self.c.teleop_enable(enable)
        self.c.wait_complete()

    def get_robot_status(self):
        """_summary_"""
        return self.state_check()

    def get_robot_temperature(self):
        """Returns robot (currently CPU) temperature.

        Returns:
            float: robot temperature
        """
        cpu_temp = CPUTemperature()
        return cpu_temp.temperature

    def get_robot_power(self):
        """Returns robot power in Watts (W)."""
        robot_power = 17
        for i in range(MAX_JOINTS):
            robot_power += self.get_joint_voltage(Joint(i)) * math.fabs(
                self.get_joint_current(Joint(i))
            )
        return robot_power

    def get_joint_state(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        joint_status = self.g["U32-in-" + str(joint.value).zfill(2)]
        if joint_status & (1 << 1):
            if joint_status & (1 << 3):
                return JointState.ERROR
            return JointState.OK
        return JointState.POWERED_OFF

    def get_joint_temperature(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        return self.get_analog_in(AI(AI.J1_TEMPERATURE.value + joint.value))

    def get_joint_communication(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        return not self.get_digital_in(DI(DI.J1_COMMUNICATION.value + joint.value))

    def get_joint_voltage(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        return self.get_analog_in(AI(AI.J1_VOLTAGE.value + joint.value))

    def get_joint_current(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_
        """
        return self.get_analog_in(AI(AI.J1_WINDING_A_CURRENT.value + joint.value))

    def get_joint_error_mask(self, joint):
        """_summary_

        Args:
            joint (Joint): _description_

        Returns:
            int: error mask
        """
        return self.g["U32-in-" + str(joint + MAX_JOINTS).zfill(2)]

    def set_power_limit(self, power_limit):
        """_summary_

        Args:
            power_limit (float): _description_
        """
        self.set_analog_out(AO.POWER_LIMIT, power_limit)

    def get_power_limit(self):
        """_summary_"""
        return self.get_analog_out(AO.POWER_LIMIT)

    def set_stopping_time(self, stopping_time):
        """_summary_

        Args:
            stopping_time (float): _description_
        """
        self.set_analog_out(AO.STOPPING_TIME, stopping_time)

    def get_stopping_time(self):
        """_summary_"""
        return self.get_analog_out(AO.STOPPING_TIME)

    def set_stopping_distance(self, stopping_distance):
        """_summary_

        Args:
            stopping_distance (float): _description_
        """
        self.set_analog_out(AO.STOPPING_DISTANCE, stopping_distance)

    def get_stopping_distance(self):
        """_summary_"""
        return self.get_analog_out(AO.STOPPING_DISTANCE)

    def set_tool_speed(self, tool_speed):
        """_summary_

        Args:
            tool_speed (float): _description_
        """
        self.set_analog_out(AO.TOOL_SPEED, tool_speed)

    def get_tool_speed(self):
        """_summary_"""
        return self.get_analog_out(AO.TOOL_SPEED)

    def set_tool_force(self, tool_force):
        """_summary_

        Args:
            tool_force (float): _description_
        """
        self.set_analog_out(AO.TOOL_FORCE, tool_force)

    def get_tool_force(self):
        """_summary_"""
        return self.get_analog_out(AO.TOOL_FORCE)

    def start_force_sensor(self):
        """_summary_"""
        raise NotImplementedError

    def stop_force_sensor(self):
        """_summary_"""
        raise NotImplementedError

    def set_motion_flexible(self, flexible):
        """_summary_

        Args:
            flexible (int): _description_
        """
        self.g["serial.0.motion_flexible"] = flexible

    def get_actual_position(self):
        """Returns actual coord position."""
        return self.get_actl_pos_float()

    def get_actual_joints(self):
        """Returns actual joints angles."""
        return self.get_actl_joint_float()

    def is_in_commanded_position(self):
        """Returns machine-in-position flag. True if commanded position
        equals actual position.
        返回机器就位标志。如果指令位置为True，等于实际位置。

        Returns:
            bool: machine-in-position flag
        """
        self.s.poll()
        return self.s.inpos

    def set_acceleration(self, acceleration):
        """_summary_

        Args:
            acceleration (_type_): _description_
        """
        self.set_analog_out(AO.ACCELERATION, acceleration)

    def jog_continuous(self, jog_mode, joint_or_axis, direction, speed, jog_id):
        """Start axis or joint jog.

        Args:
            jog_mode (_type_): jog mode (axis or joint)
            joint_or_axis (_type_): axis or joint
            direction (_type_): direction of movement
            speed (_type_): speed of movement
            jog_id: jog_id
        """
        if not self.set_jog_mode(jog_mode):
            return False
        if self.s.task_state != elerob.STATE_ON:
            return
        if (
            (jog_mode == JogMode.JOG_JOINT)
            and (self.s.motion_mode == elerob.TRAJ_MODE_TELEOP)
        ) or (
            (jog_mode == JogMode.JOG_TELEOP)
            and (self.s.motion_mode != elerob.TRAJ_MODE_TELEOP)
        ):
            return
        if jog_mode and (joint_or_axis < 0 or joint_or_axis >= MAX_JOINTS):
            return
        if not jog_mode and (joint_or_axis < 0):
            return
        if jog_id != self.command_id:
            print("jog_continuous cancelled: some delay: jog_id != command_id")
            return
        self.c.jog(elerob.JOG_CONTINUOUS, jog_mode, joint_or_axis, direction * speed)

    def jog_increment(self, jog_mode, joint_or_axis, incr, speed):
        """Move joint or axis by specified value.

        Args:
            jogmode (_type_): move by axis or joint
            joint_or_axis (_type_): axis or joint
            incr (_type_): distance or angle to move for
            speed (_type_): speed of movement

        Returns:
            bool: True if jog started successfully, False otherwise
        """
        if not self.set_jog_mode(jog_mode):
            return False
        if self.s.task_state != elerob.STATE_ON:
            return
        if (
            (jog_mode == JogMode.JOG_JOINT)
            and (self.s.motion_mode == elerob.TRAJ_MODE_TELEOP)
        ) or (
            (jog_mode == JogMode.JOG_TELEOP)
            and (self.s.motion_mode != elerob.TRAJ_MODE_TELEOP)
        ):
            return
        if jog_mode and (joint_or_axis < 0 or joint_or_axis >= MAX_JOINTS):
            return
        if not jog_mode and (joint_or_axis < 0):
            return
        direction = 1 if incr >= 0 else -1
        self.c.jog(
            elerob.JOG_INCREMENT,
            jog_mode,
            joint_or_axis,
            speed * direction,
            math.fabs(incr),
        )
        return True

    def jog_absolute(self, joint_or_axis, jog_mode, pos, speed):
        """_summary_

        Args:
            joint_or_axis (Joint): _description_
            jog_mode (JogMode): usually JogMode.JOG_JOINT
            pos (_type_): _description_
            speed (_type_): _description_
        """
        if not self.set_jog_mode(jog_mode):
            return
        if self.s.task_state != elerob.STATE_ON:
            return
        if (
            (jog_mode == JogMode.JOG_JOINT)
            and (self.s.motion_mode == elerob.TRAJ_MODE_TELEOP)
        ) or (
            (jog_mode == JogMode.JOG_TELEOP)
            and (self.s.motion_mode != elerob.TRAJ_MODE_TELEOP)
        ):
            return
        if jog_mode and (joint_or_axis < 0 or joint_or_axis >= MAX_JOINTS):
            return
        if not jog_mode and (joint_or_axis < 0):
            return
        if jog_mode == JogMode.JOG_JOINT:
            self.set_angle(joint_or_axis, pos, speed)
        elif jog_mode == JogMode.JOG_TELEOP:
            self.set_coord(joint_or_axis, pos, speed)
        else:
            print("Jog Absolute: Wrong Jog Mode.")
            return

    def jog_stop(self, joint_or_axis, jog_mode):
        """_summary_

        Args:
            joint_or_axis (_type_): _description_
            jog_mode (_type_): _description_
        """
        if jog_mode and (joint_or_axis < 0 or joint_or_axis >= MAX_JOINTS):
            return False
        if not jog_mode and (joint_or_axis < 0):
            return False
        self.c.jog(elerob.JOG_STOP, jog_mode, joint_or_axis)

    def is_task_idle(self):
        """Returns True if program is not running or finished.

        Returns:
            bool: True if program is not running or finished, False otherwise.
        """
        self.s.poll()
        return self.s.exec_state <= elerob.EXEC_DONE and not self.is_program_running()

    def start_3d_mouse(self, enable):
        """Starts movement my space nav mouse.

        Args:
            enable (bool): start if True, stop if False
        """
        raise NotImplementedError

    def set_robot_move_state(self, new_robot_state, joint_or_axis, direction):
        """_summary_

        Args:
            new_robot_state (_type_): _description_
            joint_or_axis (_type_): _description_
            direction (_type_): _description_
        """
        if new_robot_state == self.robot_state:
            return

        if new_robot_state == RobotMoveState.JOG_JOINT_STATE:
            self.command_id += 1
            self.jog_continuous(
                JogMode.JOG_JOINT, joint_or_axis, direction, 1, self.command_id
            )
            time.sleep(0.1)
        elif new_robot_state == RobotMoveState.JOG_AXIS_STATE:
            self.command_id += 1
            self.jog_continuous(
                JogMode.JOG_TELEOP, joint_or_axis, direction, 1, self.command_id
            )
            time.sleep(0.1)
        elif new_robot_state == RobotMoveState.MOVE_AXIS_STATE:
            self.send_mdi("G92.2")
            self.send_mdi("G01F2000" + self.coords_to_gcode(self.get_coords()))
        elif new_robot_state == RobotMoveState.MOVE_JOINT_STATE:
            self.send_mdi("G92.2")
            self.send_mdi("G38.3F1000" + self.angles_to_gcode(self.get_angles()))
        elif new_robot_state == RobotMoveState.RUN_PROGRAM_STATE:
            self.send_mdi("G92.2")
        self.robot_state = new_robot_state

    def update(self):
        """Updates robot data. Get current robot state. Run before any
        get function as needed.
        """
        self.s.poll()

    def set_state(self, s):
        """Change robot state to one of:
            elerob.STATE_ESTOP (1),
            elerob.STATE_ESTOP_RESET (2),
            elerob.STATE_OFF (3),
            elerob.STATE_ON (4)

        Args:
            s (str): robot state, can be "STATE_ON", "STATE_OFF", "STATE_ESTOP",
                    "STATE_ESTOP_RESET".
        """
        if "STATE_ON" == s:
            # self.c.state( elerob.STATE_ESTOP_RESET)
            # time.sleep(0.01)
            self.c.state(elerob.STATE_ON)
        elif "STATE_OFF" == s:
            self.c.state(elerob.STATE_OFF)
        elif "STATE_ESTOP" == s:
            self.c.state(elerob.STATE_ESTOP)
        elif "STATE_ESTOP_RESET" == s:
            self.c.state(elerob.STATE_ESTOP_RESET)

    def set_feedrate(self, new_val):
        """Set robot speed in percents.

        Args:
            new_val (int): speed value in percents, 0-100.
        """
        try:
            value = int(new_val)
        except ValueError:
            return
        value = value / 100.0
        if value < 0.0:
            value = 0.0
        elif value >= 1.0:
            value = 1.0
        self.c.feedrate(value)

    def send_home(self, axis_num=-1):
        """Home axis or all axes (-1 means home all axes).
        Home means sets current joint or axis value as home position.

        Args:
            axis_num (int, optional): Axis to home. -1 means home all axes.
                                     Defaults to -1.
        """
        self.c.home(axis_num)

    def is_all_homed(self):
        """Checks if all axes are homed (in home position).

        Returns:
            bool: True if all axes are homed, False otherwise.
        """
        is_homed = True
        self.s.poll()
        for i, h in enumerate(self.s.homed):
            if self.s.axis_mask & (1 << i):
                is_homed = is_homed and h
        return is_homed

    def get_axis_limits(self):
        """Returns axis limits.

        Returns:
            list: axes limits.
        """
        limits = []
        for i, limit in enumerate(self.s.limit):
            if self.s.axis_mask & (1 << i):
                limits.append(limit)
        return limits

    def ensure_mdi(self):
        """Sets mode to MDI."""
        if not self.manual_ok():
            return
        self.ensure_mode(elerob.MODE_MDI)

    def send_mdi(self, program):
        """Send command to robot.

        Args:
            program (str): G-code command

        Returns:
            bool: True if command sent successfully, false otherwise
        """
        self.s.poll()
        if self.s.task_mode != elerob.MODE_MDI:
            self.send_mode(elerob.MODE_MDI)
        self.s.poll()
        if self.s.task_mode != elerob.MODE_MDI:
            print("send_mdi error: task_mode is not MODE_MDI")
            return False
        self.c.mdi(program)
        return True

    def send_mdi_wait(self, program):
        """Send command and wait for it to complete

        Args:
            program (str): G-code command
        """
        self.c.mdi(program)
        self.c.wait_complete()

    # TODO 不明白什么意思
    def set_optional_stop(self, on=0):
        self.c.set_optional_stop(on)
        self.c.wait_complete()

    # TODO 不明白什么意思
    def set_block_delete(self, on=0):
        self.c.set_block_delete(on)
        self.c.wait_complete()

    def is_program_running(self, do_poll=True):
        """Checks if robot is moving

        Args:
            do_poll (bool, optional): if need to update robot state. Defaults to True.

        Returns:
            bool: True if robot is moving, false otherwise
        """
        if do_poll:
            self.s.poll()
        return (
            self.s.task_mode == elerob.MODE_AUTO
            and self.s.interp_state != elerob.INTERP_IDLE
        )

    # TODO 不明白什么意思
    def manual_ok(self, do_poll=True):
        if do_poll:
            self.s.poll()
        if self.s.task_state != elerob.STATE_ON:
            return False
        return self.s.interp_state == elerob.INTERP_IDLE

    def ensure_mode(self, m, *p):
        """If elerob is not already in one of the model given, switch it to
        the first mode: elerob.MODE_MDI, elerob.MODE_MANUAL, elerob.MODE_AUTO.

        Args:
            m : mode or "STEP_MODE"

        Returns:
            bool: True if mode is m or in p
        """
        if "STEP_MODE" == m:
            m = elerob.MODE_MDI
        self.s.poll()
        if self.s.task_mode == m or self.s.task_mode in p:
            return True
        if self.is_program_running(do_poll=False):
            return False
        return self.send_mode(m)

    def program_open(self, program_file_path):
        """Opens g-code file.

        Args:
            program_file_path (str): program to open
        """
        self.c.program_open(program_file_path)

    def program_run(self, mode, start_line):
        """Run g-code file starting from given line.
        从给定行开始运行g-code文件

        Args:
            mode (int): elerob.AUTO_RUN
            start_line (int): start line (first line is 0)

        Example:
            elerob.program_run(elerob.AUTO_RUN, 0)
        """
        self.set_robot_move_state(RobotMoveState.RUN_PROGRAM_STATE, 0, 0)
        self.s.poll()
        if len(self.s.file) == 0:
            return False
        self.send_mode(elerob.MODE_AUTO)
        self.c.auto(mode, start_line)
        self.c.wait_complete()

    def get_current_line(self):
        """Returns current executing line in g-code file.

        Returns:
            int: current executing line
        """
        self.s.poll()
        if self.s.task_mode != elerob.MODE_AUTO or self.s.interp_state not in (
            elerob.INTERP_READING,
            elerob.INTERP_WAITING,
        ):
            return
        self.ensure_mode(elerob.MODE_AUTO)
        return self.s.current_line

    def get_current_gcodes(self):
        """Get current execution G-code.

        Returns active G-codes for each modal group.

        Returns:
            tuple of integers: active G-codes
        """
        self.s.poll()
        return self.s.gcodes

    def set_max_velocity(self, velocity):
        """Sets maximum velocity.

        Args:
            velocity (float): max velocity to set
        """
        self.c.maxvel(float(velocity) / 60.0)

    def total_lines(self):
        """Get current line.
           Get the total of current G code lines.

        Returns:
            int: line interpreter is currently reading
        """
        self.s.poll()
        if self.s.task_mode != elerob.MODE_AUTO or self.s.interp_state not in (
            elerob.INTERP_READING,
            elerob.INTERP_WAITING,
        ):
            return
        self.ensure_mode(elerob.MODE_AUTO)
        return self.s.read_line

    def prog_exec_status(self):
        """Returns interpreter's current state.
        返回解释器当前状态

        Returns:
            int: Interpreter state: INTERP_IDLE (1),
                                    INTERP_READING (2),
                                    INTERP_PAUSED (3),
                                    INTERP_WAITING (4)
        """
        self.s.poll()
        return self.s.interp_state

    def elerob_status(self):
        """Returns current task state.

        Returns:
            int: Current task state, one of:
                    elerob.STATE_ESTOP (1),
                    elerob.STATE_ESTOP_RESET (2),
                    elerob.STATE_OFF (3),
                    elerob.STATE_ON (4)
        """
        self.s.poll()
        return self.s.task_state

    def optional_stop_resume(self):
        """Resume program after optional stop."""
        self.s.poll()
        if not self.s.paused:
            return
        if self.s.task_mode not in (elerob.MODE_AUTO, elerob.MODE_MDI):
            return
        self.ensure_mode(elerob.MODE_AUTO, elerob.MODE_MDI)
        self.c.auto(elerob.AUTO_RESUME)

    def program_pause(self):
        """Pause program."""
        self.s.poll()
        if self.s.task_mode != elerob.MODE_AUTO or self.s.interp_state not in (
            elerob.INTERP_READING,
            elerob.INTERP_WAITING,
        ):
            return
        self.ensure_mode(elerob.MODE_AUTO)
        self.c.auto(elerob.AUTO_PAUSE)

    def program_resume(self):
        """Resume program after pause."""
        if self.s.task_mode not in (elerob.MODE_AUTO, elerob.MODE_MDI):
            return
        self.ensure_mode(elerob.MODE_AUTO, elerob.MODE_MDI)
        self.s.poll()
        if self.s.paused:
            self.c.auto(elerob.AUTO_RESUME)

        elif self.s.interp_state != elerob.INTERP_IDLE:
            # self.c.auto(elerob.AUTO_PAUSE)
            pass

    def task_stop(self):
        """Stop robot and wait for stop complete."""
        self.c.abort()
        self.c.wait_complete()
        if not self.is_task_idle():
            self.c.abort()
            self.c.wait_complete()
            self.s.poll()
            print(
                f"Warning: Failed Task abort: task_mode={self.s.task_mode}, exec_status={self.s.exec_state}, interp_state={self.s.interp_state}"
            )

    def task_stop_async(self):
        """Stop robot without waiting for stop to complete."""
        self.c.abort()

    def get_digital_in(self, pin_number):
        """Returns digital input pin value. One of DI Enum.

        Args:
            pin_number (DI): pin number

        Returns:
            bool: pin value
        """
        return self.g["digital-in-" + str(pin_number.value).zfill(2)]

    def get_digital_out(self, pin_number):
        """Returns digital output pin value. One of DO Enum.

        Args:
            pin_number (DO): pin number

        Returns:
            bool: pin value
        """
        return self.g["digital-out-" + str(pin_number.value).zfill(2)]

    def set_digital_out(self, pin_number, pin_value):
        """Sets digital output pin value. One of DO Enum.

        Args:
            pin_number (DO): pin number
            pin_value (bool): pin value
        """
        self.g["digital-out-" + str(pin_number.value).zfill(2)] = pin_value

    def get_analog_in(self, pin_number):
        """Returns analog input pin value. One of AI Enum.

        Args:
            pin_number (AI): pin number

        Returns:
            float: pin value
        """
        return self.g["analog-in-" + str(pin_number.value).zfill(2)]

    def get_analog_out(self, pin_number):
        """Returns analog output pin value. One of AO Enum.

        Args:
            pin_number (AO): pin number

        Returns:
            float: pin value
        """
        return self.g["analog-out-" + str(pin_number.value).zfill(2)]

    def set_analog_out(self, pin_number, pin_value):
        """Sets analog output pin value. One of AO Enum.

        Args:
            pin_number (AO): pin number
            pin_value (float): pin value
        """
        self.g["analog-out-" + str(pin_number.value).zfill(2)] = pin_value

    def get_axis_velocity(self, axis):
        """Return axis velocity.
        获取单关节速度

        Args:
            axis (int): axis number

        Returns:
            float: axis velocity
        """
        if 0 <= axis <= 9:
            return self.s.axis[axis]["velocity"]
        return None

    def get_current_command(self):
        """Returns current executing command

        Returns:
            str: current executing command
        """
        self.s.poll()
        return self.s.command

    def get_error_info(self):
        """Returns error info.

        Returns:
            _type_: Error info.
        """
        return self.e.poll()

    def get_cmd_pos(self):
        """Returns trajectory position.
        返回轨迹位置

        Returns:
            str: string of list of values
        """
        self.s.poll()
        return str(
            [
                "%5.3f" % self.s.position[0],
                "%5.3f" % self.s.position[1],
                "%5.3f" % self.s.position[2],
                "%5.3f" % self.s.position[3],
                "%5.3f" % self.s.position[4],
                "%5.3f" % self.s.position[5],
            ]
        )

    def get_actl_pos(self):
        """Returns current trajectory position.

        Returns:
            str: string of list of values
        """
        self.s.poll()
        return str(
            [
                "%5.3f" % self.s.actual_position[0],
                "%5.3f" % self.s.actual_position[1],
                "%5.3f" % self.s.actual_position[2],
                "%5.3f" % self.s.actual_position[3],
                "%5.3f" % self.s.actual_position[4],
                "%5.3f" % self.s.actual_position[5],
            ]
        )

    def get_cmd_joint(self):
        """Returns desired joint positions

        Returns:
            str: string of list of values
        """
        self.s.poll()
        return str(
            [
                "%5.3f" % self.s.joint_position[0],
                "%5.3f" % self.s.joint_position[1],
                "%5.3f" % self.s.joint_position[2],
                "%5.3f" % self.s.joint_position[3],
                "%5.3f" % self.s.joint_position[4],
                "%5.3f" % self.s.joint_position[5],
            ]
        )

    def get_actl_joint(self):
        """Returns actual joint positions.

        Returns:
            str: string of list of values
        """
        self.s.poll()
        return str(
            [
                "%5.3f" % self.s.joint_actual_position[0],
                "%5.3f" % self.s.joint_actual_position[1],
                "%5.3f" % self.s.joint_actual_position[2],
                "%5.3f" % self.s.joint_actual_position[3],
                "%5.3f" % self.s.joint_actual_position[4],
                "%5.3f" % self.s.joint_actual_position[5],
            ]
        )

    def get_cmd_pos_float(self):
        """Returns trajectory position.

        Returns:
            list[str]: list of strings of values
        """
        self.s.poll()
        return [
            "%5.3f" % self.s.position[0],
            "%5.3f" % self.s.position[1],
            "%5.3f" % self.s.position[2],
            "%5.3f" % self.s.position[3],
            "%5.3f" % self.s.position[4],
            "%5.3f" % self.s.position[5],
        ]

    def get_actl_pos_float(self):
        """Returns current trajectory position.

        Returns:
            list[str]: list of strings of values
        """
        self.s.poll()
        return [
            "%5.3f" % self.s.actual_position[0],
            "%5.3f" % self.s.actual_position[1],
            "%5.3f" % self.s.actual_position[2],
            "%5.3f" % self.s.actual_position[3],
            "%5.3f" % self.s.actual_position[4],
            "%5.3f" % self.s.actual_position[5],
        ]

    def get_cmd_joint_float(self):
        """Returns desired joint positions.

        Returns:
            list[str]: list of string of values
        """
        self.s.poll()
        return [
            "%5.3f" % self.s.joint_position[0],
            "%5.3f" % self.s.joint_position[1],
            "%5.3f" % self.s.joint_position[2],
            "%5.3f" % self.s.joint_position[3],
            "%5.3f" % self.s.joint_position[4],
            "%5.3f" % self.s.joint_position[5],
        ]

    def get_actl_joint_float(self):
        """Returns actual joint positions

        Returns:
            list[str]: list of strings of values
        """
        self.s.poll()
        return [
            "%5.3f" % self.s.joint_actual_position[0],
            "%5.3f" % self.s.joint_actual_position[1],
            "%5.3f" % self.s.joint_actual_position[2],
            "%5.3f" % self.s.joint_actual_position[3],
            "%5.3f" % self.s.joint_actual_position[4],
            "%5.3f" % self.s.joint_actual_position[5],
        ]

    ################################
    # get motion status
    ################################
    def get_current_velocity(self):
        """Returns current velocity.

        Returns:
            float: current velocity in units per second
        """
        self.s.poll()
        return self.s.current_vel

    def get_defalut_velocity(self):
        """Returns default velocity.

        Returns:
            float: default velocity
        """
        self.s.poll()
        return self.s.velocity

    def get_defalut_acceleration(self):
        """Returns default acceleration

        Returns:
            float: default acceleration
        """
        self.s.poll()
        return self.s.acceleration

    def get_max_velocity(self):
        """Returns maximum velocity.

        Returns:
            float: maximum velocity
        """
        self.s.poll()
        return self.s.max_velocity

    def get_max_acceleration(self):
        """Returns maximum acceleration.

        Returns:
            float: maximum acceleration
        """
        self.s.poll()
        return self.s.max_acceleration

    def get_feedrate(self):
        """Returns current feedrate override, 1.0 = 100%.

        Returns:
            float: current feedrate
        """
        self.s.poll()
        return self.s.feedrate

    def get_motion_enabled(self):
        """State of trajectory planner enabled flag.
        True if robot can move, false otherwise.
        轨迹规划器启用标志的状态。如果机器人可以移动，则为True，否则为False

        Returns:
            bool: enabled flag value
        """
        self.s.poll()
        return self.s.enabled

    def get_joint_vel_fb(self, _num):
        """_summary_

        Args:
            _num (_type_): _description_

        Raises:
            NotImplementedError: _description_
        """
        self.s.poll()
        ###return self.s.jvel[num]
        raise NotImplementedError

    def get_joint_torq_fb(self, _num):
        """_summary_

        Args:
            _num (_type_): _description_

        Raises:
            NotImplementedError: _description_
        """
        self.s.poll()
        ###return self.s.jtorq[num]
        raise NotImplementedError

    def get_error_type(self, _num):
        """_summary_

        Args:
            _num (_type_): _description_

        Raises:
            NotImplementedError: _description_
        """
        self.s.poll()
        ###return self.s.errinfo
        raise NotImplementedError

    ################################
    # set cmds
    ################################
    def set_joint_torque_limit(self, num, _value):
        if num < MAX_JOINTS:
            ###self.c.set_jtorq_limit(num,value)
            raise NotImplementedError

    def set_carte_torque_limit(self, axis, value):
        """Sets cartesian torque limit.

        Args:
            axis (Axis): axis to set torque for
            value (float): torque value
        """
        if axis == Axis.X or axis == Axis.Y:
            self.set_analog_out(AO.XY_AXIS_TORQUE, value)
        elif axis == Axis.Z:
            self.set_analog_out(AO.Z_AXIS_TORQUE, value)

    def set_joint_torque_en(self, _state):
        # self.c.set_jtorq_enable(0,state)
        raise NotImplementedError

    def set_carte_torque_en(self, _state):
        # self.c.set_cartetorq_enable(0,state)
        raise NotImplementedError

    def set_jog_mode(self, jog_mode):
        """Changes jog mode to jog_mode (angular or linear).
        AKA jog_state_check().

        Args:
            jog_mode (JogMode): linear (JogMode.JOG_TELEOP) or
                                angular (JogMode.JOG_JOINT) jog mode

        Returns:
            bool: True if change state was successful, False otherwise
        """
        self.s.poll()
        if (
            self.s.estop
            or not self.s.enabled
            or (self.s.interp_state != elerob.INTERP_IDLE)
        ):
            return False
        if self.s.task_mode != elerob.MODE_MANUAL:
            self.c.mode(elerob.MODE_MANUAL)
            self.c.wait_complete()
        if jog_mode:
            if self.s.motion_mode != elerob.TRAJ_MODE_FREE:
                self.c.teleop_enable(0)
                self.c.wait_complete()
        else:
            if self.s.motion_mode != elerob.TRAJ_MODE_TELEOP:
                self.c.teleop_enable(1)
                self.c.wait_complete()
        return True

    def continuous_jog(self, jogmode, axis, direction):
        """Start axis or joint jog.

        Args:
            jogmode (_type_): jog mode (axis or joint)
            axis (_type_): axis or joint
            direction (_type_): direction of movement

        Returns:
            bool: True if successfully started jog, False otherwise
        """
        if not self.set_jog_mode(jogmode):
            return False
        if direction == 0:
            self.c.jog(elerob.JOG_STOP, jogmode, axis)
        else:
            # if axis in (3,4,5):
            # 	rate = self.angular_jog_velocity
            # else:
            rate = jog_velocity
            self.c.jog(elerob.JOG_CONTINUOUS, jogmode, axis, direction * rate)
        return True

    def incremental_jog(self, jogmode, axis, direction, distance):
        """Move joint or axis by specified value.

        Args:
            jogmode (_type_): move by axis or joint
            axis (_type_): axis or joint
            direction (_type_): direction of movement
            distance (_type_): distance or angle to move for

        Returns:
            bool: True if jog started successfully, False otherwise
        """
        if not self.set_jog_mode(jogmode):
            return False
        if direction == 0:
            self.c.jog(elerob.JOG_STOP, jogmode, axis)
        else:
            # if axis in (3,4,5):
            # 	rate = self.angular_jog_velocity
            # else:
            rate = jog_velocity
            self.c.jog(elerob.JOG_INCREMENT, jogmode, axis, direction * rate, distance)
        return True

    def coords_to_gcode(self, coords):
        """Returns gcode string to move to given coords

        Args:
            coords (list[float | str]): list of coordinate values

        Returns:
            str: gcode string to move to given coords
        """
        return (
            "X"
            + str(coords[0])
            + "Y"
            + str(coords[1])
            + "Z"
            + str(coords[2])
            + "A"
            + str(coords[3])
            + "B"
            + str(coords[4])
            + "C"
            + str(coords[5])
        )

    def angles_to_gcode(self, angles):
        """Returns gcode string to move to given joint angles

        Args:
            coords (list[float | str]): list of joint angles values

        Returns:
            str: gcode string to move to given joint angles
        """
        return self.coords_to_gcode(angles)

    def float_equal(self, a, b, epsilon=COORDS_EPSILON):
        """_summary_

        Args:
            a (float): _description_
            b (float): _description_
            epsilon (float): _description_

        Returns:
            bool: _description_
        """
        return math.fabs(a - b) < epsilon

    def coords_equal(self, coords_1, coords_2):
        """Checks if specified coords are equal.

        Args:
            coords_1 (list): first coords to compare
            coords_2 (list): second coords to compare

        Returns:
            bool: True if coords are equal, False otherwise.
        """
        return (
            self.float_equal(coords_1[Axis.X], coords_2[Axis.X])
            and self.float_equal(coords_1[Axis.Y], coords_2[Axis.Y])
            and self.float_equal(coords_1[Axis.Z], coords_2[Axis.Z])
            and self.float_equal(coords_1[Axis.RX], coords_2[Axis.RX])
            and self.float_equal(coords_1[Axis.RY], coords_2[Axis.RY])
            and self.float_equal(coords_1[Axis.RZ], coords_2[Axis.RZ])
        )

    def angles_equal(self, angles_1, angles_2):
        """Checks if specified angles are equal.

        Args:
            angles_1 (list): first angles to compare
            angles_2 (list): second angles to compare

        Returns:
            bool: True if angles are equal, False otherwise.
        """
        return (
            self.float_equal(angles_1[Axis.X], angles_2[Axis.X])
            and self.float_equal(angles_1[Axis.Y], angles_2[Axis.Y])
            and self.float_equal(angles_1[Axis.Z], angles_2[Axis.Z])
            and self.float_equal(angles_1[Axis.RX], angles_2[Axis.RX])
            and self.float_equal(angles_1[Axis.RY], angles_2[Axis.RY])
            and self.float_equal(angles_1[Axis.RZ], angles_2[Axis.RZ])
        )
