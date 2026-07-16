# coding=utf-8
import time
from pymycobot.myagvplusapi import MyAGVPlusApi, _crc16_modbus, motor_api, WHEEL_RADIUS, WHEEL_BASE_SUM
from pymycobot.common import MyagvPlusCommand
from pymycobot.DM_CAN import Control_Type


class MyAGVPlus(MyAGVPlusApi):
    def __init__(self, motor_port='/dev/ttyACM0', baudrate=921600,
                 esp32_port='/dev/ttyCH341USB0', esp32_baud=115200, timeout=0.5,
                 debug=False, socket_server=False):
        """Initialize MyAGVPlus controller instance.

        Args:
            motor_port (str): Serial port path for DM motors.
            baudrate (int): Baud rate for motor serial. Default 921600.
            esp32_port (str): Serial port path for ESP32.
            esp32_baud (int): Baud rate for ESP32 serial. Default 115200.
            timeout (float): Serial port read timeout in seconds. Default 0.5.
            debug (bool): Enable verbose debug logging. Default False.
            socket_server (bool): Flag indicating if instance runs inside socket server. Default False.
        """
        super(MyAGVPlus, self).__init__(motor_port, baudrate, esp32_port,
                                       esp32_baud, timeout, debug, socket_server)

    def get_communication_state(self, *, from_cache=False, simulate=None):
        """Read currently active communication interface setting.

        Returns:
            int: 0 (Serial), 1 (Socket), or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__)
        if simulate is None:
            simulate = not getattr(self, 'socket_server', False)
        return super(MyAGVPlus, self).get_communication_state(from_cache=from_cache, simulate=simulate)

    # ============== System & Version ==============

    def get_system_version(self):
        """Get the main firmware version of the robot controller.

        Returns:
            float: version number (e.g. 1.2), or -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_SYSTEM_VERSION)

    def get_modify_version(self):
        """Get the sub-firmware modification version.

        Returns:
            int: modification version number, or -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_MODIFY_VERSION)

    # ============== Debug ==============

    def set_debug_state(self, state):
        """Set the debug level of the bottom control board.

        Args:
            state (int): Debug mode bitmask:
                0: Battery messages
                1: Gyroscope messages
                2: LED debug state
                4: Raw serial protocol commands

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, debug_state=state)
        return self._merge(MyagvPlusCommand.SET_DEBUG_STATE, state)

    def get_debug_state(self):
        """Get the current active debug state mask.

        Returns:
            int: debug mask state (0, 1, 2, or 4), or -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_DEBUG_STATE)

    # ============== Power ==============

    def power_on(self):
        """Power on the robot relays, enable all motors and set control mode to velocity control.

        Returns:
            int: 1 if power on succeeded, -1 if failed.
        """
        return self._power_on()

    def power_off(self):
        """Disable all wheel motors and power off the relay switch.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        return self._power_off()

    def is_power_on(self):
        """Check whether the bottom relay power is active.

        Returns:
            int: 1 if powered on, 0 if offline, -1 if failed.
        """
        return self._merge(MyagvPlusCommand.IS_POWER_ON)

    # ============== Status ==============

    def get_robot_status(self):
        """Read robot status bits.

        Returns:
            list[int]: [battery_state, gyro_state, power_level], or -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_ROBOT_STATUS)

    def get_all_msg(self):
        """Read real-time battery voltage and raw gyroscope data packets.

        Returns:
            list: status values list, or -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_ALL_MSG)

    # ============== Motion Control ==============

    def move_forward(self, speed):
        """Move the robot chassis forward.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._set_velocity(-float(speed), float(speed), -float(speed), float(speed))

    def move_backward(self, speed):
        """Move the robot chassis backward.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._set_velocity(float(speed), -float(speed), float(speed), -float(speed))

    def move_left_lateral(self, speed):
        """Move the robot chassis laterally to the left.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._set_velocity(float(speed), float(speed), -float(speed), -float(speed))

    def move_right_lateral(self, speed):
        """Move the robot chassis laterally to the right.

        Args:
            speed (float): 0.01 ~ 1.60 m/s.

        Returns:
            int: 1 if successful, -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
            
        return self._set_velocity(-float(speed), -float(speed), float(speed), float(speed))

    def turn_left(self, angular_speed):
        """Turn the robot chassis left (counter-clockwise rotation).

        Args:
            angular_speed (float): chassis angular velocity in rad/s, range 0.01 ~ 7.27.

        Returns:
            int: 1 if successful, -1 if failed.

        Raises:
            ValueError: If angular_speed is out of [0.01, 7.27] range or has >2 decimal places.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angular_speed=angular_speed)
        # omega = (w_motor * r) / (a+b)  =>  w_motor = omega * (a+b) / r
        w_motor = float(angular_speed) * WHEEL_BASE_SUM / WHEEL_RADIUS
        # Counter-clockwise: all four wheels same direction
        return self._set_motor_velocity_raw(w_motor, w_motor, w_motor, w_motor)

    def turn_right(self, angular_speed):
        """Turn the robot chassis right (clockwise rotation).

        Args:
            angular_speed (float): chassis angular velocity in rad/s, range 0.01 ~ 7.27.

        Returns:
            int: 1 if successful, -1 if failed.

        Raises:
            ValueError: If angular_speed is out of [0.01, 7.27] range or has >2 decimal places.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angular_speed=angular_speed)
        # omega = (w_motor * r) / (a+b)  =>  w_motor = omega * (a+b) / r
        w_motor = float(angular_speed) * WHEEL_BASE_SUM / WHEEL_RADIUS
        # Clockwise: all four wheels reverse direction
        return self._set_motor_velocity_raw(-w_motor, -w_motor, -w_motor, -w_motor)

    def stop(self):
        """Stop all motions immediately.

        Returns:
            int: 1 if dispatched, -1 if failed.
        """
        return self._set_velocity(0, 0, 0, 0)

    # ============== Auto Report ==============

    def set_auto_report_state(self, state):
        """Set ESP32 background auto report switch state.

        Args:
            state (int): 0 to disable, 1 to enable.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        resp = self._merge(MyagvPlusCommand.SET_AUTO_REPORT_STATE, state)
        if state == 0:
            # Clear cached auto-report data so stale values are not returned
            self._latest_auto_report = None
            self._auto_report_buffer = b""
            self._auto_report_enabled = False
        else:
            self._auto_report_enabled = True
        return resp

    def get_auto_report_state(self):
        """Get ESP32 auto report state.

        Returns:
            int: 1 if active, 0 if disabled, -1 if failed.
        """
        return self._merge(MyagvPlusCommand.GET_AUTO_REPORT_STATE)

    def get_auto_report_message(self):
        """Read the latest captured automatic reporting data frames.

        Returns:
            list: parsed auto-report data when active,
            None if no data has been received yet.
        """
        return self._read_auto_report()

    # ============== Motor Control ==============

    @motor_api
    def set_motor_enable(self, motor_id, state):
        """Enable or disable target wheel motor torque.

        Args:
            motor_id (int): 1~4, or 254 (all motors).
            state (int): 1 to enable, 0 to disable.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, motor_id=motor_id, state=state)

        target_motors = self.motors if motor_id == 254 else [self.motors[motor_id - 1]]

        if state == 0:
            self.stop()
            time.sleep(0.1)

        self.motor_driver.set_motors_state(target_motors, state)
        return 1

    @motor_api
    def get_motor_enable_status(self):
        """Read the enable state of all wheel motors.

        Returns:
            list[int]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] (1=enabled, 0=disabled), or -1 if not ready.
        """
        for m in self.motors:
            self.motor_driver.refresh_motor_status(m)
        status = [1 if m.isEnable else 0 for m in self.motors]
        if 0 in status and 1 in status:
            self.stop()
        return status

    @motor_api
    def get_motor_status(self):
        """Read error codes for all wheel motors.

        Returns:
            list[int]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] error codes (0=normal), or -1 if not ready.
        """
        stall, uv = getattr(self, '_stall_locked_motors', [False]*4), getattr(self, '_undervoltage_locked_motors', [False]*4)
        for i, m in enumerate(self.motors):
            if not uv[i] and not stall[i]: 
                self.motor_driver.refresh_motor_status(m)
        status = [9 if uv[i] else 10 if stall[i] else self.motors[i].getError() for i in range(4)]
        if any(status): self.stop()
        return status

    @motor_api
    def clear_motor_error(self, motor_id):
        """Clear error codes of target motor by restarting and re-enabling it.

        Args:
            motor_id (int): 1~4 or 254.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, motor_id=motor_id)
        target_motors = self.motors if motor_id == 254 else [self.motors[motor_id - 1]]

        self.motor_driver.set_motors_state(target_motors, 0)
        time.sleep(0.1)
        self.motor_driver.set_motors_state(target_motors, 1)
        for m in target_motors: self.motor_driver.refresh_motor_status(m)
            
        global_uv = False
        try:
            self._suppress_esp_log = True
            msg = self.get_all_msg()
            if msg != -1 and isinstance(msg, list) and len(msg) >= 6:
                vol1, vol2 = msg[4], msg[5]
                is_v1_dead, is_v1_ok = (0 < vol1 < 19.0), (vol1 >= 19.0)
                is_v2_dead, is_v2_ok = (0 < vol2 < 19.0), (vol2 >= 19.0)
                global_uv = not is_v1_ok and not is_v2_ok and (is_v1_dead or is_v2_dead)
        except Exception: pass
        finally: self._suppress_esp_log = False

        if global_uv:
            self.log.warning("clear_motor_error: undervoltage (< 19.0V), motion locked.")
        else:
            for i in (range(4) if motor_id == 254 else [motor_id - 1]):
                self._undervoltage_locked_motors[i] = False
                self._stall_locked_motors[i] = False
        if hasattr(self, '_save_error_state'): self._save_error_state()

    @motor_api
    def get_motor_temps(self):
        """Read temperature sensors on wheel motor MOS controllers.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] MOS temperatures, or -1 if not ready.
        """
        with self._dm_lock:
            return [self.motor_driver.get_mos_temp(m) for m in self.motors]

    @motor_api
    def get_motor_param_cache(self, motor_id, rid):
        """Read parameter value cache stored inside DM motor model structure.

        Args:
            motor_id (int): 1~4.
            rid (int): Parameter ID.

        Returns:
            float/int: cached parameter value.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, rid=rid)
        return self.motors[motor_id - 1].getParam(rid)


    @motor_api
    def get_motor_velocity(self, motor_id):
        """Read real-time angular velocity of target motor.

        Args:
            motor_id (int): 1~4.

        Returns:
            float: angular speed (rad/s), or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        with self._dm_lock:
            return self.motor_driver.get_filtered_velocity(self.motors[motor_id - 1])

    @motor_api
    def get_motor_torque(self, motor_id):
        """Read real-time feedback torque of target motor.

        Args:
            motor_id (int): 1~4.

        Returns:
            float: torque value (Nm), or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        with self._dm_lock:
            return self.motor_driver.get_filtered_torque(self.motors[motor_id - 1])

    @motor_api
    def get_motor_move_speeds(self):
        """Read real-time linear speed translation of all wheels.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] linear speeds (m/s), or -1 if not ready.
        """
        speeds = []
        with self._dm_lock:
            for i, m in enumerate(self.motors):
                self.motor_driver.refresh_motor_status(m)
                w = m.getVelocity()
                v = (w * WHEEL_RADIUS)
                # Normalize so positive is forward for all wheels
                if i == 0 or i == 2:  # Left wheels (rl_wheel_joint, fr_wheel_joint)
                    v = -v
                if abs(v) < 0.05:
                    v = 0.0
                speeds.append(round(float(v), 2))
        return speeds

    @motor_api
    def get_motor_turn_speeds(self):
        """Read real-time angular speed translation (rotation) of all wheels.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] angular speeds (rad/s), or -1 if not ready.
        """
        speeds = []
        with self._dm_lock:
            for i, m in enumerate(self.motors):
                self.motor_driver.refresh_motor_status(m)
                w = m.getVelocity()
                # omega = (w_motor * r) / (a+b)
                omega = (w * WHEEL_RADIUS) / WHEEL_BASE_SUM
                # Normalize direction: left wheels positive w_motor means moving backward, right wheels positive w_motor means moving forward
                # For counter-clockwise rotation (turn_left), w_motor is positive for all wheels
                if abs(omega) < 0.05:
                    omega = 0.0
                speeds.append(round(float(omega), 2))
        return speeds

    @motor_api
    def get_motor_torques(self):
        """Read feedback torque values of all motors.

        Returns:
            list[float]: [rl_wheel_joint, fl_wheel_joint, fr_wheel_joint, rr_wheel_joint] torques (Nm), or -1 if not ready.
        """
        with self._dm_lock:
            return [self.motor_driver.get_filtered_torque(m) for m in self.motors]




    @motor_api
    def change_motor_param(self, motor_id, rid, value):
        """Change configuration register values inside motor controller.

        Args:
            motor_id (int): 1~4.
            rid (int): Register/Parameter index.
            value (int/float): Target value.

        Returns:
            int: result from motor_driver, or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, rid=rid, value=value)
        return self.motor_driver.change_motor_param(self.motors[motor_id - 1], rid, value)

    @motor_api
    def read_motor_param(self, motor_id, rid):
        """Read configuration parameter from motor controller flash registers.

        Args:
            motor_id (int): 1~4.
            rid (int): Parameter ID.

        Returns:
            float/int: parameter value, or -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, rid=rid)
        return self.motor_driver.read_motor_param(self.motors[motor_id - 1], rid)

    @motor_api
    def save_motor_param(self, motor_id):
        """Save active parameters to motor flash memory.

        Args:
            motor_id (int): 1~4.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        self.motor_driver.save_motor_param(self.motors[motor_id - 1])

    @motor_api
    def switch_motor_control_mode(self, motor_id, control_mode):
        """Switch operational control mode in DM motor module.

        Args:
            motor_id (int): 1~4.
            control_mode (int): Target mode code.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, control_mode=control_mode)
        self.motor_driver.switchControlMode(self.motors[motor_id - 1], control_mode)

    @motor_api
    def change_motor_limit_param(self, motor_type, pmax, vmax, tmax):
        """Change maximum constraints inside motor limits matrix.

        Args:
            motor_type (int): Motor type enum code.
            pmax (float): Position limit.
            vmax (float): Velocity limit.
            tmax (float): Torque limit.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, motor_type=motor_type, pmax=pmax, vmax=vmax, tmax=tmax)
        self.motor_driver.change_limit_param(motor_type, pmax, vmax, tmax)

    @motor_api
    def refresh_motor_status_by_id(self, motor_id):
        """Refresh raw registers cache by polling target motor module.

        Args:
            motor_id (int): 1~4.

        Returns:
            int: 1 if successful, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id)
        self.motor_driver.refresh_motor_status(self.motors[motor_id - 1])

    @motor_api
    def control_motor_mit(self, motor_id, kp, kd, q, dq, tau):
        """Apply MIT direct mode parameters to target motor.

        Args:
            motor_id (int): 1~4.
            kp, kd, q, dq, tau (float): MIT controller arguments.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, kp=kp, kd=kd, q=q, dq=dq, tau=tau)
        self.motor_driver.controlMIT(self.motors[motor_id - 1], kp, kd, q, dq, tau)

    @motor_api
    def control_motor_delay(self, motor_id, kp, kd, q, dq, tau, delay):
        """Apply MIT direct mode parameters with delay sleep duration.

        Args:
            motor_id (int): 1~4.
            kp, kd, q, dq, tau, delay (float): Controller arguments and delay.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, kp=kp, kd=kd, q=q, dq=dq, tau=tau, delay=delay)
        self.motor_driver.control_delay(self.motors[motor_id - 1], kp, kd, q, dq, tau, delay)

    @motor_api
    def control_motor_pos_vel(self, motor_id, p_desired, v_desired):
        """Position and velocity control command for target motor.

        Args:
            motor_id (int): 1~4.
            p_desired (float): Position target.
            v_desired (float): Velocity target.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, p_desired=p_desired, v_desired=v_desired)
        v_desired = 0 if abs(v_desired) < 0.1 else v_desired
        self.motor_driver.control_Pos_Vel(self.motors[motor_id - 1], p_desired, v_desired)

    @motor_api
    def control_motor_vel(self, motor_id, v_desired):
        """Velocity control command for target motor.

        Args:
            motor_id (int): 1~4.
            v_desired (float): Velocity target.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, v_desired=v_desired)
        v_desired = 0 if abs(v_desired) < 0.1 else v_desired
        self.motor_driver.control_Vel(self.motors[motor_id - 1], v_desired)

    @motor_api
    def control_motor_pos_force(self, motor_id, pos_des, vel_des, i_des):
        """Position and force control command for target motor.

        Args:
            motor_id (int): 1~4.
            pos_des, vel_des, i_des (float): Position, velocity and current targets.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, pos_des=pos_des, vel_des=vel_des, i_des=i_des)
        vel_des = 0 if abs(vel_des) < 0.1 else vel_des
        self.motor_driver.control_pos_force(self.motors[motor_id - 1], pos_des, vel_des, i_des)

    @motor_api
    def control_motor_pos_vel_csp(self, motor_id, p_desired, v_desired):
        """Position and velocity CSP mode command for target motor.

        Args:
            motor_id (int): 1~4.
            p_desired, v_desired (float): Position and velocity targets.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, p_desired=p_desired, v_desired=v_desired)
        v_desired = 0 if abs(v_desired) < 0.1 else v_desired
        self.motor_driver.control_Pos_Vel_CSP(self.motors[motor_id - 1], p_desired, v_desired)

    @motor_api
    def control_motor_vel_csp(self, motor_id, v_desired):
        """Velocity CSP mode command for target motor.

        Args:
            motor_id (int): 1~4.
            v_desired (float): Velocity target.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, v_desired=v_desired)
        v_desired = 0 if abs(v_desired) < 0.1 else v_desired
        self.motor_driver.control_Vel_CSP(self.motors[motor_id - 1], v_desired)

    @motor_api
    def control_motor_tor_csp(self, motor_id, tor_desired):
        """Torque CSP mode command for target motor.

        Args:
            motor_id (int): 1~4.
            tor_desired (float): Torque target.

        Returns:
            int: 1 if dispatched, -1 if not ready.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, single_motor_id=motor_id, tor_desired=tor_desired)
        self.motor_driver.control_Tor_CSP(self.motors[motor_id - 1], tor_desired)

    # ============== LED ==============

    def set_led_color(self, brightness, color):
        """Set DIY LED strip brightness and RGB color values.

        Args:
            brightness (int): 0 ~ 255.
            color (tuple/list): [R, G, B] each 0 ~ 255.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, brightness=brightness, color=color)
        r, g, b = color
        return self._merge(MyagvPlusCommand.SET_LED_COLOR, brightness, r, g, b)

    def set_led_mode(self, mode):
        """Set active LED display mode.

        Args:
            mode (int): 0 for battery level display, 1 for DIY strip control.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._merge(MyagvPlusCommand.SET_LED_MODE, mode)

    # ============== IO ==============

    def get_pin_input(self, pin):
        """Read voltage input levels from target IO pin ports.

        Args:
            pin (int): 0~6. 0 reads all 6 input pins.

        Returns:
            list[int]/int: pin states (255 mapped to -1), or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin=pin)
        return self._merge(MyagvPlusCommand.GET_PIN_INPUT, pin)

    def set_pin_output(self, pin, state=0):
        """Write voltage output levels to target IO pin ports.

        Args:
            pin (int): 0~6.
            state (int): 0 for Low, 1 for High.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pin=pin, state=state)
        return self._merge(MyagvPlusCommand.SET_PIN_OUTPUT, pin, state)

    # ============== Fan ==============

    def set_fan_state(self, state=1):
        """Turn on or off internal chassis cooling fans.

        Args:
            state (int): 0 for Off, 1 for On.

        Returns:
            bytes: response data, or -1 if failed.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._merge(MyagvPlusCommand.SET_FAN_STATE, state)
