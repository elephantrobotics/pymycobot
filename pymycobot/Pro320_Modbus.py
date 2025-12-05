import threading
import time

from pymycobot.mycobot320 import MyCobot320


class ModbusCommandAddress:
    """Modbus Instruction Address Constants"""

    # System Control Instructions
    POWER_ON = 0x0010  # Power on robot
    POWER_OFF = 0x0011  # Power off robot
    CLEAR_MOTION_ERROR = 0x0008  # Clear motion errors
    RETURN_TO_ZERO = 0x0004  # Return to zero

    # Read status register
    SYSTEM_VERSION = 0x0002  # System version
    SET_FRESH_MODE = 0x0016  # Set operating mode
    FRESH_MODE = 0x0017  # Operating mode
    POWER_STATUS = 0x0012  # Power-on status
    ERROR_INFORMATION = 0x0007  # Motion alarm
    ROBOT_STATUS = 0x00A2  # Motion status
    ANGLES = 0x0020  # Full Joint Angle
    COORDS = 0x0023  # Coordinate System Coordinates
    MOTOR_PAUSE = 0x0027  # Motion Pause Status
    IS_MOVING = 0x002B  # Motion Status
    DIGITAL_IO_INPUT = 0x00A1  # End-effector Input Status
    BASE_IO_INPUT = 0x007B  # Base Input Status
    SERVOS_SPEED = 0x00E1  # Servo Speed
    SERVOS_CURRENT = 0x00E2  # Servo Current
    SERVOS_STATUS = 0x00E4  # Servo Status

    # Motion Control Commands
    SINGLE_ANGLE = 0x0021  # Single Joint Control
    MULTIPLE_ANGLES = 0x0022  # Multi-Joint Control
    SINGLE_COORD = 0x0024  # Single Coordinate Control
    MULTIPLE_COORDS = 0x0025  # Multi-axis control
    PAUSE_MOTION = 0x0026  # Pause motion
    RESUME_MOTION = 0x0028  # Continue motion
    STOP_MOTION = 0x0029  # Stop motion

    # Continuous motion command
    CONTINUOUS_JOINT = 0x0030  # Continuous joint motion
    CONTINUOUS_COORD = 0x0032  # Continuous coordinate motion
    RPY_ROTATION = 0x00F5  # RPY rotation

    # Stepping motion command
    STEP_ANGLE = 0x0033  # Angle step
    STEP_COORD = 0x0034  # Coordinate step

    # Joint control command
    MOTOR_ENABLE = 0x0013  # Joint enabled state
    MOTOR_DISABLE = 0x0014  # Joint disabled state
    BRAKE_CONTROL = 0x0019  # Joint Brake Status

    # IO Control Commands
    BASE_IO_OUTPUT = 0x00A0  # Bottom IO Control
    DIGITAL_IO_OUTPUT = 0x0061  # End IO Control
    RGB_COLOR = 0x000C  # RGB Color Control


class Pro320Modbus:
    def __init__(self, port, baudrate=115200, slave=0x2D, debug=False):
        self.driver = MyCobot320(port, baudrate, debug=False)
        self.slave = slave
        self._lock = threading.RLock()
        self.debug = debug

    # CRC16(MODBUS)
    def crc16(self, data: bytes):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x01:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    # Constructing Modbus read frames
    def _build_read_frame(self, reg_addr, reg_len=1):
        addr_h = (reg_addr >> 8) & 0xFF
        addr_l = reg_addr & 0xFF
        len_h = (reg_len >> 8) & 0xFF
        len_l = reg_len & 0xFF

        payload = bytes([self.slave, 0x03, addr_h, addr_l, len_h, len_l])
        crc = self.crc16(payload)
        frame = payload + crc
        self._log("Modbus write", frame)
        return frame

    def _log(self, prefix, frame):
        if self.debug:
            ts = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() * 1000) % 1000:03d}"
            frame_str = ' '.join(f'{b:02X}' for b in frame)
            print(f"{ts} {prefix}: {frame_str}")

    # Constructing Modbus write frames
    def _build_write_frame(self, reg_addr, values):
        addr_h = (reg_addr >> 8) & 0xFF
        addr_l = reg_addr & 0xFF
        reg_count = len(values)
        num_h = (reg_count >> 8) & 0xFF
        num_l = reg_count & 0xFF
        byte_count = reg_count * 2

        data_bytes = bytearray()
        for value in values:
            data_bytes.extend([(value >> 8) & 0xFF, value & 0xFF])

        payload = bytes([self.slave, 0x10, addr_h, addr_l, num_h, num_l, byte_count]) + bytes(data_bytes)
        crc = self.crc16(payload)
        frame = payload + crc
        self._log("Modbus write", frame)
        return frame

    def _encode_status(self, status_list):
        bitmask = 0
        for code in status_list:
            bitmask |= (1 << code)
        return bitmask

    # Read the interface (print Modbus frames + invoke the original protocol)
    def get_angles(self):
        self._build_read_frame(ModbusCommandAddress.ANGLES, 1)

        angles = self.driver.get_angles()
        if not isinstance(angles, list) or len(angles) != 6:
            self._log("ERROR", b"Read angles failed")
            return -1
        # 模拟返回 Modbus 格式帧（6寄存器，每寄存器2字节）包含CRC
        modbus_data = bytearray()
        for a in angles:
            val = int(a * 100)
            if val < 0:
                val += 65536
            modbus_data.extend([(val >> 8) & 0xFF, val & 0xFF])
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return angles

    def get_coords(self):
        self._build_read_frame(ModbusCommandAddress.COORDS, 1)  # 1寄存器

        coords = self.driver.get_coords()
        if not isinstance(coords, list) or len(coords) != 6:
            self._log("ERROR", b"Read coords failed")
            return -1

        # 模拟返回 Modbus 格式帧（6寄存器，每寄存器2字节）包含CRC
        modbus_data = bytearray()
        for i, c in enumerate(coords):
            if i < 3:  # x, y, z
                val = int(c * 10)
            else:  # rx, ry, rz
                val = int(c * 100)
            if val < 0:
                val += 65536
            modbus_data.extend([(val >> 8) & 0xFF, val & 0xFF])

        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)

        return coords

    def is_moving(self):
        self._build_read_frame(ModbusCommandAddress.IS_MOVING, 1)
        status = self.driver.is_moving()
        if not isinstance(status, int) or status == -1:
            return -1
        modbus_data = bytearray()
        modbus_data.extend([(status >> 8) & 0xFF, status & 0xFF])
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return status

    def get_robot_status(self):
        self._build_read_frame(ModbusCommandAddress.ROBOT_STATUS, 1)
        robot_status = self.driver.get_robot_status()
        if not isinstance(robot_status, list) or len(robot_status) < 6:
            self._log("ERROR", b"Read robot status failed")
            return -1

        modbus_data = bytearray()
        for status in robot_status:
            if isinstance(status, list):
                value = self._encode_status(status)
            else:
                value = status

            # Convert to unsigned 16bit
            if value < 0:
                value += 65536

            modbus_data.extend([(value >> 8) & 0xFF, value & 0xFF])

        # payload + crc
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc

        self._log("Modbus read", resp_frame)
        return resp_frame

    def get_fresh_mode(self):
        self._build_read_frame(ModbusCommandAddress.FRESH_MODE, 1)

        fresh_mode = self.driver.get_fresh_mode()
        if not isinstance(fresh_mode, int) or fresh_mode == -1:
            return -1
        modbus_data = bytearray()
        modbus_data.extend([(fresh_mode >> 8) & 0xFF, fresh_mode & 0xFF])
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return fresh_mode

    def is_power_on(self):
        self._build_read_frame(ModbusCommandAddress.POWER_STATUS, 1)
        is_power = self.driver.is_power_on()
        if not isinstance(is_power, int) or is_power == -1:
            return -1
        modbus_data = bytearray()
        modbus_data.extend([(is_power >> 8) & 0xFF, is_power & 0xFF])
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return is_power

    def get_error_information(self):
        self._build_read_frame(ModbusCommandAddress.ERROR_INFORMATION, 1)
        error_info = self.driver.get_error_information()
        if not isinstance(error_info, int) or error_info == -1:
            return -1
        modbus_data = bytearray()
        modbus_data.extend([(error_info >> 8) & 0xFF, error_info & 0xFF])
        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return error_info

    def get_digital_input(self, pin_no):
        self._build_read_frame(ModbusCommandAddress.DIGITAL_IO_INPUT, 1)

        value = self.driver.get_digital_input(pin_no)
        if value == -1:
            self._log("ERROR", b"digital input read fail")
            return -1

        # 转换为 Modbus 响应帧
        payload = bytes([self.slave, 0x03, 0x02, (value >> 8) & 0xFF, value & 0xFF])
        crc = self.crc16(payload)
        resp = payload + crc
        self._log("Modbus read", resp)
        return value

    def get_basic_input(self, pin_no):
        self._build_read_frame(ModbusCommandAddress.BASE_IO_INPUT, 1)

        value = self.driver.get_basic_input(pin_no)
        if value == -1:
            self._log("ERROR", b"basic input read fail")
            return -1

        # 转换为 Modbus 响应帧
        payload = bytes([self.slave, 0x03, 0x02, (value >> 8) & 0xFF, value & 0xFF])
        crc = self.crc16(payload)
        resp = payload + crc
        self._log("Modbus read", resp)
        return value

    def get_servos_speed(self):
        self._build_read_frame(ModbusCommandAddress.SERVOS_SPEED, 1)
        servos_speed = self.driver.get_servo_speeds()
        if not isinstance(servos_speed, list) or servos_speed == -1:
            return -1
        modbus_data = bytearray()
        for i in servos_speed:
            modbus_data.extend([(i >> 8) & 0xFF, i & 0xFF])

        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)

        return servos_speed

    def get_servos_current(self):
        self._build_read_frame(ModbusCommandAddress.SERVOS_CURRENT, 1)
        servo_currents = self.driver.get_servo_currents()

        if not isinstance(servo_currents, list) or servo_currents == -1:
            return -1
        modbus_data = bytearray()
        for i in servo_currents:
            modbus_data.extend([(i >> 8) & 0xFF, i & 0xFF])

        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return servo_currents

    def get_servos_status(self):
        self._build_read_frame(ModbusCommandAddress.SERVOS_STATUS, 1)

        servos_status = self.driver.get_servo_status()
        if not isinstance(servos_status, list) or servos_status == -1:
            return -1
        modbus_data = bytearray()
        for status in servos_status:
            if isinstance(status, list):
                value = self._encode_status(status)
            else:
                value = status

            # Convert to unsigned 16bit
            if value < 0:
                value += 65536

            modbus_data.extend([(value >> 8) & 0xFF, value & 0xFF])

        resp_payload = bytes([self.slave, 0x03, len(modbus_data)]) + modbus_data
        resp_crc = self.crc16(resp_payload)
        resp_frame = resp_payload + resp_crc
        self._log("Modbus read", resp_frame)
        return servos_status

    # Write to the interface (print Modbus frames + call the original protocol)
    def send_angle(self, joint, angle, speed):
        angle_int = int(angle * 100)
        if angle_int < 0:
            angle_int += 65536
        values = [joint, angle_int, speed]
        self._build_write_frame(ModbusCommandAddress.SINGLE_ANGLE, values)
        return self.driver.send_angle(joint, angle, speed)

    def send_angles(self, angles, speed):
        angle_ints = []
        for a in angles:
            val = int(a * 100)
            if val < 0:
                val += 65536
            angle_ints.append(val)
        values = angle_ints + [speed]
        self._build_write_frame(ModbusCommandAddress.MULTIPLE_ANGLES, values)
        return self.driver.send_angles(angles, speed)

    def send_coord(self, coord_id, coord, speed):
        if coord_id < 3:  # XYZ
            val = int(coord * 10)
        else:  # Rx,Ry,Rz
            val = int(coord * 100)
        if val < 0:
            val += 65536
        values = [coord_id, val, speed]
        self._build_write_frame(ModbusCommandAddress.SINGLE_COORD, values)
        return self.driver.send_coord(coord_id, coord, speed)

    def send_coords(self, coords, speed):
        coord_ints = []
        for i, c in enumerate(coords):
            if i < 3:
                val = int(c * 10)
            else:
                val = int(c * 100)
            if val < 0:
                val += 65536
            coord_ints.append(val)
        values = coord_ints + [speed]
        self._build_write_frame(ModbusCommandAddress.MULTIPLE_COORDS, values)
        return self.driver.send_coords(coords, speed)

    def pause(self):
        self._build_write_frame(ModbusCommandAddress.PAUSE_MOTION, [0x0000])
        return self.driver.pause()

    def resume(self):
        self._build_write_frame(ModbusCommandAddress.RESUME_MOTION, [0x0000])
        return self.driver.resume()

    def stop(self):
        self._build_write_frame(ModbusCommandAddress.STOP_MOTION, [0x0000])
        return self.driver.stop()

    def jog_angle(self, joint_id, direction, speed):
        values = [joint_id, direction, speed]
        self._build_write_frame(ModbusCommandAddress.CONTINUOUS_JOINT, values)
        return self.driver.jog_angle(joint_id, direction, speed)

    def jog_coord(self, coord_id, direction, speed):
        values = [coord_id, direction, speed]
        self._build_write_frame(ModbusCommandAddress.CONTINUOUS_COORD, values)
        return self.driver.jog_coord(coord_id, direction, speed)

    def jog_rpy(self, end_direction, direction, speed):
        values = [end_direction, direction, speed]
        self._build_write_frame(ModbusCommandAddress.RPY_ROTATION, values)
        return self.driver.jog_rpy(end_direction, direction, speed)

    def jog_increment(self, joint_id, increment, speed):
        values = [joint_id, increment * 100, speed]
        self._build_write_frame(ModbusCommandAddress.STEP_ANGLE, values)
        return self.driver.jog_increment(joint_id, increment, speed)

    def jog_increment_coord(self, coord_id, increment, speed):
        values = [coord_id, increment, speed]
        self._build_write_frame(ModbusCommandAddress.STEP_COORD, values)
        return self.driver.jog_increment_coord(coord_id, increment, speed)

    def set_fresh_mode(self, mode):
        self._build_write_frame(ModbusCommandAddress.SET_FRESH_MODE, [mode])
        return self.driver.set_fresh_mode(mode)

    def power_on(self):
        self._build_write_frame(ModbusCommandAddress.POWER_ON, [0x0000])
        return self.driver.power_on()

    def power_off(self):
        self._build_write_frame(ModbusCommandAddress.POWER_OFF, [0x0000])
        return self.driver.power_off()

    def go_home(self):
        return self.send_angles([0, 0, 0, 0, 0, 0], 20)

    def clear_error_information(self):
        self._build_write_frame(ModbusCommandAddress.CLEAR_MOTION_ERROR, [1])
        return self.driver.clear_error_information()

    def release_all_servos(self):
        self._build_write_frame(ModbusCommandAddress.MOTOR_DISABLE, [0x0000])
        return self.driver.release_all_servos()

    def release_servo(self, joint_id):
        self._build_write_frame(ModbusCommandAddress.MOTOR_DISABLE, [joint_id])
        return self.driver.release_servo(joint_id)

    def focus_all_servos(self):
        self._build_write_frame(ModbusCommandAddress.MOTOR_ENABLE, [0x0000])
        return self.driver.focus_all_servos()

    def focus_servo(self, joint_id):
        self._build_write_frame(ModbusCommandAddress.MOTOR_ENABLE, [joint_id])
        return self.driver.focus_servo(joint_id)

    def set_digital_output(self, pin, value):
        self._build_write_frame(ModbusCommandAddress.DIGITAL_IO_OUTPUT, [pin, value])
        return self.driver.set_digital_output(pin, value)

    def set_basic_output(self, pin, value):
        self._build_write_frame(ModbusCommandAddress.BASE_IO_OUTPUT, [pin, value])
        return self.driver.set_basic_output(pin, value)

    def set_color(self, r, g, b):
        self._build_write_frame(ModbusCommandAddress.RGB_COLOR, [r, g, b])
        return self.driver.set_color(r, g, b)

    def process_modbus_command(self, frame: list[int]):
        # frame example:
        # [slave, func, regH, regL, lenH, lenL, data..., crcL, crcH]

        slave = frame[0]
        func = frame[1]
        reg_addr = (frame[2] << 8) | frame[3]

        # delete CRC
        data = frame[7:-2] if func == 0x10 else frame[4:-2]

        # function code
        if func == 0x03:  # read
            return self._handle_read(reg_addr)

        elif func == 0x10:  # write
            return self._handle_write(reg_addr, data)

        else:
            return None

    def _handle_read(self, reg):
        if reg == ModbusCommandAddress.FRESH_MODE:
            return self.get_fresh_mode()

        elif reg == ModbusCommandAddress.ANGLES:
            return self.get_angles()

        elif reg == ModbusCommandAddress.COORDS:
            return self.get_coords()

        elif reg == ModbusCommandAddress.IS_MOVING:
            return self.is_moving()

        elif reg == ModbusCommandAddress.POWER_ON:
            return self.power_on()
        elif reg == ModbusCommandAddress.POWER_OFF:
            return self.power_off()
        elif reg == ModbusCommandAddress.ROBOT_STATUS:
            return self.get_robot_status()
        elif reg == ModbusCommandAddress.SERVOS_STATUS:
            return self.get_servos_status()
        elif reg == ModbusCommandAddress.SERVOS_CURRENT:
            return self.get_servos_current()

        elif reg == ModbusCommandAddress.ERROR_INFORMATION:
            return self.get_error_information()

        elif reg >= ModbusCommandAddress.DIGITAL_IO_INPUT:
            pin_no = reg - ModbusCommandAddress.DIGITAL_IO_INPUT
            return self.get_digital_input(pin_no)

        else:
            self._log("ERROR", b"read unsupported register")
            return None

    def _handle_write(self, reg, data):
        if reg == ModbusCommandAddress.SET_FRESH_MODE:
            mode = data[-1]  # 最后1字节
            return self.set_fresh_mode(mode)

        elif reg == ModbusCommandAddress.STEP_ANGLE:
            joint = (data[0] << 8) | data[1]
            increment = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.jog_increment(joint, increment, speed)

        elif reg == ModbusCommandAddress.STEP_COORD:
            coord_id = (data[0] << 8) | data[1]
            increment = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.jog_increment_coord(coord_id, increment, speed)

        elif reg == ModbusCommandAddress.CONTINUOUS_COORD:
            coord_id = (data[0] << 8) | data[1]
            direction = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.jog_coord(coord_id, direction, speed)

        elif reg == ModbusCommandAddress.RPY_ROTATION:
            end_direction = (data[0] << 8) | data[1]
            direction = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.jog_rpy(end_direction, direction, speed)

        elif reg == ModbusCommandAddress.CONTINUOUS_JOINT:
            joint = (data[0] << 8) | data[1]
            direction = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.jog_angle(joint, direction, speed)

        elif reg == ModbusCommandAddress.SINGLE_ANGLE:
            joint = (data[0] << 8) | data[1]
            angle = (data[2] << 8) | data[3]
            speed = (data[4] << 8) | data[5]
            return self.send_angle(joint, angle, speed)

        elif reg == ModbusCommandAddress.SINGLE_COORD:
            coord_id = (data[0] << 8) | data[1]
            coord = ((data[2] << 8) | data[3]) / 100
            speed = (data[4] << 8) | data[5]
            return self.send_coord(coord_id, coord, speed)

        elif reg == ModbusCommandAddress.MULTIPLE_ANGLES:
            # data = [angle1H, angle1L, angle2H, angle2L, ... , speedH, speedL]
            angles = []
            for i in range(0, len(data) - 2, 2):
                raw = (data[i] << 8) | data[i + 1]
                if raw > 32767:
                    raw -= 65536
                angles.append(raw / 100.0)

            speed = (data[-2] << 8) | data[-1]
            return self.send_angles(angles, speed)

        elif reg == ModbusCommandAddress.MULTIPLE_COORDS:
            coords = []
            for i in range(1, len(data) - 2, 2):
                raw = (data[i] << 8) | data[i + 1]
                if raw > 32767:
                    raw -= 65536
                # XYZ *10 / Rx Ry Rz *100
                coords.append(raw / 10.0 if i < 6 else raw / 100.0)

            speed = (data[-2] << 8) | data[-1]
            return self.send_coords(coords, speed)
        elif reg == ModbusCommandAddress.RGB_COLOR:
            r = data[1]
            g = data[3]
            b = data[5]
            return self.set_color(r, g, b)

        else:
            self._log("ERROR", b"write unsupported register")
            return None
