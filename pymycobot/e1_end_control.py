# coding=utf-8
import time

from pymycobot.common import ProtocolCode, ProGripper, MyHandGripper


class EndControlBase(object):
    _serial_port = None
    def calibration_parameters(self, *args, **kwargs):
        pass

    def _mesg(self, *args, **kwargs):
        pass

    def tool_serial_write_data(self, command):
        pass

    def _modbus_crc(self, data: bytes, mode='little') -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder=mode)

    def _clear_serial_input_buffer(self):
        serial_port = getattr(self, "_serial_port", None)
        if serial_port is None:
            return
        try:
            serial_port.reset_input_buffer()
        except Exception:
            pass

    def _check_gripper_id(self, gripper_id):

        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id)


class ForceGripper(EndControlBase):
    def _send_modbus_command(self, gripper_id, func_code, reg_addr, value_high=None, value_low=None, custom_mode=False):
        """
        General Modbus command sending method

        Args:
            gripper_id: Device ID
            func_code: Function code (0x03 = read, 0x06 = write)
            reg_addr: Register address
            value_high: High byte of the write data (can be None for read operations)
            value_low: Low byte of the write data (can be None for read operations)
        """
        # Base payload part shared by both modes
        payload = [gripper_id, func_code,
                   (reg_addr >> 8) & 0xFF, reg_addr & 0xFF]

        # Append value for write, or 0x00 0x00 for read
        if func_code == 0x06:
            payload.extend([value_high, value_low])
        else:
            payload.extend([0x00, 0x00])

        # Build full command based on mode
        if not custom_mode:
            # Modbus RTU classic format
            cmd = payload.copy()
            cmd.extend(self._modbus_crc(cmd))  # little-endian
        else:
            # Custom packet: FE FE LEN + payload + CRC(big-end)
            # LEN = payload length + CRC length (2)锛屽嵆 6+2 = 8
            cmd = [0xFE, 0xFE, 0x08] + payload
            cmd.extend(self._modbus_crc(cmd, mode='big'))
        self._clear_serial_input_buffer()
        recv = self.tool_serial_write_data(cmd)
        if not recv:
            return cmd, -1
        return cmd, recv

    def _write_and_check(self, gripper_id, reg_addr, value, custom_mode=False):
        """Write register and verify response robustly (support calibration delay)"""
        self._check_gripper_id(gripper_id)
        high, low = (value >> 8) & 0xFF, value & 0xFF
        # Continuously read the response packets, and send a read command to trigger feedback each time.
        _, recv = self._send_modbus_command(gripper_id, 0x06, reg_addr, high, low, custom_mode)

        # Basic validity check
        min_len = 9 if custom_mode else 8

        if not isinstance(recv, (list, bytearray)) or len(recv) < min_len:
            return -1

        # Two modes have different byte offsets
        # Modbus RTU standard: [id][cmd][regH][regL][valH][valL]...
        # Custom packet       : [fe][fe][len][id][cmd][regH][regL][valH][valL]...
        offset = 0 if not custom_mode else 3

        cmd_idx = 1 + offset  # command index
        reg_h_idx = 2 + offset  # register high
        reg_l_idx = 3 + offset  # register low
        val_h_idx = 4 + offset  # value high
        val_l_idx = 5 + offset  # value low

        # Verify command
        if recv[cmd_idx] != 0x06:
            return -1

        # Verify register address consistency
        if recv[reg_h_idx] != (reg_addr >> 8) & 0xFF or recv[reg_l_idx] != (reg_addr & 0xFF):
            return -1

        # Determine return status
        if recv[val_h_idx] == 0x00 and recv[val_l_idx] == 0x01:
            return 1

        return -1

    def _read_register(self, gripper_id, reg_addr):
        """Reads a register with command verification"""
        self._check_gripper_id(gripper_id)

        cmd, recv = self._send_modbus_command(gripper_id, 0x03, reg_addr)

        if isinstance(recv, (list, bytearray)) and len(recv) >= 6:
            recv_func = recv[1]
            recv_addr = (recv[2] << 8) | recv[3]
            if recv_func == 0x03 and recv_addr == reg_addr:
                return (recv[4] << 8) | recv[5]
            else:
                return -1


    def get_pro_gripper_firmware_version(self, gripper_id=14):
        """ Read the firmware major and minor version numbers

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (float): x.x
        """
        val = self._read_register(gripper_id, ProGripper.MODBUS_GET_FIRMWARE_VERSION)
        return val / 10.0 if val >= 0 else -1

    def get_pro_gripper_firmware_modified_version(self, gripper_id=14):
        """ Read the firmware revision number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            version number (int)
        """
        val = self._read_register(gripper_id, ProGripper.MODBUS_GET_FIRMWARE_MODIFY_VERSION)
        return val if val >= 0 else -1

    def set_pro_gripper_id(self, target_id, gripper_id=14):
        """ Set the gripper ID

        Args:
            target_id (int): Target ID, 1 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, target_id=target_id)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ID, target_id)

    def get_pro_gripper_id(self, gripper_id=14):
        """ Read the gripper ID

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_id (int): 1 ~ 254
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_ID)

    def set_pro_gripper_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ANGLE, gripper_angle)

    def get_pro_gripper_angle(self, gripper_id=14):
        """ Get the gripper angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_ANGLE)

    def set_pro_gripper_open(self, gripper_id=14):
        """ Open the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        return self.set_pro_gripper_angle(100, gripper_id)

    def set_pro_gripper_close(self, gripper_id=14):
        """ Close the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        return self.set_pro_gripper_angle(0, gripper_id)

    def set_pro_gripper_calibration(self, gripper_id=14):
        """ Set the gripper zero position

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_CALIBRATION, 0)

    def get_pro_gripper_status(self, gripper_id=14):
        """ Get the gripper status

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_STATUS)

    def set_pro_gripper_enabled(self, state, gripper_id=14):
        """ Set the gripper enable state

        Args:
            state (bool): 0 or 1, 0 - Disable 1 - Enable
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ENABLED, state)

    def set_pro_gripper_torque(self, gripper_torque, gripper_id=14):
        """ Set the gripper torque

        Args:
            gripper_torque (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_torque=gripper_torque)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_TORQUE, gripper_torque)

    def get_pro_gripper_torque(self, gripper_id=14):
        """ Set the gripper torque

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            gripper_torque (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_TORQUE)

    def set_pro_gripper_speed(self, speed, gripper_id=14):
        """ Set the gripper torque

        Args:
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed=speed)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_SPEED, speed)

    def get_pro_gripper_speed(self, gripper_id=14):
        """ Get the gripper speed

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            speed (int): 1 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_SPEED)

    def set_pro_gripper_abs_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper absolute angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_ABS_ANGLE, gripper_angle)

    def set_pro_gripper_io_open_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper IO open angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_IO_OPEN_ANGLE, gripper_angle)

    def get_pro_gripper_io_open_angle(self, gripper_id=14):
        """ Get the gripper IO open angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_IO_OPEN_ANGLE)

    def set_pro_gripper_io_close_angle(self, gripper_angle, gripper_id=14):
        """ Set the gripper IO close angle

        Args:
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_angle=gripper_angle)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_IO_CLOSE_ANGLE, gripper_angle)

    def get_pro_gripper_io_close_angle(self, gripper_id=14):
        """ Get the gripper IO close angle

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            angle (int): 0 ~ 100
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_IO_CLOSE_ANGLE)

    def set_pro_gripper_mini_pressure(self, pressure_value, gripper_id=14):
        """ Set the gripper mini pressure

        Args:
            pressure_value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pressure_value=pressure_value)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MINI_PRESSURE, pressure_value)

    def get_pro_gripper_mini_pressure(self, gripper_id=14):
        """ Get the gripper mini pressure

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            mini pressure (int): 0 ~ 254
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_MINI_PRESSURE)

    def set_pro_gripper_protection_current(self, current_value, gripper_id=14):
        """ Set the gripper protection current

        Args:
            current_value (int): 100 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, current_value=current_value)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_PROTECTION_CURRENT, current_value)

    def get_pro_gripper_protection_current(self, gripper_id=14):
        """ Get the gripper protection current

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            current_value (int): 100 ~ 300
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_PROTECTION_CURRENT)

    def set_pro_gripper_modbus(self, state, custom_mode=False, gripper_id=14):
        """ Set the gripper modbus mode

        Args:
            state (int): 0 or 1, 0 - close modbus 1 - open modbus
            custom_mode (bool):
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 -  failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        if custom_mode:
            return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MODE, state, custom_mode=custom_mode)
        else:
            return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_MODE, state)

    def set_pro_gripper_baud(self, baud_rate=0, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            baud_rate (int): 0 ~ 5, defaults to 0 - 115200
                0 - 115200
                1 - 1000000
                2 - 57600
                3 - 19200
                4 - 9600
                5 - 4800
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            1 - success, -1 - failed
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_baud_rate=baud_rate)
        return self._write_and_check(gripper_id, ProGripper.MODBUS_SET_BAUD_RATE, baud_rate)

    def get_pro_gripper_baud(self, gripper_id=14):
        """ Set the gripper baud rate

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Returns:
            baud_rate (int): 0 ~ 5, defaults to 0 - 115200
                0 - 115200
                1 - 1000000
                2 - 57600
                3 - 19200
                4 - 9600
                5 - 4800
        """
        return self._read_register(gripper_id, ProGripper.MODBUS_GET_BAUD_RATE)

    def set_tool_serial_baud_rate(self, baud_rate=115200):
        """ Set the end 485 baud rate

            Args:
                baud_rate (int): Standard baud rates, such as 115200, 1000000, 57600, 19200, 9600, 4800.
                                defaults to 115200
            """
        self.calibration_parameters(class_name=self.__class__.__name__, end_485_baud_rate=baud_rate)
        data = bytearray()
        data += baud_rate.to_bytes(4, 'big')
        return self._mesg(ProtocolCode.SET_TOOL_485_BAUD_RATE, *data)

    def set_tool_serial_timeout(self, timeout=10000):
        """
        Set end 485 timeout (unit: ms)

        Args:
            timeout (int): Timeout period, in ms, range 0 ~ 10000 ms, defaults to 10000
        """
        self.calibration_parameters(class_name=self.__class__.__name__, timeout=timeout)

        high_byte = (timeout >> 8) & 0xFF
        low_byte = timeout & 0xFF

        return self._mesg(ProtocolCode.SET_TOOL_SERIAL_TIMEOUT, high_byte, low_byte)

    def get_tool_config(self):
        """ Get the end 485 baud rate and timeout

            Returns: (list) [baud_rate, timeout]
            """
        return self._mesg(ProtocolCode.GET_TOOL_485_BAUD_RATE_TIMEOUT)

    def set_free_move_mode(self, mode):
        """ Set the free move mode"""
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        return self._mesg(ProtocolCode.SET_FREE_MODE, mode)

    def get_free_move_mode(self):
        """ Set the free move mode"""
        return self._mesg(ProtocolCode.IS_FREE_MODE)

    def set_pro_gripper_init(self, gripper_id=14):
        """
        Initialize the Pro450 gripper and automatically recover communication.

        This function automatically handles **four possible gripper states** and
        brings the device back to Modbus mode at **115200 baud**:

        1. Already in Modbus mode with baudrate = 115200
           - Angle reading works directly.

        2. Modbus mode but baudrate incorrect (e.g., 1,000,000)
           - Angle fails at 115200 but succeeds at alternative baudrate.

        3. Custom (non-Modbus) mode + correct baudrate
           - Baudrate is correct but Modbus commands fail.
           - Forcing Modbus mode succeeds.

        4. Custom mode + wrong baudrate
           - Requires scanning possible baudrates and forcing Modbus mode.

        After successful initialization:
          - Gripper is set to Modbus mode (mode = 1)
          - Baudrate is restored to 115200
          - Tool serial port baudrate is restored to 115200

        Args:
            gripper_id (int): Modbus ID of the gripper, range 1鈥?54.
                Defaults to 14.

        Returns:
            bool: True if initialization succeeds, otherwise False.
        """

        try_bauds = [115200, 1000000]

        print("The gripper is initializing, please wait...")

        self.set_tool_serial_timeout(250)

        test = self.get_pro_gripper_angle(gripper_id=gripper_id)
        if test != -1:
            self.set_pro_gripper_modbus(1, gripper_id=gripper_id)  # ensure normal modbus mode

            self.set_pro_gripper_baud(0, gripper_id=gripper_id)  # gripper -> 115200

            self.set_tool_serial_baud_rate(115200)  # end -> 115200
            self.set_tool_serial_timeout(10000)

            print("Gripper Initialization Successful!")
            return True

        for baud in try_bauds:
            # print(f"\n馃憠 Try the end baud rate: {baud}")
            self.set_tool_serial_baud_rate(baud_rate=baud)

            cfg = self.get_tool_config()
            # print(f"   485 current config: {cfg}")

            # Read the angle again, applicable to: Baud rate = Correct, Mode = Modbus
            test = self.get_pro_gripper_angle(gripper_id=gripper_id)
            # print(f"馃攣 Test Modbus to read angle return: {test}")

            if test != -1:
                self.set_pro_gripper_baud(0, gripper_id=gripper_id)
                self.set_tool_serial_baud_rate(115200)
                self.set_tool_serial_timeout(10000)
                print("Gripper Initialization Successful!")
                return True

            # print(f"\n馃憠 Try the end baud rate again: {baud}")
            self.set_tool_serial_baud_rate(baud_rate=baud)
            cfg = self.get_tool_config()
            # print(f"   485 current config: {cfg}")
            ret = self.set_pro_gripper_modbus(1, True, gripper_id=gripper_id)
            # print(f"   set_modbus(custom) ret={ret}")

            if ret == 1:
                for i in range(3):
                    t = self.get_pro_gripper_angle(gripper_id=gripper_id)
                    # print(f" 馃敡 Test angle read[{i}] -> {t}")
                    if t != -1:
                        break
                    time.sleep(0.1)

                if t != -1:

                    self.set_pro_gripper_baud(0, gripper_id=gripper_id)  # change gripper 鈫?115200

                    self.set_tool_serial_baud_rate(115200)  # end back 鈫?115200
                    self.set_tool_serial_timeout(10000)

                    print("Gripper Initialization Successful!")
                    return True

        print("Gripper Initialization Failed!")


class ThreeHand(EndControlBase):
    def _send_custom_command(self, gripper_id, func_code, reg_addr, *data):
        """
        General Modbus command sending method

        Args:
            gripper_id: Device ID
            func_code: Function code (0x03 = read, 0x06 = write)
            reg_addr: Register address
            data      : Variable payload bytes
        """
        self._clear_serial_input_buffer()

        # Base payload part shared by both modes
        payload = [gripper_id, func_code,
                   (reg_addr >> 8) & 0xFF, reg_addr & 0xFF]

        # Append value for write, or 0x00 0x00 for read
        # No parameters -> use 00 00 placeholder
        if not data:
            payload.extend([0x00, 0x00])
        else:
            payload.extend(data)

        # Custom packet: FE FE LEN + payload + CRC(big-end)
        # LEN = payload length + CRC length (2)锛屽嵆 6+2 = 8
        cmd = [0xFE, 0xFE, len(payload) + 2] + payload
        cmd.extend(self._modbus_crc(cmd, mode='big'))
        self._clear_serial_input_buffer()
        recv = self.tool_serial_write_data(cmd)
        if not recv:
            return cmd, -1
        return cmd, recv

    def _write_custom_check_bytes(self, gripper_id, reg_addr, *data):
        """Write custom register with raw byte parameters."""
        self._check_gripper_id(gripper_id)
        _, recv = self._send_custom_command(gripper_id, 0x06, reg_addr, *data)
        if not isinstance(recv, (list, bytearray)) or len(recv) < 9:
            return -1

        offset = 3
        cmd_idx = 1 + offset
        reg_h_idx = 2 + offset
        reg_l_idx = 3 + offset
        val_h_idx = 4 + offset
        val_l_idx = 5 + offset

        if recv[cmd_idx] != 0x06:
            return -1
        if recv[reg_h_idx] != (reg_addr >> 8) & 0xFF or recv[reg_l_idx] != (reg_addr & 0xFF):
            return -1
        if recv[val_h_idx] == 0x00 and recv[val_l_idx] == 0x01:
            return 1
        return -1

    def _write_custom_check(self, gripper_id, reg_addr, *values):
        """Write register and verify response robustly (support calibration delay)"""
        self._check_gripper_id(gripper_id)
        data = []
        for value in values:
            # support list / tuple
            if isinstance(value, (list, tuple)):
                for v in value:
                    data.extend([(v >> 8) & 0xFF, v & 0xFF])
            else:
                data.extend([(value >> 8) & 0xFF, value & 0xFF])

        # Continuously read the response packets, and send a read command to trigger feedback each time.
        _, recv = self._send_custom_command(gripper_id, 0x06, reg_addr, *data)
        # Basic validity check
        if not isinstance(recv, (list, bytearray)) or len(recv) < 9:
            return -1

        # Two modes have different byte offsets
        # Modbus RTU standard: [id][cmd][regH][regL][valH][valL]...
        # Custom packet       : [fe][fe][len][id][cmd][regH][regL][valH][valL]...
        offset =  3

        cmd_idx = 1 + offset  # command index
        reg_h_idx = 2 + offset  # register high
        reg_l_idx = 3 + offset  # register low
        val_h_idx = 4 + offset  # value high
        val_l_idx = 5 + offset  # value low

        # Verify command
        if recv[cmd_idx] != 0x06:
            return -1

        # Verify register address consistency
        if recv[reg_h_idx] != (reg_addr >> 8) & 0xFF or recv[reg_l_idx] != (reg_addr & 0xFF):
            return -1

        # Determine return status
        if recv[val_h_idx] == 0x00 and recv[val_l_idx] == 0x01:
            return 1

        return -1

    def _read_custom_register(self, gripper_id, reg_addr, *values):
        """Reads a register with command verification"""
        self._check_gripper_id(gripper_id)
        data = []
        for value in values:
            # support list / tuple
            if isinstance(value, (list, tuple)):
                for v in value:
                    data.extend([(v >> 8) & 0xFF, v & 0xFF])
            else:
                data.extend([(value >> 8) & 0xFF, value & 0xFF])

        cmd, recv = self._send_custom_command(gripper_id, 0x03, reg_addr, *data)
        if isinstance(recv, (list, bytearray)) and len(recv) >= 9:
            recv_func = recv[4]
            recv_addr = (recv[5] << 8) | recv[6]
            if recv_func == 0x03 and recv_addr == reg_addr:
                data = recv[7:-2]
                if len(data) % 2 != 0:
                    return -1
                res = []
                for index in range(0, len(data), 2):
                    res.append((data[index] << 8) | data[index + 1])
                return res[0] if len(res) == 1 else res
            else:
                return -1
        else:
            return -1


    # myHand  Gripper Control
    def get_hand_firmware_major_version(self, gripper_id=14):
        """Read the firmware major version number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        val = self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_MAJOR_FIRMWARE_VERSION)
        return val / 10.0 if val >= 0 else -1

    def get_hand_firmware_minor_version(self, gripper_id=14):
        """Read the firmware minor version number

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        val = self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_MINOR_FIRMWARE_VERSION)
        return val if val >= 0 else -1

    def set_hand_gripper_id(self, target_id, gripper_id=14):
        """Set the gripper ID

        Args:
            target_id (int): 1 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, target_id=target_id)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_ID, target_id)

    def get_hand_gripper_id(self, gripper_id=14):
        """Get the gripper ID

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper ID (int): 1 ~ 254
        """
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_ID)

    def set_hand_gripper_angle(self, joint_id, gripper_angle, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_angle (int): 0 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id, hand_gripper_angle=gripper_angle)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_ANGLE, joint_id, gripper_angle)

    def get_hand_gripper_angle(self, joint_id, gripper_id=14):
        """Get the angle of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper_angle (int): 0 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_ANGLE, joint_id)

    def set_hand_gripper_angles(self, gripper_angles, speed, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            gripper_angles (list): A list of integers, length 6, each value range 0 ~ 100
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14
        """
        self.calibration_parameters(class_name=self.__class__.__name__,  hand_gripper_angles=gripper_angles, hand_gripper_speed=speed)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_ANGLES, gripper_angles, speed)

    def get_hand_gripper_angles(self, gripper_id=14):
        """Set the angle of the single joint of the gripper

        Args:
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            gripper_angles (list): A list of integers, length 6
        """
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_ALL_ANGLES)

    def set_hand_gripper_torque(self, joint_id, torque_value, gripper_id=14):
        """ Setting gripper torque

        Args:
            joint_id (int): 1 ~ 6
            torque_value (int): 0 ~ 300
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            -1: Set failed
            1: Set successful
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id, gripper_torque=torque_value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_TORQUE, joint_id, torque_value)

    def get_hand_gripper_torque(self, joint_id, gripper_id=14):
        """ Setting gripper torque

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            torque_value (int): 0 ~ 100
        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_TORQUE, joint_id)

    def set_hand_gripper_calibrate(self, joint_id, gripper_id=14):
        """ Setting the gripper jaw zero position

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_CALIBRATION, joint_id, 0)

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
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_STATUS)

    def set_hand_gripper_enabled(self, flag, gripper_id=14):
        """ Set the enable state of the gripper

        Args:
            flag (int): 0 or 1; 0 - disable enable, 1 - enable enable.
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_flag=flag)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_ENABLED, flag)

    def set_hand_gripper_speed(self, joint_id, speed, gripper_id=14):
        """ Set the speed of the gripper

        Args:
            joint_id (int): 1 ~ 6
            speed (int): 1 ~ 100
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id, hand_gripper_speed=speed)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_SPEED, joint_id, speed)

    def get_hand_gripper_default_speed(self,joint_id, gripper_id=14):
        """ Get the default speed of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            default speed (int): 1 ~ 100

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_DEFAULT_SPEED, joint_id)

    def set_hand_gripper_p(self, joint_id, value, gripper_id=14):
        """ Set the P value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, gripper_p=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_P, joint_id, value)

    def get_hand_gripper_p(self, joint_id, gripper_id=14):
        """ Get the P value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            P value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_P, joint_id)

    def set_hand_gripper_d(self, joint_id, value, gripper_id=14):
        """ Set the D value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, gripper_d=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_D, joint_id, value)

    def get_hand_gripper_d(self, joint_id, gripper_id=14):
        """ Get the D value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            D value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_D, joint_id)

    def set_hand_gripper_i(self, joint_id, value, gripper_id=14):
        """ Set the I value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, gripper_i=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_I, joint_id, value)

    def get_hand_gripper_i(self, joint_id, gripper_id=14):
        """ Get the I value of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            I value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_I, joint_id)

    def set_hand_gripper_min_pressure(self, joint_id, value, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 254
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, min_pressure=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_MIN_PRESSURE, joint_id, value)

    def get_hand_gripper_min_pressure(self, joint_id, gripper_id=14):
        """ Set the minimum starting force of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            min pressure value (int): 0 ~ 254

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_MIN_PRESSURE, joint_id)

    def set_hand_gripper_clockwise(self, joint_id, value, gripper_id=14):
        """ Set the clockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 16
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, clockwise=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_CLOCKWISE, joint_id, value)

    def get_hand_gripper_clockwise(self, joint_id, gripper_id=14):
        """ Get the clockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            value (int): 0 ~ 16

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_CLOCKWISE, joint_id)

    def set_hand_gripper_counterclockwise(self, joint_id, value, gripper_id=14):
        """ Set the counterclockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            value (int): 0 ~ 16
            gripper_id (int): 1 ~ 254, defaults to 14

        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_gripper_joint_id=joint_id, clockwise=value)
        return self._write_custom_check(gripper_id, MyHandGripper.SET_HAND_GRIPPER_COUNTERCLOCKWISE, joint_id, value)

    def get_hand_gripper_counterclockwise(self, joint_id, gripper_id=14):
        """ Get the counterclockwise runnable error of the single joint of the gripper

        Args:
            joint_id (int): 1 ~ 6
            gripper_id (int): 1 ~ 254, defaults to 14

        Return:
            value (int): 0 ~ 16

        """
        self.calibration_parameters(class_name=self.__class__.__name__, hand_gripper_joint_id=joint_id)
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_COUNTERCLOCKWISE, joint_id)

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
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_pinch_pose=pinch_pose, hand_rank_mode=rank_mode)
            return self._write_custom_check_bytes(gripper_id, MyHandGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode)

        else:
            self.calibration_parameters(class_name=self.__class__.__name__, gripper_id=gripper_id, hand_pinch_pose=pinch_pose, hand_rank_mode=rank_mode, hand_idle_flag=idle_flag)
            return self._write_custom_check_bytes(gripper_id, MyHandGripper.SET_HAND_GRIPPER_PINCH_ACTION_SPEED_CONSORT, pinch_pose, rank_mode, idle_flag)

    def get_hand_gripper_model(self, gripper_id=14):
        """ Get the model of the gripper"""
        return self._read_custom_register(gripper_id, MyHandGripper.GET_HAND_GRIPPER_ROBOT_MODEL)



class FiveFinger(EndControlBase):
    def _five_fingers_modbus_read(self, slave_id, start_reg, reg_count, retry=3):
        """Aoyi Five-Finger Dexterity Hand Universal Modbus Read Holding Register Interface

        Args:
            slave_id (int): Device ID (your dexterity hand is 0x02)
            start_reg (int): Starting register address
            reg_count (int): Number of registers to read
            retry (int): Number of retries

        Returns:
            list[int] or -1
        """
        for attempt in range(retry):
            cmd = [
                slave_id,
                0x03,  # Read register
                (start_reg >> 8) & 0xFF,
                start_reg & 0xFF,
                (reg_count >> 8) & 0xFF,
                reg_count & 0xFF
            ]

            # CRC
            cmd += list(self._modbus_crc(bytes(cmd)))
            self._clear_serial_input_buffer()
            recv = self.tool_serial_write_data(cmd)
            if not recv or recv== -1 or len(recv) < 5:
                # print("Modbus no response:", recv)
                continue

            # Find the Modbus starting point
            start = None
            for i in range(len(recv) - 1):
                if recv[i] == slave_id and recv[i + 1] == 0x03:
                    start = i
                    break

            if start is None:
                # print("Modbus header not found:", recv)
                continue

            recv = recv[start:]

            # Length check
            if len(recv) < 3:
                continue
            byte_count = recv[2]
            expected_len = 3 + byte_count + 2

            if len(recv) < expected_len:
                # print("Incomplete frame:", recv)
                continue
            recv = recv[:expected_len]

            # CRC check
            data = recv[:-2]
            crc_recv = recv[-2:]

            if self._modbus_crc(bytes(data)) != bytes(crc_recv):
                continue

            # Data Analysis Format: [addr, func, byte_count, data..., crc_l, crc_h]
            data_bytes = recv[3:3 + byte_count]

            values = []
            for i in range(0, len(data_bytes), 2):
                val = (data_bytes[i] << 8) | data_bytes[i + 1]
                values.append(val)

            return values
        return -1

    def _five_fingers_modbus_write(self, slave_id, start_reg, values):
        reg_count = len(values)
        byte_count = reg_count * 2

        cmd = [
            slave_id,
            0x10,
            (start_reg >> 8) & 0xFF,
            start_reg & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF,
            byte_count
        ]

        for v in values:
            high = (v >> 8) & 0xFF
            low = v & 0xFF
            cmd.extend([high, low])

        cmd += list(self._modbus_crc(bytes(cmd)))

        self._clear_serial_input_buffer()
        recv = self.tool_serial_write_data(cmd)

        if recv is None or recv == -1:
            return -1

        # Find the starting address for verification
        start = None
        for i in range(len(recv) - 1):
            if recv[i] == slave_id and recv[i + 1] == 0x10:
                start = i
                break

        if start is None:
            return -1

        recv = recv[start:]

        if len(recv) < 8:
            return -1

        data = recv[:-2]
        crc_recv = recv[-2:]

        if self._modbus_crc(bytes(data)) != bytes(crc_recv):
            return -1


    def get_five_fingers_angles(self, hand_id=2):
        """Read the angle of the five fingers (unit: degrees)

        Args:
            hand_id (int): Hand ID, range 2 ~ 254, default 2

        Returns:
            list[float]: For example, [33.54, 173.83, 171.68, 172.1, 174.71, 1.0] represent
                        [thumb bending, index finger, middle finger, ring finger, little finger, and thumb rotation], respectively.
        """

        start_reg = 0x0483
        reg_count = 0x06
        self.calibration_parameters(class_name=self.__class__.__name__, five_hand_id=hand_id)

        raw = self._five_fingers_modbus_read(hand_id, start_reg, reg_count)

        if not raw or raw==-1:
            return -1
        if not isinstance(raw, list):
            return -1
        angles = [round(v / 100.0, 2) for v in raw]

        return angles

    def get_five_fingers_angle(self, finger_id, hand_id=2):
        """Read the angle of a single joint of the five fingers

        Args:
            finger_id (int): 1 ~ 6
                    1 - thumb bending
                    2 - index finger
                    3 - middle finger
                    4 - ring finger
                    5 - little finger
                    6 - thumb rotation
            hand_id (int): Hand ID, range 2 ~ 254, default 2

        Returns:
            float: angle value
        """

        self.calibration_parameters(class_name=self.__class__.__name__, finger_id=finger_id, five_hand_id=hand_id)
        start_reg = 1155 + (finger_id - 1)
        raw = self._five_fingers_modbus_read(hand_id, start_reg, 1)

        if not raw or raw==-1:
            return -1
        if not isinstance(raw, list):
            return -1

        angle = raw[0] / 100.0

        return angle

    def set_five_fingers_angles(self, fingers_angles, hand_id=2):
        """Set all finger angles.

        Args:
            fingers_angles (list): A list of length 6, where J1-J6 represent [thumb bending, index finger, middle finger, ring finger, little finger, thumb rotation] respectively.
                           J1:2.26掳 ~ 36.76
                           J2:100.22掳~178.37掳
                           J3:97.81掳 ~ 176.06掳
                           J4:101.38掳 ~ 176.54掳
                           J5:98.84掳 ~ 174.86掳
                           J6:0掳 ~ 90掳
            hand_id (int): Five-finger device ID, range 2 ~ 254, default 2

        Returns:
            int
        """
        self.calibration_parameters(class_name=self.__class__.__name__, five_fingers_angles=fingers_angles, five_hand_id=hand_id)
        values = [int(a * 100 + 1e-8) for a in fingers_angles]

        return self._five_fingers_modbus_write(hand_id, 0x0483, values)

    def set_five_fingers_angle(self, finger_id, finger_angle, hand_id=2):
        """Set single finger angles

        Args:
            finger_id (int): 1 ~ 6
                    1 - thumb bending
                    2 - index finger
                    3 - middle finger
                    4 - ring finger
                    5 - little finger
                    6 - thumb rotation
            finger_angle (int or float) : angle value
                            J1:2.26掳 ~ 36.76
                            J2:100.22掳~178.37掳
                            J3:97.81掳 ~ 176.06掳
                            J4:101.38掳 ~ 176.54掳
                            J5:98.84掳 ~ 174.86掳
                            J6:0掳 ~ 90掳
            hand_id (int): Five-finger device ID, range 2 ~ 254, default 2
        """

        self.calibration_parameters(class_name=self.__class__.__name__, finger_id=finger_id,
                                    five_finger_angle=finger_angle, five_hand_id=hand_id)

        reg = 1155 + (finger_id - 1)

        value = int(finger_angle * 100)

        return self._five_fingers_modbus_write(hand_id, reg, [value])

    def get_five_fingers_version(self, hand_id=2):
        """Read the firmware major and minor version numbers of the five fingers

        Args:
            hand_id (int): Hand ID, range 2 ~ 254, default 2

        Returns:
            float: Major and minor version numbers, such as 3.1
        """
        raw = self._five_fingers_modbus_read(hand_id, 1001, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1

        val = raw[0]

        major = (val >> 8) & 0xFF
        minor = val & 0xFF

        version = f"{major}.{minor}"
        return version

    def get_five_fingers_modified_version(self, hand_id=2):
        """Read firmware and modify version number of the five fingers

        Args:
            hand_id (int): Hand ID, range 2 ~ 254, default 2

        Returns:
            int: modified version numbers, such as 79
        """
        raw = self._five_fingers_modbus_read(hand_id, 1002, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1
        val = raw[0]
        return val

    def get_five_fingers_hand_id(self, hand_id=2):
        """Read hand id of the five fingers

        Args:
            hand_id (int): Hand ID, range 2 ~ 254, default 2

        Returns:
            int: hand ID, such as 2
        """
        raw = self._five_fingers_modbus_read(hand_id, 1005, 1)
        if raw == -1:
            return -1

        if not isinstance(raw, list) or len(raw) < 1:
            return -1

        val = raw[0]

        roh_hand_id = val & 0xFF

        return roh_hand_id

    def set_five_fingers_hand_id(self, target_hand_id, hand_id=2):
        """Set hand id of the five fingers

        Args:
            target_hand_id (int): Hand ID, range 2 ~ 254, default 2
            hand_id (int): Hand ID, range 2 ~ 254, default 2
        """
        self.calibration_parameters(class_name=self.__class__.__name__, target_five_hand_id=target_hand_id)

        return self._five_fingers_modbus_write(hand_id, 1005, [target_hand_id])


class E1EndControl(ForceGripper, ThreeHand, FiveFinger):
    pass
