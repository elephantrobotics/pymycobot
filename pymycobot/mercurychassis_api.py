#!/usr/bin/python
# -*- coding:utf-8 -*-
import traceback
from datetime import datetime
import time
import serial


class ProtocolCode(object):
    # send data
    header = 0x7B
    footer = 0x7D
    # Ultrasonic data
    ultrasound_header = 0xFA
    ultrasound_footer = 0xFC


class ChassisControl:

    def __init__(self, port, baudrate=115200, timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info, default: False
        """

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = True
        self._serial_port.dtr = True
        self._serial_port.open()
        self.debug = debug
        self.Send_Data = [0] * 11

    def _read(self):
        data = b""
        time.sleep(0.1)
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
        return data

    def _debug(self, data):
        """whether show info."""
        hex_data = " ".join(f"{value:02X}" for value in data)
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if self.debug:
            print(f"\n***** Debug Info *****\n{current_time} send command: {hex_data}")

    def close(self):
        self._serial_port.close()

    def open(self):
        self._serial_port.open()

    def _request(self, flag=""):
        """
        Read Data
        :param flag: Data type parameter variable
        :return:
        """
        receive_all_data = self._read()
        receive_all_data = [byte for byte in receive_all_data]
        return_data = ''
        if flag == "voltage":
            receive_data = self._extract_frame(receive_all_data, ProtocolCode.header, ProtocolCode.footer)
            # print(receive_data, len(receive_data))
            self._debug(receive_data)
            transition_16 = 0
            transition_16 |= receive_data[20] << 8
            transition_16 |= receive_data[21]
            return_data = round(transition_16 / 1000 + (transition_16 % 1000) * 0.001, 3)

            return return_data

        elif flag == "ultrasonic":
            receive_data = self._extract_frame(receive_all_data, ProtocolCode.ultrasound_header,
                                               ProtocolCode.ultrasound_footer)
            # print(receive_data, len(receive_data))
            self._debug(receive_data)
            return_data = [receive_data[1] * 256 + receive_data[2], receive_data[3] * 256 + receive_data[4],
                           receive_data[5] * 256 + receive_data[6]]
            return return_data
        
        elif flag == "version":
            receive_data = self._extract_frame(receive_all_data, ProtocolCode.header, ProtocolCode.footer)
            # print(receive_data, len(receive_data))
            self._debug(receive_data)
            
            major_version = receive_data[22]
            minor_version = receive_data[23]
            return_data = f"v{major_version}.{minor_version}"

            return return_data

        else:
            print('no data', return_data)

    def _extract_frame(self, data, frame_header, frame_tail):
        """
        Extract the corresponding data
        :param data: Get all data
        :param frame_header: Frame Header
        :param frame_tail: Frame tail
        :return: A list of integers
        """
        try:
            # Find the location of the frame header
            start_index = data.index(frame_header)
            # Find the end of the frame, starting from the frame header
            end_index = data.index(frame_tail, start_index)
            # Extracting dataframe
            frame = data[start_index:end_index + 1]
            return frame
        except ValueError:
            # If the frame header or frame trailer is not found, an empty list is returned.
            return []

    def _check_sum(self, count_number, mode):
        """
        Calculate the check digit
        :param count_number: The total number of bytes before the check digit
        :param mode: Verify whether to send or receive data, 0-Receive data mode, 1-Send data mode
        :return: A byte type
        """
        check_sum_result = 0
        # 接收数据模式
        if mode == 0:
            for k in range(count_number):
                check_sum_result ^= self.Send_Data[k]
        # 发送数据模式
        elif mode == 1:
            for k in range(count_number):
                check_sum_result ^= self.Send_Data[k]

        return check_sum_result

    def get_power_voltage(self):
        """
        Get battery voltage
        :return: A floating point voltage value, Unit: Volt
        """
        return self._request("voltage")

    def get_ultrasonic_value(self):
        """
        Get ultrasonic value
        :return: A list of length 3,Unit: mm
        """
        return self._request('ultrasonic')
    
    def get_base_version(self):
        """
        Get base version
        :return: A string representing the firmware version in the format 'vX.X' (e.g., 'v1.1').
        """
        return self._request('version')

    def go_straight(self, speed=0.2):
        """
        Forward control
        :param speed: speed (float, optional): Movement speed. Defaults to 0.2. range 0 ~ 0.5 m/s
        :return: None
        """
        if speed < 0 or speed > 0.5:
            raise Exception("The movement speed range is 0~0.5, but the received value is {}".format(speed))
        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 0
        self.Send_Data[2] = 0
        # The target velocity of the X-axis of the robot
        transition = int(speed * 1000)
        self.Send_Data[4] = transition & 0xFF  # Lower 8 bits
        self.Send_Data[3] = (transition >> 8) & 0xFF  # Higher 8 bits
        # The target velocity of the Y-axis of the robot
        self.Send_Data[6] = 0
        self.Send_Data[5] = 0

        self.Send_Data[8] = 0
        self.Send_Data[7] = 0
        # Check the bits for the Check_Sum function
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer

        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))

    def go_back(self, speed=-0.2):
        """
        Back control
        :param speed: speed (float, optional): Movement speed. Defaults to -0.2. range -0.5 ~ 0 m/s
        :return: None
        """
        if not -0.5 <= speed <= 0:
            raise Exception("The movement speed range is -0.5~0, but the received value is {}".format(speed))
        self.Send_Data = [0] * 11
        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 0
        self.Send_Data[2] = 0
        # The target velocity of the X-axis of the robot
        transition = int(speed * 1000)
        if transition < 0:
            transition = (1 << 16) + transition
        self.Send_Data[4] = transition & 0xFF  # Lower 8 bits
        self.Send_Data[3] = (transition >> 8) & 0xFF  # Higher 8 bits
        # The target velocity of the Y-axis of the robot
        self.Send_Data[6] = 0
        self.Send_Data[5] = 0
        # The target velocity of the Z-axis of the robot
        self.Send_Data[8] = 0
        self.Send_Data[7] = 0
        # Check the bits for the Check_Sum function
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer

        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))

    def turn_left(self, speed=0.2):
        """
        Left turn control
        :param speed: speed (float, optional): Movement speed. Defaults to 0.2. range 0 ~ 0.5 m/s
        :return: None
        """

        if speed < 0 or speed > 0.5:
            raise Exception("The movement speed range is 0~5, but the received value is {}".format(speed))
        self.Send_Data = [0] * 11
        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 0
        self.Send_Data[2] = 0
        # The target velocity of the X-axis of the robot
        self.Send_Data[4] = 0
        self.Send_Data[3] = 0
        # The target velocity of the Y-axis of the robot
        self.Send_Data[6] = 0
        self.Send_Data[5] = 0
        # The target velocity of the Z-axis of the robot
        transition = int(speed * 1000)
        self.Send_Data[8] = transition & 0xFF  # Lower 8 bits
        self.Send_Data[7] = (transition >> 8) & 0xFF  # Higher 8 bits
        # Check the bits for the Check_Sum function
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer

        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))

    def turn_right(self, speed=-0.2):
        """
        Right turn control
        :param speed: speed (float, optional): Movement speed. Defaults to -0.2. range -0.5 ~ 0 m/s
        :return: None
        """
        if not -0.5 <= speed <= 0:
            raise Exception("The movement speed range is -0.5~0, but the received value is {}".format(speed))
        self.Send_Data = [0] * 11

        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 0
        self.Send_Data[2] = 0
        # The target velocity of the X-axis of the robot
        self.Send_Data[4] = 0
        self.Send_Data[3] = 0
        # The target velocity of the Y-axis of the robot
        self.Send_Data[6] = 0
        self.Send_Data[5] = 0
        # The target velocity of the Z-axis of the robot
        transition = int(speed * 1000)
        if transition < 0:
            transition = (1 << 16) + transition
        self.Send_Data[8] = transition & 0xFF  # Lower 8 bits
        self.Send_Data[7] = (transition >> 8) & 0xFF  # Higher 8 bits
        # Check the bits for the Check_Sum function
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer

        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))

    def stop(self):
        """
        stop motion
        :return:
        """
        self.Send_Data = [0] * 11
        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 0
        self.Send_Data[2] = 0
        # The target velocity of the X-axis of the robot
        self.Send_Data[4] = 0
        self.Send_Data[3] = 0
        # The target velocity of the Y-axis of the robot
        self.Send_Data[6] = 0
        self.Send_Data[5] = 0
        # The target velocity of the Z-axis of the robot
        self.Send_Data[8] = 0
        self.Send_Data[7] = 0
        # Check the bits for the Check_Sum function
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer

        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))

    def set_color(self, r=0, g=0, b=0):
        """
        Set the color control of the X1 base light strip.
        :param r: (int): 0 ~ 255
        :param g: (int): 0 ~ 255
        :param b: (int): 0 ~ 255
        :return: None
        """
        if not (isinstance(r, int) and isinstance(g, int) and isinstance(b, int)):
            raise ValueError("r, g, and b must be integers.")
        if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
            raise ValueError("r, g, and b must be in the range 0-255.")

        self.Send_Data[0] = ProtocolCode.header
        self.Send_Data[1] = 4
        if r == 0 and g == 0 and b == 0:
            self.Send_Data[2] = 0
        else:
            self.Send_Data[2] = 1
        self.Send_Data[3] = r
        self.Send_Data[4] = g
        self.Send_Data[5] = b
        self.Send_Data[6] = 0
        self.Send_Data[7] = 0
        self.Send_Data[8] = 0
        self.Send_Data[9] = self._check_sum(9, 1)
        self.Send_Data[10] = ProtocolCode.footer
        try:
            self._serial_port.write(bytes(self.Send_Data))
            self._serial_port.flush()
            self._debug(self.Send_Data)
        except serial.SerialException as e:
            e = traceback.format_exc()
            print('Unable to send data through serial port: {}'.format(e))


