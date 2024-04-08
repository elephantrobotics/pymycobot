
import enum
import serial
import time
import struct
import logging
from pymycobot.log import setup_logging
from pymycobot.common import DataProcessor
from pymycobot.error import calibration_parameters

class ProtocolCode(enum.Enum):
    HEADER = 0xFE
    RESTORE = [0x01, 0x00]
    SET_LED = [0x01, 0x02]
    GET_FIRMWARE_VERSION = [0x01, 0x03]
    GET_MOTORS_CURRENT = [0x01, 0x04]
    GET_BATTERY_INFO = [0x01, 0x05]
    SET_GYRO_STATE = [0x01, 0x07]
    GET_GYRO_STATE = [0x01, 0x08]
    GET_MODIFIED_VERSION = [0x01, 0x09]

class MyAgv(DataProcessor):
    def __init__(self, port="/dev/ttyAMA0", baudrate="115200", timeout=0.1, debug=False):
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        
    def _write(self, command):
        self._serial_port.reset_input_buffer()
        self.log.debug("_write: {}".format([hex(i) for i in command]))
        self._serial_port.write(command)
        self._serial_port.flush()
        
    def _read(self, command):
        datas = b''
        k = 0
        pre = 0
        end = 5
        t = time.time()
        if command[k-1] == 29:
            end = 28
        elif command[k-1] == 41:
            end = 40
        while time.time() - t < 0.2:
            data = self._serial_port.read()
            k += 1
            if len(datas) == 4:
                if datas[-2] == 0x01 and datas[-1] == 0x05:
                    end = 7
                datas += data
                
            elif len(datas) == end:
                datas += data
                break
            elif len(datas) > 4:
                datas += data
            
            elif len(datas) >= 2:
                data_len = struct.unpack("b", data)[0]
                if command[-1] == 29 or command[-1] == 41 or data_len == command[k-1]:
                    datas += data
                else:
                    datas = b''
                    k = 0
                    pre = 0
            elif data == b"\xfe":
                if datas == b'':
                    datas += data
                    if k != 1:
                        k = 1
                    pre = k
                else:
                    if k - 1 == pre:
                        datas += data
                    else:
                        datas = b"\xfe"
                        k = 1
                        pre = 0
        else:
            datas = b''
        self.log.debug("_read: {}".format([hex(data) for data in datas]))
        return datas
            
        
        
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
        has_reply = kwargs.get("has_reply", None)
        real_command = self._process_data_command(genre, self.__class__.__name__, args)
        command = [
            ProtocolCode.HEADER.value,
            ProtocolCode.HEADER.value,
        ]
        if isinstance(genre, list):
            for data in genre:
                command.append(data)
        else:
            command.append(genre)
        command.append(real_command)
        command = self._flatten(command)
        if genre == ProtocolCode.SET_LED.value:
            command.append(sum(command[2:]) & 0xff)
        elif genre == ProtocolCode.GET_FIRMWARE_VERSION.value:
            command.append(4)
        elif genre == ProtocolCode.GET_MOTORS_CURRENT.value:
            command.append(5)
        elif genre == ProtocolCode.GET_BATTERY_INFO.value:
            command.append(6)
        else:
            # del command[2]
            command.append(sum(command[2:]) & 0xff)
        self._write(command)
        if has_reply:
            data = self._read(command)
            if data:
                if genre in [ProtocolCode.GET_FIRMWARE_VERSION.value]:
                    return self._int2coord(data[4])
                elif genre == ProtocolCode.GET_MOTORS_CURRENT.value:
                    return self._decode_int16(data[4:6])
                elif genre == ProtocolCode.GET_BATTERY_INFO.value:
                    res = []
                    byte_1 = bin(data[4])[2:]
                    res =[]
                    while len(byte_1) != 6:
                        byte_1 = "0"+byte_1
                    res.append(byte_1)
                    res.append(self._int2coord(data[5]))
                    res.append(self._int2coord(data[6]))
                    if byte_1[0] == "0":
                        res[-1] = 0
                    elif byte_1[1] == "0":
                        res[1] = 0
                    return res
                return data[4]
            # print(res)
        return None
    
    def set_led(self, mode, R, G, B):
        """Set up LED lights

        Args:
            mode (int): 1 - Set LED light color. 2 - Set the LED light to blink
            R (int): 0 ~ 255
            G (int): 0 ~ 255
            B (int): 0 ~ 255
        """
        calibration_parameters(class_name = self.__class__.__name__, rgb=[R,G,B], led_mode=mode)
        return self._mesg(ProtocolCode.SET_LED.value, mode, R, G, B)
    
    def get_firmware_version(self):
        """Get firmware version number
        """
        return self._mesg(ProtocolCode.GET_FIRMWARE_VERSION.value, has_reply = True)
    
    def get_motors_current(self):
        """Get the total current of the motor
        """
        return self._mesg(ProtocolCode.GET_MOTORS_CURRENT.value, has_reply = True)
    
    def get_battery_info(self):
        """Read battery information
        
        Return:
            list : [battery_data, battery_1_voltage, battery_2_voltage].
                battery_data:
                    A string of length 6, represented from left to right: 
                    bit5, bit4, bit3, bit2, bit1, bit0.

                    bit5 : Battery 2 is inserted into the interface 1 means inserted, 0 is not inserted.
                    bit4 : Battery 1 is inserted into the interface, 1 means inserted, 0 is not inserted.
                    bit3 : The adapter is plugged into the interface 1 means plugged in, 0 not plugged in.
                    bit2 : The charging pile is inserted into the interface, 1 means plugged in, 0 is not plugged in.
                    bit1 : Battery 2 charging light 0 means off, 1 means on.
                    bit0 : Battery 1 charging light, 0 means off, 1 means on.
                battery_1_voltage : Battery 1 voltage in volts.
                battery_2_voltage : Battery 2 voltage in volts.
        """
        return self._mesg(ProtocolCode.GET_BATTERY_INFO.value, has_reply = True)
    
    # def move_control(self, direction_1, direction_2, direction_3):
    #     """Control the car to rotate forward, backward, left, right and forward/counterclockwise

    #     Args:
    #         direction_1 (int): Control forward or backward: 0 ~ 127 is backward, 129 ~ 255 is forward, 128 is stop.
    #         direction_2 (int): control left and right movement: 0 ~ 127 is right, 129 ~ 255 is left, 128 is stop.
    #         direction_3 (int): control rotation: 0 ~ 127 is clockwise, 129 ~ 255 is counterclockwise, 128 is stop.
    #     """
    #     calibration_parameters(class_name = self.__class__.__name__, direction_1=direction_1, direction_2=direction_2,direction_3=direction_3)
    #     return self._mesg(direction_1, direction_2, direction_3)
    
    def go_ahead(self, go_speed, timeout=5):
        """Control the car to move forward. Send control commands every 100ms. with a default motion time of 5 seconds.

        Args:
            go_speed (int): 1 ~ 127 is forward.
            timeout (int): default 5 s.
        """
        # go_speed (int): 129 ~ 255 is forward
        calibration_parameters(class_name = self.__class__.__name__, data=go_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128+go_speed, 128, 128)
            time.sleep(0.1)
        self.stop()
        
    def retreat(self, back_speed, timeout=5):
        """Control the car back. Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            back_speed (int): 1 ~ 127 is backward
            timeout (int): default 5 s.
        """
        calibration_parameters(class_name = self.__class__.__name__, data=back_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128-back_speed, 128, 128)
            time.sleep(0.1)
        self.stop()
        
    def pan_left(self, pan_left_speed, timeout=5):
        """Control the car to pan to the left. Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            pan_left_speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        # pan_left_speed (int): 129 ~ 255
        calibration_parameters(class_name = self.__class__.__name__, data=pan_left_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128, 128+pan_left_speed, 128)
            time.sleep(0.1)
        self.stop()
        
    def pan_right(self, pan_right_speed, timeout=5):
        """Control the car to pan to the right. Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            pan_right_speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        calibration_parameters(class_name = self.__class__.__name__, pan_right_speed=pan_right_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128, 128-pan_right_speed, 128)
            time.sleep(0.1)
        self.stop()
        
    def clockwise_rotation(self, rotate_right_speed, timeout=5):
        """Control the car to rotate clockwise. Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            clockwise_rotation_speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        calibration_parameters(class_name = self.__class__.__name__, rotate_right_speed=rotate_right_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128, 128, 128-rotate_right_speed)
            time.sleep(0.1)
        self.stop()
        
        
    def counterclockwise_rotation(self, rotate_left_speed, timeout=5):
        """Control the car to rotate counterclockwise. Send control commands every 100ms. with a default motion time of 5 seconds

        Args:
            clockwise_rotation_speed (int): 1 ~ 127
            timeout (int): default 5 s.
        """
        calibration_parameters(class_name = self.__class__.__name__, rotate_left_speed=rotate_left_speed)
        t = time.time()
        while time.time() - t < timeout:
            self._mesg(128, 128, 128+rotate_left_speed)
            time.sleep(0.1)
        self.stop()
        
    def stop(self):
        """stop motion
        """
        self._mesg(128, 128, 128)
        
    def get_mcu_info(self, version=1.0):
        """"""
        if version == 1.0:
            data_len = 29
        else:
            data_len = 41
        datas = self._read([0xfe, 0xfe, data_len])
        res = []
        index = 2
        while index < len(datas) - 1:
            if index < 5:
                res.append(datas[index])
                index+=1
            elif index < 17 or (index >= 20 and index < 28):
                res.append(self._decode_int16(datas[index:index+2]))
                index+=2
            elif index >= 28 and index <= 32:
                res.append(self._int2angle(self._decode_int16(datas[index:index+2])))
                index+=2
            elif index == 17:
                byte_1 = bin(datas[index])[2:]
                while len(byte_1) != 6:
                        byte_1 = "0"+byte_1
                res.append(byte_1)
                index+=1
                
            elif index < 20 or (index > 32 and index < 41):
                res.append(self._int2coord(datas[index]))
                index+=1
                
        return res
    
    def restore(self):
        """Motor stall recovery"""
        self._mesg(ProtocolCode.RESTORE.value, 1)
        
    def set_gyro_state(self, state=1):
        """Set gyroscope calibration status (save after power failure)

        Args:
            state (int, optional): 1 - open. 0 - close. Defaults to 1.
        """
        self._mesg(ProtocolCode.SET_GYRO_STATE.value, state)
        
    def get_gyro_state(self):
        """Get gyroscope calibration status

        Return:
            1 - open
            0 - close
        """
        return self._mesg(ProtocolCode.GET_GYRO_STATE.value, has_reply = True)
    
    def get_modified_version(self):
        return self._mesg(ProtocolCode.GET_MODIFIED_VERSION.value, has_reply = True)
    
    # def get_battery_voltage(self, num=1):
    #     """Get battery voltage

    #     Args:
    #         num (int, optional): Battery ID number. Defaults to 1.
    #     """
    #     mcu_data = self.get_mcu_info()
    #     return self._mesg(ProtocolCode.GET_BATTERY_INFO.value, has_reply = True)