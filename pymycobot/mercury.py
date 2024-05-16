# coding=utf-8

import time
import threading

from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class Mercury(MercuryCommandGenerator):
    _write = write
    _read = read
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(Mercury, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()
        self.has_reply_command = []
        self.is_stop = False
        self.read_threading = threading.Thread(target=self.read_thread, daemon=True)
        self.read_threading.start()
        

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
        real_command, has_reply = super(Mercury, self)._mesg(genre, *args, **kwargs)
        is_in_position = False
        with self.lock:
            self.write_command.append(genre)
            self._write(self._flatten(real_command))
        if genre in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_ANGLE, ProtocolCode.JOG_COORD, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
            has_reply = True
        if has_reply:
            t = time.time()
            wait_time = 0.1   
            if genre == ProtocolCode.POWER_ON:
                wait_time = 8
            elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.RELEASE_ALL_SERVOS, ProtocolCode.FOCUS_ALL_SERVOS,
                        ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP]:
                wait_time = 3
            elif genre in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_ANGLE, ProtocolCode.JOG_COORD, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
                wait_time = 300
                is_in_position = True
            need_break = False
            while True and time.time() - t < wait_time:
                for data in self.read_command:
                    if is_in_position and data == b'\xfe\xfe\x04[\x01\r\x87':
                        need_break = True
                        with self.lock:
                            self.read_command.remove(data)
                        break
                    elif genre == data[3]:
                        need_break = True
                        with self.lock:
                            self.read_command.remove(data)
                        break
                if need_break:
                    break
                time.sleep(0.001)
            else:
                res = []
            if need_break:
                res = []
                data_len = data[2] - 3
                unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]
                if genre in unique_data:
                    data_pos = 5
                    data_len -= 1
                else:
                    data_pos = 4
                valid_data = data[data_pos : data_pos + data_len]
                if data_len in [6, 8, 12, 14, 16, 24, 26, 60]:
                    if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                        for v in valid_data:
                            res.append(v)
                    elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                        i = 0
                        while i < data_len:
                            byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
                            i+=4
                            res.append(byte_value)
                    for header_i in range(0, len(valid_data), 2):
                        one = valid_data[header_i : header_i + 2]
                        res.append(self._decode_int16(one))
                elif data_len == 2:
                    if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                        return [self._decode_int8(valid_data[1:2])]
                    elif genre in [ProtocolCode.GET_ERROR_INFO]:
                        return [self._decode_int8(valid_data[1:])]
                    res.append(self._decode_int16(valid_data))
                elif data_len == 3:
                    res.append(self._decode_int16(valid_data[1:]))
                elif data_len == 4:
                    if genre == ProtocolCode.COBOTX_GET_ANGLE:
                        byte_value = int.from_bytes(valid_data, byteorder='big', signed=True)
                        res.append(byte_value)
                    for i in range(1,4):
                        res.append(valid_data[i])
                elif data_len == 7:
                    error_list = [i for i in valid_data]
                    for i in error_list:
                        if i in range(16,23):
                            res.append(1)
                        elif i in range(23,29):
                            res.append(2)
                        elif i in range(32,112):
                            res.append(3)
                        else:
                            res.append(i)
                elif data_len == 28:
                    for i in range(0, data_len, 4):
                        byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
                        res.append(byte_value) 
                elif data_len == 40:
                    i = 0
                    while i < data_len:
                        if i < 28:
                            byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
                            res.append(byte_value) 
                            i+=4
                        else:
                            one = valid_data[i : i + 2]
                            res.append(self._decode_int16(one))
                            i+=2
                elif data_len == 30:
                    i = 0
                    res = []
                    while i < 30:
                        if i < 9 or i >= 23:
                            res.append(valid_data[i])
                            i+=1
                        elif i < 23:
                            one = valid_data[i : i + 2]
                            res.append(self._decode_int16(one))
                            i+=2
                elif data_len == 38:
                    i = 0
                    res = []
                    while i < data_len:
                        if i < 10 or i >= 30:
                            res.append(valid_data[i])
                            i+=1
                        elif i < 38:
                            one = valid_data[i : i + 2]
                            res.append(self._decode_int16(one))
                            i+=2
                elif data_len == 56:
                    for i in range(0, data_len, 8):
                        
                        byte_value = int.from_bytes(valid_data[i:i+4], byteorder='big', signed=True)
                        res.append(byte_value)
                else:
                    if genre in [
                        ProtocolCode.GET_SERVO_VOLTAGES,
                        ProtocolCode.GET_SERVO_STATUS,
                        ProtocolCode.GET_SERVO_TEMPS,
                    ]:
                        for i in range(data_len):
                            data1 = self._decode_int8(valid_data[i : i + 1])
                            res.append(0xFF & data1 if data1 < 0 else data1)
                    res.append(self._decode_int8(valid_data))
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
                    ProtocolCode.SET_SSID_PWD,
                    ProtocolCode.COBOTX_IS_GO_ZERO,
                    ProtocolCode.GET_ERROR_DETECT_MODE,
                    ProtocolCode.POWER_ON,
                    ProtocolCode.POWER_OFF,
                    ProtocolCode.RELEASE_ALL_SERVOS,
                    ProtocolCode.RELEASE_SERVO,
                    ProtocolCode.FOCUS_ALL_SERVOS,
                    ProtocolCode.FOCUS_SERVO,
                    ProtocolCode.STOP,
                    ProtocolCode.SET_BREAK,
                    ProtocolCode.IS_BTN_CLICKED,
                ] or (isinstance(res, list) and len(res) == 1):
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [
                    ProtocolCode.GET_COORDS,
                    ProtocolCode.MERCURY_GET_BASE_COORDS,
                    ProtocolCode.GET_TOOL_REFERENCE,
                    ProtocolCode.GET_WORLD_REFERENCE,
                ]:
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
                elif genre in [ProtocolCode.GET_BASIC_VERSION, ProtocolCode.SOFTWARE_VERSION, ProtocolCode.GET_ATOM_VERSION]:
                    return self._int2coord(self._process_single(res))
                elif genre in [
                    ProtocolCode.GET_JOINT_MAX_ANGLE,
                    ProtocolCode.GET_JOINT_MIN_ANGLE,
                ]:
                    return self._int2coord(res[0])
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
                elif genre == ProtocolCode.GO_ZERO:
                    r = []
                    if res:
                        if 1 not in res[1:]:
                            return res[0]
                        else:
                            for i in range(1, len(res)):
                                if res[i] == 1:
                                    r.append(i)
                    return r
                elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES]:
                        return self._int2angle(res[0])
                elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
                    same_error = []
                    if len(res) == 23:
                        index = 9
                    else:
                        index = 10
                    for i in range(10, len(res)):
                        if res[i] != 0:
                            data = bin(res[i])[2:]
                            res[i] = []
                            while len(data) != 16:
                                data = "0"+data
                            for j in range(16):
                                if data[j] != "0":
                                    error_id = 15-j
                                    if error_id in [0,5,6]:
                                        if error_id not in same_error:
                                            same_error.append(error_id)
                                            res[i].append(error_id)
                                    else:
                                        res[i].append(error_id)
                            if res[i] == []:
                                res[i] = 0
                    return res
                elif genre == ProtocolCode.GET_QUICK_INFO:
                    r = []
                    for index in range(len(res)):
                        if index < 7:
                            r.append(self._int2angle(res[index]))
                        elif index < 10:
                            r.append(self._int2coord(res[index]))
                        elif index < 13:
                            r.append(self._int2angle(res[index]))
                        else:
                            r.append(res[index])
                    return r
                else:
                    return res
            else:
                with self.lock:
                    self.write_command.remove(genre)
                return None
    
    def open(self):
        self._serial_port.open()
        
    def close(self):
        self._serial_port.close()
