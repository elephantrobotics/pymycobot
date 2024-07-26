# coding=utf-8

import time
import threading

from pymycobot.close_loop import CloseLoop
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode


class Pro630(CloseLoop):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(Pro630, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial
        # import RPi.GPIO as GPIO
        # self.power_control_1 = 3
        # self.power_control_2 = 4
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setwarnings(False)
        # GPIO.setup(self.power_control_1, GPIO.IN)
        # GPIO.setup(self.power_control_2, GPIO.OUT)
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()
        self.has_reply_command = []
        self.is_stop = False
        self.read_threading = threading.Thread(target=self.read_thread)
        self.read_threading.daemon = True
        self.read_threading.start()
        
    def _mesg(self, genre, *args, **kwargs):
        read_data = super(Pro630, self)._mesg(genre, *args, **kwargs)
        if read_data is None:
            return None
        elif read_data == 1:
            return 1
        valid_data, data_len = read_data
        res = []
        if data_len in [8, 12, 14, 16, 26, 60]:
            if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1,n):
                    res.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                res = self.bytes4_to_int(valid_data)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    res.append(valid_data[i])
            else:
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
                res = self.bytes4_to_int(valid_data)
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
        elif data_len == 24:
            res = self.bytes4_to_int(valid_data)
        elif data_len == 40:
            i = 0
            while i < data_len:
                if i < 28:
                    res += self.bytes4_to_int(valid_data)
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
        elif data_len == 6:
            for i in valid_data:
                res.append(i)
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
            ProtocolCode.GET_CONTROL_MODE,
            ProtocolCode.GET_VR_MODE,
            ProtocolCode.GET_FILTER_LEN,
            ProtocolCode.IS_SERVO_ENABLE,
            ProtocolCode.GET_POS_SWITCH
        ]:
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
        elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, ProtocolCode.GET_POS_OVER]:
                return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            if len(res) == 23:
                i = 9
                for i in range(9, len(res)):
                    if res[i] != 0:
                        data = bin(res[i])[2:]
                        res[i] = []
                        while len(data) != 16:
                            data = "0"+data
                        for j in range(16):
                            if data[j] != "0":
                                res[i].append(15-j)
                return res
            else:
                for i in range(10, len(res)):
                    if res[i] != 0:
                        data = bin(res[i])[2:]
                        res[i] = []
                        while len(data) != 16:
                            data = "0"+data
                        for j in range(16):
                            if data[j] != "0":
                                res[i].append(15-j)
                return res
        else:
            return res

    def open(self):
        self._serial_port.open()
        
    def close(self):
        self._serial_port.close()
        
    def power_on(self, delay=2):
        # import RPi.GPIO as GPIO
        # GPIO.output(self.power_control_2, GPIO.HIGH)
        # time.sleep(delay)
        return super(Pro630, self).power_on()
     
    def power_off(self):
        # import RPi.GPIO as GPIO
        res = super(Pro630, self).power_off()
        # GPIO.output(self.power_control_2, GPIO.LOW)
        return res
    
    # def power_on_only(self):
        # import RPi.GPIO as GPIO
        # GPIO.output(self.power_control_2, GPIO.HIGH)
        
    # def set_basic_output(self, pin_no, pin_signal):
    #     """Set basic output.IO low-level output high-level, high-level output high resistance state

    #     Args:
    #         pin_no: pin port number. range 1 ~ 6
    #         pin_signal: 0 / 1
    #     """
    #     import RPi.GPIO as GPIO
    #     if pin_no == 1:
    #         pin_no = 17
    #     elif pin_no == 2:
    #         pin_no = 27
    #     elif pin_no == 3:
    #         pin_no = 22
    #     elif pin_no == 4:
    #         pin_no = 5
    #     elif pin_no == 5:
    #         pin_no = 6
    #     elif pin_no == 6:
    #         pin_no = 19
    #     GPIO.setup(pin_no, GPIO.OUT)
    #     GPIO.output(pin_no, pin_signal)
        
    # def get_basic_input(self, pin_no):
    #     """Get basic input.

    #     Args:
    #         pin_no: pin port number. range 1 ~ 6
            
    #     Return:
    #         1 - high
    #         0 - low
    #     """
    #     import RPi.GPIO as GPIO
    #     if pin_no == 1: 
    #         pin_no = 26
    #     elif pin_no == 2:
    #         pin_no = 21
    #     elif pin_no == 3:
    #         pin_no = 20 #23
    #     elif pin_no == 4:
    #         pin_no = 16
    #     elif pin_no == 5:
    #         pin_no = 24
    #     elif pin_no == 6:
    #         pin_no = 23
    #     GPIO.setup(pin_no, GPIO.IN)
    #     return GPIO.input(pin_no)
        
    def send_angles_sync(self, angles, speed):
        self.calibration_parameters(class_name = self.__class__.__name__, angles=angles, speed=speed)
        angles = [self._angle2int(angle) for angle in angles]
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, no_return=True)
    
    def set_pos_switch(self, mode):
        """Set position switch mode.

        Args:
            mode: 0 - switch off, 1 - switch on
        """
        if mode == 0:
            return self._mesg(ProtocolCode.SET_POS_SWITCH, mode, asyn_mode=True)
        return self._mesg(ProtocolCode.SET_POS_SWITCH, mode,asyn_mode=False)
    
    def get_pos_switch(self):
        """Get position switch mode.

        Return:
            1 - switch on, 0 - switch off
        """
        return self._mesg(ProtocolCode.GET_POS_SWITCH, has_reply=True)