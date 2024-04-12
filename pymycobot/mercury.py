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
        with self.lock:
            self._write(self._flatten(real_command))

            if has_reply:
                data = self._read(genre, _class=self.__class__.__name__)
                if genre == ProtocolCode.SET_SSID_PWD:
                    return None
                res = self._process_received(data, genre, 14)
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
                    ProtocolCode.IS_BTN_CLICKED
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
                elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES]:
                        return self._int2angle(res[0])
                elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
                    if self._serial_port.port == "/dev/ttyTHS0":
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
            return None
    
    def open(self):
        self._serial_port.open()
        
    def close(self):
        self._serial_port.close()
