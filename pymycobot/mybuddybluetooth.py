# coding: utf-8

import threading

from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters
from pymycobot.bluet import BluetoothConnection


class MyBuddyBlueTooth(MyBuddyCommandGenerator):
    """MyBuddy bluetooth API"""
    _write = write
    def __init__(self, bt_address=None, port = 10):
        """When bt_address is the default value of None, enter the Bluetooth search mode. There is a default Bluetooth search time of 5 seconds"""
        super(MyBuddyBlueTooth).__init__()
        self.ble = BluetoothConnection(bt_address, port)
        self.sock = self.ble.connect_target_device()
        self.lock = threading.Lock()
        
    def connect(self, serialport="/dev/ttyAMA0", baudrate="1000000", timeout='0.2'):
        """Connect the robot arm through the serial port and baud rate
        
        Args:
            serialport: (str) default /dev/ttyAMA0
            baudrate: default 1000000
            timeout: default 0.1
        
        """
        self.rasp = True
        self._write(serialport, "socket")
        self._write(baudrate, "socket")
        self._write(timeout, "socket")
        # self._write([254, 254, 1, 2, 32, 32], "socket")
        # self._write([254, 254, 1, 2, 32, 32], "socket")
        # self._write([254, 254, 1, 2, 32, 32], "socket")
        
        # self._write(timeout, "socket")
        # self._write(timeout, "socket")
        
        
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
        real_command, has_reply = super(
            MyBuddyBlueTooth, self)._mesg(genre, *args, **kwargs)
        with self.lock:
            data = self._write(self._flatten(real_command), "socket")

            if data:
                res = self._process_received(data, genre, arm=12)
                if res == []:
                    return None
                if genre in [
                    ProtocolCode.ROBOT_VERSION,
                    ProtocolCode.SOFTWARE_VERSION,
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
                    ProtocolCode.SetHTSGripperTorque,
                    ProtocolCode.GetHTSGripperTorque,
                    ProtocolCode.GetGripperProtectCurrent,
                    ProtocolCode.InitGripper,
                    ProtocolCode.SET_FOUR_PIECES_ZERO
                    # ProtocolCode.GET_SERVO_CURRENTS
                ]:
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [ProtocolCode.GET_ANGLE]:
                    return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_COORD]:
                    if real_command[5] < 4:
                        if real_command[2] == 3:
                            return self._int2angle(res[0]) if res else None
                        return self._int2coord(res[0]) if res else None
                    else:
                        return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_ALL_BASE_COORDS, ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE, ProtocolCode.GET_BASE_COORDS, ProtocolCode.GET_BASE_COORD, ProtocolCode.BASE_TO_SINGLE_COORDS]:
                    if res:
                        r = [] 
                        for idx in range(3):
                            r.append(self._int2coord(res[idx]))
                        for idx in range(3, 6):
                            r.append(self._int2angle(res[idx]))
                        if len(res) == 12:
                            r1 = []
                            for idx in range(6, 9):
                                r1.append(self._int2coord(res[idx]))
                            for idx in range(9, 12):
                                r1.append(self._int2angle(res[idx]))
                            return [r, r1]
                        return r
                    else:
                        return res
                elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
                    return self._int2coord(res[0])
                elif genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.COLLISION]:
                    return [self._int2coord(angle) for angle in res]
                else:
                    return res
            return None
        
    def close(self):
        self._write("close","socket")
        self.sock.close()
        