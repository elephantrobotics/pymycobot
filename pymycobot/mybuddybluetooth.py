# coding: utf-8
import bluetooth

from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters
from pymycobot.bluet import BluetoothConnection


class MyBuddyBlueTooth(MyBuddyCommandGenerator):
    """MyBuddy bluetooth API"""
    def __init__(self):
        """There is a default Bluetooth search time of 5 seconds"""
        super(MyBuddyBlueTooth).__init__()
        self.ble = BluetoothConnection()
        self.ble_obj = self.ble.connect_target_device()
        
    def __read(self):
        while True:
            data = self.ble_obj.recv(1024)
            if data:
                return data
        
        
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
        self.ble_obj.write(bytes(self._flatten(real_command)))

        if has_reply:
            data = self.__read()
            res = self._process_received(data, genre)
            if genre in [
                ProtocolCode.ROBOT_VERSION,
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
                ProtocolCode.GET_JOINT_MIN_ANGLE,
                ProtocolCode.GET_JOINT_MAX_ANGLE
            ]:
                return self._process_single(res)
            elif genre in [ProtocolCode.GET_ANGLES]:
                return [self._int2angle(angle) for angle in res]
            elif genre in [ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE]:
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
            else:
                return res
        return None
        
        