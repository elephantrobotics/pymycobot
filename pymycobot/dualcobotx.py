# coding=utf-8

import time
import threading

from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class DualMercury(CommandGenerator):
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
        super(DualMercury, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()
        

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
                data = self._read(genre)
                if genre == ProtocolCode.SET_SSID_PWD:
                    return None
                res = self._process_received(data, genre)
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
                ]:
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [
                    ProtocolCode.GET_COORDS,
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
                    if 1 not in res[1:]:
                        return res[0]
                    else:
                        for i in range(1, len(res)):
                            if res[i] == 1:
                                r.append(i)
                    return r
                elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.GET_SOLUTION_ANGLES]:
                        return self._int2angle(res[0])
                else:
                    return res
            return None

    def set_solution_angles(self, angle, speed):
        """Set zero space deflection angle value

        Args:
            angle: Angle of joint 1. The angle range is -90 ~ 90
            speed: 1 - 100.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, speed=speed, solution_angle=angle
        )
        return self._mesg(
            ProtocolCode.COBOTX_SET_SOLUTION_ANGLES, [self._angle2int(angle)], speed
        )

    def get_solution_angles(self):
        """Get zero space deflection angle value"""
        return self._mesg(ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, has_reply=True)

    def write_move_c(self, transpoint, endpoint, speed):
        """_summary_

        Args:
            transpoint (list): Arc passing point coordinates
            endpoint (list): Arc end point coordinates
            speed (int): 1 ~ 100
        """
        start = []
        end = []
        for index in range(6):
            if index < 3:
                start.append(self._coord2int(transpoint[index]))
                end.append(self._coord2int(endpoint[index]))
            else:
                start.append(self._angle2int(transpoint[index]))
                end.append(self._angle2int(endpoint[index]))
        return self._mesg(ProtocolCode.WRITE_MOVE_C, start, end, speed)

    def focus_all_servos(self):
        """Lock all joints"""
        return self._mesg(ProtocolCode.FOCUS_ALL_SERVOS)

    def go_zero(self):
        """Control the machine to return to the zero position.
        
        Return:
            1 : All motors return to zero position.
            list : Motor with corresponding ID failed to return to zero.
        """
        return self._mesg(ProtocolCode.GO_ZERO, has_reply=True)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 7 or 11 ~ 13.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id, has_reply=True)

    def is_gone_zero(self):
        """Check if it has returned to zero

        Return:
            1 : Return to zero successfully.
            0 : Returning to zero.
        """
        return self._mesg(ProtocolCode.COBOTX_IS_GO_ZERO, has_reply=True)

    def set_encoder(self, joint_id, encoder):
        """Set a single joint rotation to the specified potential value.

        Args:
            joint_id (int): Joint id 1 - 7.
            encoder: The value of the set encoder.
        """
        # TODO Mercury此接口待定
        # self.calibration_parameters(
        #     class_name=self.__class__.__name__, id=joint_id, encoder=encoder
        # )
        return self._mesg(ProtocolCode.SET_ENCODER, joint_id, [encoder])
    

    def servo_restore(self, joint_id):
        """Abnormal recovery of joints

        Args:
            joint_id (int): Joint ID.
                arm : 1 ~ 7 
                waist : 13
                All joints: 254
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, servo_restore=joint_id
        )
        self._mesg(ProtocolCode.SERVO_RESTORE, joint_id)

    def close(self):
        self._serial_port.close()
        
    def open(self):
        self._serial_port.open()

    def sync_send_angles(self, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached
            
        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(degrees, speed)
        while time.time() - t < timeout:
            f = self.is_in_position(degrees, 0)
            if f == 1:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, coords, speed, mode=None, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached
            
        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular（default）, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(coords, speed, mode)
        while time.time() - t < timeout:
            if self.is_in_position(coords, 1) == 1:
                break
            time.sleep(0.1)
        return self
