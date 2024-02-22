# coding=utf-8

import time
import socket
import threading

from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MercurySocket(CommandGenerator):
    _write = write
    _read = read
    def __init__(self, ip, netport=9000, debug=False):
        """
        Args:
            ip: Server ip
            netport: Server port(default 9000)
        """
        super(MercurySocket, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.sock = self.connect_socket()
        self.lock = threading.Lock()

    def connect_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.SERVER_IP, self.SERVER_PORT))
        return sock
        

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
        real_command, has_reply = super(MercurySocket, self)._mesg(genre, *args, **kwargs)
        with self.lock:
            self._write(self._flatten(real_command), "socket")

            if has_reply:
                data = self._read(genre, _class=self.__class__.__name__, method='socket')
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
                    ProtocolCode.GET_ERROR_DETECT_MODE
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
                    return res
            return None
    
    def open(self):
        self._serial_port.open()
        
    def close(self):
        self._serial_port.close()

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
        return self._mesg(ProtocolCode.FOCUS_ALL_SERVOS, has_reply=True)

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
        
    def set_error_detect_mode(self, mode):
        """Set error detection mode. Turn off without saving, default to open state
        
        Return:
            mode : 0 - close 1 - open.
        """
        self.calibration_parameters(
            class_name=self.__class__.__name__, mode=mode
        )
        self._mesg(ProtocolCode.SET_ERROR_DETECT_MODE, mode)
        
    def get_error_detect_mode(self):
        """Set error detection mode"""
        return self._mesg(ProtocolCode.GET_ERROR_DETECT_MODE, has_reply=True)
    
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
        
    def get_base_coords(self):
        """get base coords"""
        return self._mesg(ProtocolCode.MERCURY_GET_BASE_COORDS, has_reply=True)
    
    def send_base_coord(self, axis, coord, speed):
        """_summary_

        Args:
            axis (_type_): _description_
            coord (_type_): _description_
            speed (_type_): _description_
        """
        if axis < 4:
            coord = self._coord2int(coord)
        else:
            coord = self._angle2int(coord)
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORD, axis, [coord], speed)
    
    def send_base_coords(self, coords, speed):
        """_summary_

        Args:
            coords (_type_): _description_
            speed (_type_): _description_
        """
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.MERCURY_SET_BASE_COORDS, coord_list, speed)
    
    def jog_base_coord(self, axis, direction, speed):
        """_summary_

        Args:
            axis (_type_): _description_
            direction (_type_): _description_
            speed (_type_): _description_
        """
        return self._mesg(ProtocolCode.MERCURY_JOG_BASE_COORD, axis, direction, speed)
    
    def drag_tech_save(self):
        """Start recording the dragging teaching point. In order to show the best sports effect, the recording time should not exceed 90 seconds."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_SAVE)
    
    def drag_tech_execute(self):
        """Start dragging the teaching point and only execute it once."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE)
    
    def drag_tech_pause(self):
        """Pause recording of dragging teaching point"""
        self._mesg(ProtocolCode.MERCURY_DRAG_TECH_PAUSE)
        
    def is_gripper_moving(self, mode=None):
        """Judge whether the gripper is moving or not
        
        Args:
            mode: 1 - pro gripper(default)  2 - Parallel gripper

        Returns:
            0 - not moving
            1 - is moving
            -1- error data
        """
        if mode:
            return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, mode, has_reply=True)
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, has_reply=True)
    
    def set_gripper_enabled(self, value):
        """Pro adaptive gripper enable setting

        Args:
            value (int): 
                1 : enable
                0 : release
        """
        self.calibration_parameters(class_name=self.__class__.__name__, value=value)
        return self._mesg(ProtocolCode.SET_GRIPPER_ENABLED, value)
    
    def is_btn_clicked(self):
        """Check if the end button has been pressed.
        
        Return:
            1 : pressed.
            0 : not pressed.
        """
        return self._mesg(ProtocolCode.IS_BTN_CLICKED, has_reply=True)
        
    def tool_serial_restore(self):
        """485 factory reset
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_RESTORE)
    
    def tool_serial_ready(self):
        """Set up 485 communication
        
        Return:
            0 : not set
            1 : Setup completed
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_READY, has_reply=True)
    
    def tool_serial_available(self):
        """Read 485 buffer length
        
        Return:
            485 buffer length available for reading
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_AVAILABLE, has_reply=True)
    
    def tool_serial_read_data(self, data_len):
        """Read fixed length data. Before reading, read the buffer length first. After reading, the data will be cleared

        Args:
            data_len (int): The number of bytes to be read, range 1 ~ 45
        """
        self.calibration_parameters(class_name=self.__class__.__name__, data_len=data_len)
        return self._mesg(ProtocolCode.TOOL_SERIAL_READ_DATA, data_len, has_reply=True)
    
    def tool_serial_write_data(self, command):
        """End 485 sends data， Data length range is 1 ~ 45 bytes

        Args:
            command : data instructions
            
        Return:
            number of bytes received
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_WRITE_DATA, command, has_reply=True)
    
    def tool_serial_flush(self):
        """Clear 485 buffer
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_FLUSH)
    
    def tool_serial_peek(self):
        """View the first data in the buffer, the data will not be cleared
        
        Return:
            1 byte data
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_PEEK, has_reply=True)
    
    def tool_serial_set_baud(self, baud=115200):
        """Set 485 baud rate, default 115200

        Args:
            baud (int): baud rate
        """
        return self._mesg(ProtocolCode.TOOL_SERIAL_SET_BAUD, baud)
    
    def tool_serial_set_timeout(self, max_time):
        """Set 485 timeout in milliseconds, default 30ms

        Args:
            max_time (int): timeout
        """
        self.calibration_parameters(class_name=self.__class__.__name__, max_time=max_time)
        return self._mesg(ProtocolCode.TOOL_SERIAL_SET_TIME_OUT, max_time)

    def get_robot_status(self):
        return self._mesg(ProtocolCode.MERCURY_ROBOT_STATUS, has_reply=True)
    
    def power_on(self):
        """Open communication with Atom."""
        return self._mesg(ProtocolCode.POWER_ON, has_reply=True)

    def power_off(self):
        """Close communication with Atom."""
        return self._mesg(ProtocolCode.POWER_OFF, has_reply=True)
    
    def release_all_servos(self):
        """Relax all joints
        """
        return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS, has_reply=True)
    
    def focus_servo(self, servo_id):
        """Power on designated servo

        Args:
            servo_id: int. joint id 1 - 7
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.FOCUS_SERVO, servo_id, has_reply=True)
    
    def release_servo(self, servo_id):
        """Power off designated servo

        Args:
            servo_id: int. joint id 1 - 7
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=servo_id)
        return self._mesg(ProtocolCode.RELEASE_SERVO, servo_id, has_reply=True)
    
    def stop(self):
        """Stop moving"""
        return self._mesg(ProtocolCode.STOP, has_reply=True)

        