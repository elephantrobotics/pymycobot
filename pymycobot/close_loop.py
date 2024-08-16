# coding=utf-8
import time
import struct

from pymycobot.error import calibration_parameters
from pymycobot.generate import CommandGenerator
from pymycobot.common import ProtocolCode, write, read

class CloseLoop(CommandGenerator):
    _write = write
    _read = read
    def __init__(self, debug=False):
        super(CloseLoop, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        self.is_stop = False
        self.write_command = []
        self.read_command = []
        self.no_return = False
        
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
        real_command, has_reply = super(CloseLoop, self)._mesg(genre, *args, **kwargs)
        no_return = kwargs.get("no_return", False)
        if no_return:
            self.no_return = True
        t = kwargs.get("asyn_mode", None)
        if t:
            self.no_return = True
        elif t == False:
            self.no_return = False
        # is_in_position = False
        with self.lock:
            self.write_command.append(genre)
            if self.__class__.__name__ in ["Pro630Client", "Pro400Client"]:
                self._write(self._flatten(real_command), "socket")
                time.sleep(0.01)
            else:
                self._write(self._flatten(real_command))
                
        close_loop = [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]
        if genre in close_loop and self.no_return == False:
            has_reply = True
        if genre == ProtocolCode.SEND_ANGLES and no_return:
            has_reply = False   
        if has_reply:
            t = time.time()
            wait_time = 0.1   
            if genre == ProtocolCode.POWER_ON:
                wait_time = 8
            elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.RELEASE_ALL_SERVOS, ProtocolCode.FOCUS_ALL_SERVOS,
                        ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP]:
                wait_time = 3
            # elif genre in close_loop:
            #     wait_time = 300
            #     is_in_position = True
            need_break = False
            data = None
            while True and time.time() - t < wait_time:
                for v in self.read_command:
                    # if is_in_position and v == b'\xfe\xfe\x04[\x01\r\x87' and genre in close_loop:
                    #     need_break = True
                    #     with self.lock:
                    #         self.read_command.remove(v)
                    #         self.write_command.remove(genre)
                    #         return 1
                    # elif genre == v[3]:
                    if genre == v[3]:
                        need_break = True
                        data = v
                        with self.lock:
                            self.read_command.remove(v)
                            self.write_command.remove(genre)
                        break
                if need_break:
                    break
                time.sleep(0.01)
            if data is None:
                return data
            data = bytearray(data)
            data_len = data[2] - 3
            # unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]
            if genre == ProtocolCode.GET_BASIC_INPUT:
                data_pos = 5
                data_len -= 1
                if self.__class__.__name__ == "Pro630Client":
                    data_len += 1
            else:
                data_pos = 4
            
            valid_data = data[data_pos : data_pos + data_len]
            return (valid_data, data_len)
        return None
        
    def _process_received(self, data):
        if not data:
            return []
        elif data == b'\xfe\xfe\x04[\x01\r\x87':
            return data
        
        data = bytearray(data)
        data_len = len(data)
        # Get valid header: 0xfe0xfe
        header_i, header_j = 0, 1
        while header_j < data_len - 4:
            if self._is_frame_header(data, header_i, header_j):
                cmd_id = data[header_i + 3]
                if cmd_id in self.write_command:
                    break
            header_i += 1
            header_j += 1
        else:
            return []
        return data
        
    def read_thread(self, method=None):
        while True:
            
            datas = b""
            data_len = -1
            k = 0
            pre = 0
            t = time.time()
            wait_time = 0.1   
            if method is not None:
                try:
                    wait_time = 0.1
                    self.sock.settimeout(wait_time)
                    data = self.sock.recv(1024)
                    
                    if isinstance(data, str):
                        datas = bytearray()
                        for i in data:   
                            datas += hex(ord(i))
                except:
                    data = b""
                
                if data != b'':
                    if self.check_python_version() == 2:
                        command_log = ""
                        for d in data:
                            command_log += hex(ord(d))[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                        # self.log.debug("_read: {}".format([hex(ord(d)) for d in data]))
                    else:
                        command_log = ""
                        for d in data:
                            command_log += hex(d)[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                    res = self._process_received(data)
                    if res != []:
                        with self.lock:
                            self.read_command.append(res)
            else:
                while True and time.time() - t < wait_time:
                    if self._serial_port.inWaiting() > 0:
                        data = self._serial_port.read()
                        k += 1
                        if data_len == 3:
                            datas += data
                            crc = self._serial_port.read(2)
                            if self.crc_check(datas) == [v for v in crc]:
                                datas+=crc
                                break
                        if data_len == 1 and data == b"\xfa":
                            datas += data
                            # if [i for i in datas] == command:
                            #     datas = b''
                            #     data_len = -1
                            #     k = 0
                            #     pre = 0
                            #     continue
                            # break
                        elif len(datas) == 2:
                            data_len = struct.unpack("b", data)[0]
                            datas += data
                        elif len(datas) > 2 and data_len > 0:
                            datas += data
                            data_len -= 1
                        elif data == b"\xfe":
                            if datas == b"":
                                datas += data
                                pre = k
                            else:
                                if k - 1 == pre:
                                    datas += data
                                else:
                                    datas = b"\xfe"
                                    pre = k
                else:
                    datas = b''
                if datas:
                    res = self._process_received(datas)
                    if res != [] and res[3] == 0x5b:
                        continue
                    if self.check_python_version() == 2:
                        command_log = ""
                        for d in datas:
                            command_log += hex(ord(d))[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                    else:
                        command_log = ""
                        for d in datas:
                            command_log += hex(d)[2:] + " "
                        self.log.debug("_read : {}".format(command_log))
                    if res != []:
                        with self.lock:
                            self.read_command.append(res)
                # return datas
                
    def bytes4_to_int(self, bytes4):
        i = 0
        res = []
        data_len = len(bytes4)
        while i < data_len:
            if self.check_python_version() == 2:
                byte_value = 0
                data = bytes4[i:i+4]
                for b in data:
                    byte_value = byte_value * 256 + b
                
                if byte_value & (1 << 31):
                    byte_value -= 1 << (32)
            else:
                byte_value = int.from_bytes(bytes4[i:i+4], byteorder='big', signed=True)
            i+=4
            res.append(byte_value)
        return res
    
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

    def go_home(self, robot, speed=20):
        """Control the machine to return to the zero position.
        
        Args:
            robot (int): 
                1 - Mercury A1 
                2 - Mercury B1 or X1
            speed (int): 1 ~ 100
        Return:
            1 : All motors return to zero position.
            0 : failed.
        """
        if robot == 1:
            return self.sync_send_angles([0, 0, 0, 0, 0, 90, 0], speed)
        else:
            self.send_angle(11, 0, speed)
            self.send_angle(12, 0, speed)
            self.send_angle(13, 0, speed)
            return self.sync_send_angles([0, 0, 0, 0, 0, 90, 0], speed)

    def get_angle(self, joint_id):
        """Get single joint angle

        Args:
            joint_id (int): 1 ~ 7 or 11 ~ 13.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, id=joint_id)
        return self._mesg(ProtocolCode.COBOTX_GET_ANGLE, joint_id, has_reply=True)
    
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
                return 1
            time.sleep(0.1)
        return 0

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
                return 1
            time.sleep(0.1)
        return 0
        
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
    
    def drag_teach_save(self):
        """Start recording the dragging teaching point. In order to show the best sports effect, the recording time should not exceed 90 seconds."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_SAVE)
    
    def drag_teach_execute(self):
        """Start dragging the teaching point and only execute it once."""
        return self._mesg(ProtocolCode.MERCURY_DRAG_TECH_EXECUTE)
    
    def drag_teach_pause(self):
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
        # self.is_stop = True
        # self.write_command.remove()
        return self._mesg(ProtocolCode.STOP, has_reply=True)
    
    def get_robot_type(self):
        """Get robot type
        """
        return self._mesg(ProtocolCode.GET_ROBOT_ID, has_reply=True)
        
        
    def get_zero_pos(self):
        """Read the zero encoder value

        Returns:
            list: The values of the zero encoders of the seven joints
        """
        return self._mesg(ProtocolCode.GET_ZERO_POS, has_reply=True)
    
    def is_init_calibration(self):
        """Check if the robot is initialized for calibration

        Returns:
            bool: True if the robot is initialized for calibration, False otherwise
        """
        return self._mesg(ProtocolCode.IS_INIT_CALIBRATION, has_reply=True)
    
    def set_break(self, joint_id, value):
        """Set break point

        Args:
            joint_id: int. joint id 1 - 7
            value: int. 0 - disable, 1 - enable
            
        Return:
            0 : failed
            1 : success 
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id=joint_id, value=value)
        return self._mesg(ProtocolCode.SET_BREAK, joint_id, value, has_reply=True)
    
    def over_limit_return_zero(self):
        """Return to zero when the joint is over the limit
        """
        return self._mesg(ProtocolCode.OVER_LIMIT_RETURN_ZERO)
    
    def jog_increment_angle(self, joint_id, increment, speed):
        """angle step mode

        Args:
            joint_id: Joint id 1 - 7.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id = joint_id, speed = speed)
        return self._mesg(ProtocolCode.JOG_INCREMENT, joint_id, [self._angle2int(increment)], speed)
    
    def jog_increment_coord(self, coord_id, increment, speed):
        """coord step mode

        Args:
            coord_id: axis id 1 - 6.
            increment: 
            speed: int (1 - 100)
        """
        self.calibration_parameters(class_name = self.__class__.__name__, id = coord_id, speed = speed)
        value = self._coord2int(increment) if coord_id <= 3 else self._angle2int(increment)
        return self._mesg(ProtocolCode.JOG_INCREMENT_COORD, coord_id, [value], speed)
    
    def drag_teach_clean(self):
        """clear sample
        """
        return self._mesg(ProtocolCode.MERCURY_DRAG_TEACH_CLEAN)
    
    def get_comm_error_counts(self, joint_id, _type):
        """Read the number of communication exceptions

        Args:
            joint_id (int): joint ID
            _type (int): Error type to be read, 1 ~ 4.
                1-The number of exceptions sent by the joint
                2-The number of exceptions obtained by the joint
                3-The number of exceptions sent by the end
                4-The number of exceptions read by the end
        """
        return self._mesg(ProtocolCode.MERCURY_ERROR_COUNTS, joint_id, _type, has_reply=True)
    
    def set_pos_over_shoot(self, value):
        """Set position deviation value

        Args:
            value (_type_): _description_
        """
        return self._mesg(ProtocolCode.MERCURY_SET_POS_OVER_SHOOT, [value*100])
        
    def get_pos_over_shoot(self):
        """Get position deviation value
        """
        return self._mesg(ProtocolCode.MERCURY_GET_POS_OVER_SHOOT, has_reply=True)
    
    def stop(self, deceleration=True):
        """Robot stops moving

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - Stop completion
        """
        if deceleration:
            return self._mesg(ProtocolCode.STOP, has_reply=True)
        else:
            return self._mesg(ProtocolCode.STOP, 1, has_reply=True)
        
    def pause(self, deceleration=False):
        """Robot pauses movement

        Args:
            deceleration (bool, optional): Whether to slow down and stop. Defaults to False.

        Returns:
            int: 1 - pause completion
        """
        if deceleration:
            return self._mesg(ProtocolCode.PAUSE, has_reply=True)
        else:
            return self._mesg(ProtocolCode.PAUSE, 1, has_reply=True)
        
    def get_modified_version(self):
        return self._mesg(ProtocolCode.ROBOT_VERSION, has_reply=True)
    
    def get_pos_over(self):
        return self._mesg(ProtocolCode.GET_POS_OVER, has_reply=True)
    
    def clear_encoders_error(self):
        return self._mesg(ProtocolCode.CLEAR_ENCODERS_ERROR)
    
    def get_down_encoders(self):
        return self._mesg(ProtocolCode.GET_DOWN_ENCODERS, has_reply=True)
    
    def set_control_mode(self, mode):
        """Set robot motion mode

        Args:
            mode (int): 0 - location mode, 1 - torque mode

        """
        return self._mesg(ProtocolCode.SET_CONTROL_MODE, mode)
    
    def get_control_mode(self):
        """Get robot motion mode

        Returns:
            int: 0 - location mode, 1 - torque mode
        """
        return self._mesg(ProtocolCode.GET_CONTROL_MODE, has_reply=True)
    
    def set_collision_mode(self, mode):
        """Set collision detection mode

        Args:
            mode (int): 0 - disable, 1 - enable

        """
        return self._mesg(ProtocolCode.SET_COLLISION_MODE, mode)
    
    def set_collision_threshold(self, joint_id, value=100):
        """Set joint collision threshold

        Args:
            joint_id (int): joint ID， range 1 ~ 7
            value (int): Collision threshold, range is 50 ~ 250, default is 100, the smaller the value, the easier it is to trigger a collision
        """
        return self._mesg(ProtocolCode.SET_COLLISION_THRESHOLD, joint_id, value)
    
    def get_collision_threshold(self):
        """Get joint collision threshold
        """
        return self._mesg(ProtocolCode.GET_COLLISION_THRESHOLD, has_reply=True)
    
    def set_torque_comp(self, joint_id, value=100):
        """Set joint torque compensation

        Args:
            joint_id (int): joint ID， range 1 ~ 7
            value (int): Compensation value, range is 0 ~ 250, default is 100, The smaller the value, the harder it is to drag the joint
        """
        return self._mesg(ProtocolCode.SET_TORQUE_COMP, joint_id, value)
    
    def get_torque_comp(self):
        """Get joint torque compensation
        """
        return self._mesg(ProtocolCode.GET_TORQUE_COMP, has_reply=True)
    
    def power_on_only(self):
        """Only turn on the power
        """
        return self._mesg(ProtocolCode.POWER_ON_ONLY, has_reply=True)
    
    def get_vr_mode(self):
        """Check if the robot is in VR mode
        """
        return self._mesg(ProtocolCode.GET_VR_MODE, has_reply=True)
    
    def set_vr_mode(self, mode):
        """Set VR mode

        Args:
            mode (int): 0 - open, 1 - close
        """
        return self._mesg(ProtocolCode.SET_VR_MODE, mode)
    
    def get_model_direction(self):
        """Get the direction of the robot model
        """
        return self._mesg(ProtocolCode.GET_MODEL_DIRECTION, has_reply=True)
    
    def set_model_direction(self, id, direction):
        """Set the direction of the robot model

        Args:
            id (int): joint ID, 1 ~ 7.
            direction (int): 0 - forward, 1 - backward
        """
        return self._mesg(ProtocolCode.SET_MODEL_DIRECTION, id, direction)
    
    def get_filter_len(self, rank):
        """Get the filter length
        
        Args:
            rank (int): 
                1 : Drag teaching sampling filter
                2 : Drag teaching execution filter
                3 : Joint velocity fusion filter
                4 : Coordinate velocity fusion filter
                5 : Drag teaching sampling period
        """
        return self._mesg(ProtocolCode.GET_FILTER_LEN, rank, has_reply=True)
    
    def set_filter_len(self, rank, value):
        """Set the filter length

        Args:
            rank (int): 
                1 : Drag teaching sampling filter
                2 : Drag teaching execution filter
                3 : Joint velocity fusion filter
                4 : Coordinate velocity fusion filter
                5 : Drag teaching sampling period
            value (int): Filter length, range is 1 ~ 100
        """
        return self._mesg(ProtocolCode.SET_FILTER_LEN, rank, value)