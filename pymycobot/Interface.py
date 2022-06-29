# coding=utf-8
from pymycobot.common import ProtocolCode
from pymycobot.generate import MyCobotCommandGenerator


class MyBuddyCommandGenerator(MyCobotCommandGenerator):
    """MyBuddy Python API"""

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
        command_data = self._process_data_command(args)

        if genre == 178:
            # 修改wifi端口
            command_data = self._encode_int16(command_data)

        LEN = len(command_data) + 1
        check_digit = sum(command_data[1:]) + genre
        if check_digit >= 256:
            check_digit %= 256
        command = [
            ProtocolCode.HEADER,
            ProtocolCode.HEADER,
            command_data[0],
            LEN,
            genre,
            command_data[1:],
            check_digit,
        ]
        # print(command)
        real_command = self._flatten(command)
        has_reply = kwargs.get("has_reply", False)
        return real_command, has_reply

    # System status
    def get_robot_version(self, id):
        """Get cobot version
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.ROBOT_VERSION, id, has_reply=True)

    def get_system_version(self, id):
        """Get cobot version
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.SOFTWARE_VERSION, id, has_reply=True)

    def get_robot_id(self, id):
        """Detect this robot id
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.GET_ROBOT_ID, id, has_reply=True)

    def set_robot_id(self, id, new_id):
        """Set this robot id
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            new_id: 1 - 253
        """
        return self._mesg(ProtocolCode.SET_ROBOT_ID, id, new_id)

    # Overall status
    def power_on(self, id=0):
        """Open communication with Atom.
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.POWER_ON, id)

    def power_off(self, id=0):
        """Close communication with Atom.
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.POWER_OFF, id)

    def is_power_on(self, id=0):
        """Adjust robot arm status

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            
        Return:
            1 - power on
            0 - power off
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_POWER_ON, id, has_reply=True)

    def release_all_servos(self, id=0):
        """Robot turns off torque output
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.RELEASE_ALL_SERVOS, id)

    def is_controller_connected(self, id=0):
        """Wether connected with Atom.
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.IS_CONTROLLER_CONNECTED, id, has_reply=True)

    def read_next_error(self, id = 0):
        """Robot Error Detection
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        self._mesg(ProtocolCode.READ_NEXT_ERROR, id, has_reply=True)

    def set_free_mode(self, id):
        """set free mode

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
        """
        return self._mesg(ProtocolCode.SET_FREE_MODE, id)

    def is_free_mode(self, id):
        """Check if it is free mode

        Args:
            id: 0/1/2/3 (ALL/L/R/W)

        Return: 
            0/1
        """
        return self._process_single(
            self._mesg(ProtocolCode.IS_FREE_MODE, id, has_reply=True)
        )

    def get_angles(self, id):
        """ Get the degree of all joints.

        Args:
            id: 1/2/3 (L/R/W)
            
        Return:
            list: A float list of all degree.
        """
        return self._mesg(ProtocolCode.GET_ANGLES, id, has_reply=True)

    def send_angle(self, id, joint, angle, speed):
        """Send one degree of joint to robot arm.

        Args:
            id: 1/2/3 (L/R/W)
            joint: 1 ~ 6
            angle: int
            speed: 1 ~ 100
        """
        return self._mesg(
            ProtocolCode.SEND_ANGLE, id, joint, [self._angle2int(angle)], speed
        )

    def send_angles(self, id, degrees, speed, mode):
        """Send all angles to the robotic arm

        Args:
            id: 1/2/3 (L/R/W).
            degrees: [angle_list]
            speed: 1 - 100
            mode: 0 - with interpolation 1 - No interpolation 2 - reserved.

        """
        degrees = [self._angle2int(degree) for degree in degrees]
        return self._mesg(ProtocolCode.SEND_ANGLES, id, degrees, speed, mode)

    def get_coords(self, id):
        """Get the coordinates of the robotic arm
        
        Args:
            id: 1/2/3 (L/R/W).
        """
        return self._mesg(ProtocolCode.GET_COORDS, id, has_reply=True)

    def send_coord(self, id, coord, data, speed):
        """Send a single coordinate to the robotic arm
        
        Args:
            id: 1/2/3 (L/R/W).
            coord: 1 ~ 6 (x/y/z/rx/ry/rz)
            data: int
            speed: 0 ~ 100
        """
        value = self._coord2int(data) if coord <= 3 else self._angle2int(data)
        return self._mesg(ProtocolCode.SEND_COORD, id, coord, [value], speed)

    def send_coords(self, id, coords, speed, mode):
        """Send all coords to robot arm.

        Args:
            id: 1/2/3 (L/R/W).
            coords: a list of coords value(List[float]), length 6, [x(mm), y, z, rx(angle), ry, rz]\n
            speed : (int) 0 ~ 100
            mode : (int) 0 - moveJ, 1 - moveL, 2 - moveC
        """
        # self.calibration_parameters(coords=coords, speed=speed)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SEND_COORDS, id, coord_list, speed, mode)

    def pause(self, id):
        """Pause movement

        Args:
            id: 0/1/2/3 (ALL/L/R/W).
        """
        return self._mesg(ProtocolCode.PAUSE, id)

    def is_paused(self, id):
        """Judge whether the manipulator pauses or not.

        Args:
            id: 0/1/2/3 (ALL/L/R/W).
            
        Return:
            1 - paused
            0 - not paused
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_PAUSED, id, has_reply=True)

    def resume(self, id):
        """Recovery movement

        Args:
            id: 0/1/2/3 (ALL/L/R/W).
        """
        return self._mesg(ProtocolCode.RESUME, id)

    def stop(self, id):
        """Stop moving

        Args:
            id: 0/1/2/3 (ALL/L/R/W).
        """
        return self._mesg(ProtocolCode.STOP, id)

    def is_in_position(self, id, mode, data):  # TODO 通信协议可能有问题，待完善
        """Judge whether in the position.

        Args:
            id: 0/1/2/3 (ALL/L/R/W).
            mode: 1 - coords, 0 - angles
            data: A data list, angles or coords, length 6.

        Return:
            1 - True
            0 - False
            -1 - Error
        """
        if mode == 1:
            self.calibration_parameters(coords=data)
            data_list = []
            for idx in range(3):
                data_list.append(self._coord2int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle2int(data[idx]))
        elif mode == 0:
            self.calibration_parameters(degrees=data)
            data_list = [self._angle2int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        return self._mesg(ProtocolCode.IS_IN_POSITION, data_list, id, has_reply=True)

    def is_moving(self, id):
        """Detect if the robot is moving
        
        Args:
            id: 0/1/2/3 (ALL/L/R/W).
            
        Return:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_MOVING, id, has_reply=True)

    def jog_angle(self, id, joint_id, direction, speed):
        """Jog control angle.

        Args:
            id: 1/2/3 (L/R/W).
            joint_id:int 1-6.\n
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        return self._mesg(ProtocolCode.JOG_ANGLE, id, joint_id, direction, speed)

    def jog_absolute(self, id, joint_id, angle, speed):
        """Absolute joint control

        Args:
            id: 1/2/3 (L/R/W).
            joint_id: int 1-6.
            angle: int
            speed: int (0 - 100)
        """
        return self._mesg(
            ProtocolCode.JOG_ABSOLUTE, id, joint_id, [self._angle2int(angle)], speed
        )

    def jog_coord(self, id, coord_id, direction, speed):
        """Jog control coord.

        Args:
            id: 1/2/3 (L/R/W).
            coord_id: int 1-6 (x/y/z/rx/ry/rz).
            direction: 0 - decrease, 1 - increase
            speed: int (0 - 100)
        """
        return self._mesg(ProtocolCode.JOG_COORD, id, coord_id, direction, speed)

    def jog_increment(self, id, joint_id, increment, speed):
        """step mode

        Args:
            id: 1/2/3 (L/R/W).
            joint_id: int 1-6.
            increment: # TODO 未注明
            speed: int (1 - 100)
        """
        return self._mesg(ProtocolCode.JOG_INCREMENT, id, joint_id, increment, speed)

    def jog_stop(self, id):
        """Stop jog moving
        
        Args:
            id: 1/2/3 (L/R/W).
        """
        return self._mesg(ProtocolCode.JOG_STOP, id)

    def set_encoder(self, id, joint_id, encoder, speed):
        """Set a single joint rotation to the specified potential value.

        Args:
            id: 1/2/3 (L/R/W).
            joint_id: 1 - 6.
            encoder: The value of the set encoder.
        """
        return self._mesg(ProtocolCode.SET_ENCODER, id, joint_id, [encoder], speed)

    def get_encoder(self, id, joint_id):
        """Obtain the specified joint potential value.

        Args:
            id: 1/2/3 (L/R/W).
            joint_id: (int) 1 ~ 6

        Returns:
            0 ~ 4096
        """
        return self._mesg(ProtocolCode.GET_ENCODER, id, joint_id, has_reply=True)

    def set_encoders(self, id, encoders, speed, mode):
        """Set the six joints of the manipulator to execute synchronously to the specified position.

        Args:
            id: 1/2/3 (L/R/W).
            encoders: A encoder list, length 6.
            speed: speed 1 ~ 100
            mode: 0 - with interpolation 1 - No interpolation 2 - reserved.
        """
        return self._mesg(ProtocolCode.SET_ENCODERS, id, encoders, speed, mode)

    def get_encoders(self, id):
        """Get the six joints of the manipulator

        Args:
            id: 1/2 (L/R).
            
        Return:
            The list of encoders
        """
        return self._mesg(ProtocolCode.GET_ENCODERS, id, has_reply=True)

    def get_speed(self, id):
        """Get speed

        Args:
            id: 1/2/3 (L/R/W).
            
        Return:
            int: speed
        """
        return self._mesg(ProtocolCode.GET_SPEED, id, has_reply=True)

    def set_speed(self, id, speed):
        """Set speed value

        Args:
            id: 1/2/3 (L/R/W)
            speed (int): 0 - 100
        """
        self.calibration_parameters(speed=speed)
        return self._mesg(ProtocolCode.SET_SPEED, id, speed)

    def get_acceleration(self, id):
        """Read acceleration during all moves
        
        Args:
            id: 1/2/3 (L/R/W)
        """
        return self._mesg(ProtocolCode.GET_ACCELERATION, id, has_reply=True)

    def set_acceleration(self, id, acc):
        """Read acceleration during all moves
        
        Args:
            id: 1/2/3 (L/R/W)
            acc: 1 - 100
        """
        return self._mesg(ProtocolCode.SET_ACCELERATION, id, acc)

    def get_joint_min_angle(self, id, joint_id):
        """Gets the minimum movement angle of the specified joint

        Args:
            id: 1/2/3 (L/R/W)
            joint_id: (int) 1 - 6

        Returns:
            angle value(float)
        """
        self.calibration_parameters(id=joint_id)
        return self._mesg(
            ProtocolCode.GET_JOINT_MIN_ANGLE, id, joint_id, has_reply=True
        )

    def get_joint_max_angle(self, id, joint_id):
        """Gets the maximum movement angle of the specified joint

        Args:
            id: 1/2/3 (L/R/W)
            joint_id: (int) 1 - 6

        Return:
            angle value(float)
        """
        self.calibration_parameters(id=joint_id)
        return self._mesg(
            ProtocolCode.GET_JOINT_MAX_ANGLE, id, joint_id, has_reply=True
        )

    def set_joint_max(self, id, joint_id, angle):
        """Set the joint maximum angle

        Args:
            id: 1/2/3 (L/R/W)
            joint_id: int 1-6.\n
            angle: 0 ~ 180
        """
        return self._mesg(ProtocolCode.SET_JOINT_MAX, id, joint_id, [angle])

    def set_joint_min(self, id, joint_id, angle):
        """Set the joint minimum angle

        Args:
            id: 1/2/3 (L/R/W)
            joint_id: int 1-6.\n
            angle: 0 ~ 180
        """
        return self._mesg(ProtocolCode.SET_JOINT_MIN, id, joint_id, [angle])

    def is_servo_enable(self, id, servo_id):
        """Determine whether all steering gears are connected

        Args:
            id: 1/2/3 (L/R/W)
            servo_id: (int) 1 ~ 6

        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_SERVO_ENABLE, id, servo_id, has_reply=True)

    def is_all_servo_enable(self, id):
        """Determine whether the specified steering gear is connected

        Args:
            id: 1/2/3 (L/R/W)
            
        Return:
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(ProtocolCode.IS_ALL_SERVO_ENABLE, id, has_reply=True)

    def set_servo_data(self, id, servo_no, data_id, value):
        """Set the data parameters of the specified address of the steering gear

        Args:
            id: 1/2/3 (L/R/W)
            servo_no: Serial number of articulated steering gear, 1 - 6.
            data_id: Data address.
            value: 0 - 4096
        """
        return self._mesg(ProtocolCode.SET_SERVO_DATA, id, servo_no, data_id, value)

    def get_servo_data(self, id, servo_no, data_id):
        """Read the data parameter of the specified address of the steering gear.

        Args:
            id: 1/2/3 (L/R/W)
            servo_no: Serial number of articulated steering gear, 1 - 6.
            data_id: Data address.

        Return:
            values (0 - 4096)
            0 - disable
            1 - enable
            -1 - error
        """
        return self._mesg(
            ProtocolCode.GET_SERVO_DATA, id, servo_no, data_id, has_reply=True
        )

    def set_servo_calibration(self, id, servo_no):
        """The current position of the calibration joint actuator is the angle zero point,
            and the corresponding potential value is 2048.

        Args:
            id: 1/2/3 (L/R/W)
            servo_no: Serial number of articulated steering gear, 1 - 6.
        """
        return self._mesg(ProtocolCode.SET_SERVO_CALIBRATION, id, servo_no)

    def joint_brake(self, id, joint_id):
        """Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

        Args:
            id: 1/2/3 (L/R/W)
            joint_id: 1 - 6
        """
        return self._mesg(ProtocolCode.JOINT_BRAKE, id, joint_id)

    def release_servo(self, id, servo_id):
        """Power off designated servo

        Args:
            id: 1/2/3 (L/R/W)
            servo_id: 1 - 6.\n
        """
        return self._mesg(ProtocolCode.RELEASE_SERVO, id, servo_id)

    def focus_servo(self, id, servo_id):
        """Power on designated servo

        Args:
            id: 1/2/3 (L/R/W)
            servo_id: 1 - 6\n
        """
        return self._mesg(ProtocolCode.FOCUS_SERVO, id, servo_id)

    # Atom IO
    

    def set_pin_mode(self, id, pin_no, pin_mode):
        """Set the state mode of the specified pin in atom.

        Args:
            id: 1/2 (L/R)
            pin_no   (int): pin number (1 - 5).
            pin_mode (int): 0 - input, 1 - output
        """
        return self._mesg(ProtocolCode.SET_PIN_MODE, id, pin_no, pin_mode)

    def set_digital_output(self, id, pin_no, pin_signal):
        """Set atom IO output level

        Args:
            id: 1/2 (L/R)
            pin_no     (int): 1 - 5
            pin_signal (int): 0 / 1
        """
        return self._mesg(ProtocolCode.SET_DIGITAL_OUTPUT, id, pin_no, pin_signal)

    def get_digital_input(self, id, pin_no):
        """singal value

        Args:
            id: 1/2 (L/R)
            pin_no     (int): 1 - 5
        """
        return self._mesg(ProtocolCode.GET_DIGITAL_INPUT, id, pin_no, has_reply=True)

    def set_pwm_output(self, id, channel, frequency, pin_val):
        """PWM control

        Args:
            id: 1/2 (L/R)
            channel (int): IO number (1 - 5).
            frequency (int): clock frequency (0/1: 0 - 1Mhz 1 - 10Mhz)
            pin_val (int): Duty cycle 0 ~ 100: 0 ~ 100%
        """
        return self._mesg(ProtocolCode.SET_PWM_OUTPUT, channel, [frequency], pin_val)

    def get_gripper_value(self, id):
        """Get the value of gripper.

        Args:
            id: 1/2 (L/R)
            
        Return: 
            gripper value (int)
        """
        return self._mesg(ProtocolCode.GET_GRIPPER_VALUE, id, has_reply=True)

    def set_gripper_state(self, id, flag):
        """Set gripper switch state

        Args:
            id: 1/2 (L/R)
            flag  (int): 0 - close, 1 - open
        """
        return self._mesg(ProtocolCode.SET_GRIPPER_STATE, id, flag)

    def set_gripper_value(self, id, value):
        """Set gripper value

        Args:
            id: 1/2 (L/R)
            value (int): 0 ~ 100
        """
        return self._mesg(ProtocolCode.SET_GRIPPER_VALUE, id, value)

    def set_gripper_calibration(self, id):
        """Set the current position to zero, set current position value is `2048`.
        
        Args:
            id: 1/2 (L/R)
        """
        return self._mesg(ProtocolCode.SET_GRIPPER_CALIBRATION, id)

    def is_gripper_moving(self, id):
        """Judge whether the gripper is moving or not

        Args:
            id: 1/2 (L/R)
            
        Returns:
            0 - not moving
            1 - is moving
            -1 - error data
        """
        return self._mesg(ProtocolCode.IS_GRIPPER_MOVING, id, has_reply=True)

    def set_color(self, id, r=0, g=0, b=0):
        """Set the light color on the top of the robot arm.

        Args:
            id: 1/2 (L/R)
            r (int): 0 ~ 255
            g (int): 0 ~ 255
            b (int): 0 ~ 255

        """
        self.calibration_parameters(rgb=[r, g, b])
        return self._mesg(ProtocolCode.SET_COLOR, id, r, g, b)

    def set_tool_reference(self, id, coords):
        """Set tool coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
            coords: a list of coords value(List[float]), length 6. [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_TOOL_REFERENCE, id, coord_list)

    def get_tool_reference(self, id):
        """Get tool coordinate system
        
        Args:
            id: 0/1/2 (ALL/L/R)
        """
        return self._mesg(ProtocolCode.GET_TOOL_REFERENCE, id, has_reply=True)

    def set_world_reference(self, id, coords):
        """Set the world coordinate system
        
        Args:
            id: 0/1/2 (ALL/L/R)
            coords: a list of coords value(List[float]), length 6 [x(mm), y, z, rx(angle), ry, rz]
        """
        self.calibration_parameters(coords=coords)
        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord2int(coords[idx]))
        for angle in coords[3:]:
            coord_list.append(self._angle2int(angle))
        return self._mesg(ProtocolCode.SET_WORLD_REFERENCE, id, coord_list)

    def get_world_reference(self, id):
        """Get the world coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
        """
        return self._mesg(ProtocolCode.GET_WORLD_REFERENCE, id, has_reply=True)

    def set_reference_frame(self, id, rftype):
        """Set the base coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
            rftype: 0 - base 1 - tool.
        """
        return self._mesg(ProtocolCode.SET_REFERENCE_FRAME, id, rftype)

    def get_reference_frame(self, id):
        """Get the base coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
            
        Return: 
            0 - base 1 - tool.
        """
        return self._mesg(ProtocolCode.GET_REFERENCE_FRAME, id, has_reply=True)

    def set_movement_type(self, id, move_type):
        """Set movement type

        Args:
            id: 0/1/2 (ALL/L/R)
            move_type: 1 - movel, 0 - moveJ
        """
        return self._mesg(ProtocolCode.SET_MOVEMENT_TYPE, id, move_type)

    def get_movement_type(self, id):
        """Get movement type

        Args:
            id: 0/1/2 (ALL/L/R)
            
        Return: 
            1 - movel
            0 - moveJ
        """
        return self._mesg(ProtocolCode.GET_MOVEMENT_TYPE, id, has_reply=True)

    def set_end_type(self, id, end):
        """Set end coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
            end: 0 - flange, 1 - tool
        """
        return self._mesg(ProtocolCode.SET_END_TYPE, id, end)

    def get_end_type(self, id):
        """Get end coordinate system

        Args:
            id: 0/1/2 (ALL/L/R)
            
        Return: 
            0 - flange
            1 - tool
        """
        return self._mesg(ProtocolCode.GET_END_TYPE, id, has_reply=True)

    def set_joint_current(self, id, joint_id, current):
        """Set Collision Current

        Args:
            id: 0/1/2 (ALL/L/R)
            joint_id: 1 - 6
            current: current value
        """
        return self._mesg(ProtocolCode.SET_JOINT_CURRENT, id, joint_id, current)

    def get_joint_current(self, id, joint_id):
        """Get Collision Current

        Args:
            id: 0/1/2 (ALL/L/R)
            joint_id: 1 - 6
        """
        return self._mesg(ProtocolCode.GET_JOINT_CURRENT, id, joint_id)

    # Basic
    def set_basic_mode(self, pin_no, pin_mode):
        """Set base IO, input or output mode

        Args:
            pin_no: 1 - 5
            pin_mode: 0 - input 1 - output
        """
        return self._mesg(ProtocolCode.SET_BASIC_OUTPUT, 0, pin_no, pin_mode)

    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output.

        Args:
            pin_no: pin port number (0 - 20).
            pin_signal: 0 / 1 (0 - low, 1 - high)
        """
        return self._mesg(ProtocolCode.GET_BASIC_INPUT, 0, pin_no, pin_signal)

    def get_basic_input(self, pin_no):
        """Get basic input for M5 version.

        Args:
            pin_no: (int) pin port number (0 - 20).
        """
        return self._mesg(ProtocolCode.GET_BASE_INPUT, 0, pin_no, has_reply=True)

    def get_plan_speed(self, id = 0):
        """Get planning speed

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            
        Return: 
            [movel planning speed, movej planning speed].
        """
        return self._mesg(ProtocolCode.GET_PLAN_SPEED, id, has_reply=True)

    def get_plan_acceleration(self, id = 0):
        """Get planning acceleration

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            
        Return: 
            [movel planning acceleration, movej planning acceleration].
        """
        return self._mesg(ProtocolCode.GET_PLAN_ACCELERATION, id, has_reply=True)

    def set_plan_speed(self, id, speed):
        """Set planning speed

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            speed (int): (0 ~ 100).
        """
        return self._mesg(ProtocolCode.SET_PLAN_SPEED, id, speed)

    def set_plan_acceleration(self, id, acceleration):
        """Set planning acceleration

        Args:
            id: 0/1/2/3 (ALL/L/R/W)
            acceleration (int): (0 ~ 100).
        """
        return self._mesg(
            ProtocolCode.SET_PLAN_ACCELERATION, id, acceleration
        )

    def get_servo_currents(self, id):
        """Get joint current

        Args:
            id: 1/2/3 (L/R/W)
            
        Return: 
            value mA
        """
        return self._mesg(ProtocolCode.GET_SERVO_VOLTAGES, id, has_reply=True)

    def get_servo_voltages(self, id):
        """Get joint voltages

        Args:
            id: 1/2/3 (L/R/W)
            
        Return: 
            volts < 24 V
        """
        return self._mesg(ProtocolCode.GET_SERVO_STATUS, id, has_reply=True)

    def get_servo_status(self, id):
        """Get joint status

        Args:
            id: 1/2/3 (L/R/W)
            
        Return: 
            [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error
        """
        return self._mesg(ProtocolCode.GET_SERVO_TEMPS, id, has_reply=True)

    def get_servo_temps(self, id):
        """Get joint temperature

        Args:
            id: 1/2/3 (L/R/W)
        """
        return self._mesg(0xE6, id, has_reply=True)
    
    
    # def init_iic(self):
    #     from smbus2 import SMBus
    #     i2c = SMBus(1)   # 1 代表 /dev/i2c-1
