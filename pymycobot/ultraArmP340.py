# coding: utf-8
import time
from pymycobot.common import ProtocolCode
from pymycobot.error import calibration_parameters
import math
import threading


class ultraArmP340:
    def __init__(self, port, baudrate=115200, timeout=0.1, debug=False):

        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info, default: False
        """
        import serial

        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = True
        self._serial_port.dtr = True
        self._serial_port.open()
        self.debug = debug
        self.calibration_parameters = calibration_parameters
        self.lock = threading.Lock()
        time.sleep(1)

    def _respone(self):
        """Send data to the robot"""
        time.sleep(0.02)
        data = None
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
            queue_size = self._get_queue_size()
            if data != None:
                if 40 <= queue_size <= 90:
                    data = str(data.decode())
                    if self.debug:
                        print(data)
                elif 0 <= queue_size < 40:
                    data = str(data.decode())
                    if self.debug:
                        print(data)
                else:
                    qs = 0
                    while True:
                        try:
                            time.sleep(2)
                            qs = self._get_queue_size()
                            if self.debug:
                                print('respone_size1:', qs)
                            if qs < 40:
                                time.sleep(1)
                                qsize = self._get_queue_size()
                                print('respone_size2:', qsize)
                                break

                        except Exception as e:
                            print(e)

    def _request(self, flag=""):
        """Get data from the robot"""
        time.sleep(0.02)
        data = None
        count = 0
        while count < 5:
            if self._serial_port.inWaiting() > 0:
                data = self._serial_port.read(self._serial_port.inWaiting())
                data = str(data.decode())
                if self.debug:
                    print("\nreceived data:\n%s**********************\n" % data)
                if data.find("ERROR: COMMAND NOT RECOGNIZED") > -1:
                    flag = None

                if flag == "angle":
                    a = data[data.find("ANGLES"):]
                    astart = a.find("[")
                    aend = a.find("]")
                    if a != None and a != "":
                        try:
                            all = list(map(float, a[astart + 1: aend].split(",")))
                            return all
                        except Exception:
                            print("received angles is not completed! Retry receive...")
                            count += 1
                            continue

                elif flag == "coord":
                    c = data[data.find("COORDS"):]
                    cstart = c.find("[")
                    cend = c.find("]")
                    if c != None and c != "":
                        try:
                            all = list(map(float, c[cstart + 1: cend].split(",")))
                            return all
                        except Exception:
                            print("received coords is not completed! Retry receive...")
                            count += 1
                            continue

                elif flag == "endstop":
                    e = data[data.find("ENDSTOP"):]
                    eend = e.find("\n")
                    if e != None and e != "":
                        try:
                            all = e[:eend]
                            return all
                        except Exception:
                            print("received coords is not completed! Retry receive...")
                            count += 1
                            continue

                elif flag == "size":
                    try:
                        q = data[data.find("QUEUE SIZE"):]
                        qstart = q.find("[")
                        qend = q.find("]")
                        if q != None and q != "":
                            qsize = int(q[qstart + 1: qend])
                            return qsize
                    except Exception as e:
                        count += 1
                        continue
                elif flag == 'isStop':
                    if "Moving end" in data:
                        return 1
                    else:
                        return 0
                elif flag == 'gripper':
                    header = "GRIPPERANGLE["
                    read = data.find(header) + len(header)
                    # print(data[read:])
                    end = data[read:].find(']')
                    return data[read:read + end]
                elif flag == 'system':
                    header = "ReadSYS["
                    read = data.find(header) + len(header)
                    end = data[read:].find(']')
                    return int(data[read:read + end])
                elif flag == 'system_version':
                    start_idx = data.find("GetSystemVersion[")
                    if start_idx != -1:
                        version_str = data[start_idx + len("GetSystemVersion["):]
                        version_str = version_str.split(']')[0]
                        version = float(version_str.strip())
                        return version
                    else:
                        return None
                elif flag == 'modify_version':
                    start_idx = data.find("GetModifyVersion[")
                    if start_idx != -1:
                        version_str = data[start_idx + len("GetModifyVersion["):]
                        version_str = version_str.split(']')[0]
                        version = int(version_str.strip())
                        return version
                    else:
                        return None
                elif flag == None:
                    return 0

    def _debug(self, data):
        """whether show info."""
        if self.debug:
            print("\n***** Debug Info *****\nsend command: %s" % data)

    def _get_queue_size(self):
        """Get real queue_size from the robot."""
        command = "M120" + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("size")

    def release_all_servos(self):
        """relax all joints."""
        with self.lock:
            command = ProtocolCode.RELEASE_SERVOS + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def power_on(self):
        """Lock all joints."""
        with self.lock:
            command = ProtocolCode.LOCK_SERVOS + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def go_zero(self):
        """back to zero."""
        with self.lock:
            command = ProtocolCode.BACK_ZERO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            time.sleep(0.1)
            data = None
            while True:
                if self._serial_port.inWaiting() > 0:
                    data = self._serial_port.read(self._serial_port.inWaiting())
                    if data != None:
                        data = str(data.decode())
                        if data.find("ok") > 0:
                            if self.debug:
                                print(data)
                            break

    def sleep(self, wait_time):
        """Set wait time

        Args:
            wait_time (int): wait time 0 ~ 65535
        """
        self.calibration_parameters(class_name=self.__class__.__name__, wait_time=wait_time)
        with self.lock:
            command = ProtocolCode.SLEEP_TIME + " S" + str(wait_time) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def get_angles_info(self):
        """Get the current joint angle information."""
        with self.lock:
            command = ProtocolCode.GET_CURRENT_ANGLE_INFO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("angle")

    def get_radians_info(self):
        """Get the current joint radian information."""
        with self.lock:
            while True:
                angles = self.get_angles_info()
                if angles != None:
                    break
            radians = []
            for i in angles:
                radians.append(round(i * (math.pi / 180), 3))
            return radians

    def get_coords_info(self):
        """Get current Cartesian coordinate information."""
        with self.lock:
            command = ProtocolCode.GET_CURRENT_COORD_INFO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("coord")

    def get_switch_state(self):
        """Get the current state of all home switches."""
        with self.lock:
            command = ProtocolCode.GET_BACK_ZERO_STATUS + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("endstop")

    def set_init_pose(self, pose_coords):
        """Set the current coords to zero.

        Args:
            pose_coords (list[float]): [x, y, z] coordinate values.
        """
        self.calibration_parameters(class_name=self.__class__.__name__, pose_coords=pose_coords)
        with self.lock:
            command = ProtocolCode.SET_JOINT
            command += " x" + str(pose_coords[0])
            command += " y" + str(pose_coords[1])
            command += " z" + str(pose_coords[2])
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_coords(self, coords, speed):
        """Set all joints coords.

        Args:
            coords: a list of coords value(List[float]).
                x : -360 ~ 365.55 mm
                y : -365.55 ~ 365.55 mm
                z : -140 ~ 130 mm
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coords=coords, speed=speed)

        with self.lock:
            command = ProtocolCode.COORDS_SET
            command += " X" + str(coords[0])
            command += " Y" + str(coords[1])
            command += " Z" + str(coords[2])
            if len(coords) == 4:
                if coords[3] is not None:
                    command += " E" + str(coords[3])
            command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_coord(self, coord_id, coord, speed):
        """Set single coordinate.

        Args:
            coord_id (str): 'X', 'Y', 'Z', 'E'
            coord (float): coordinate value
            speed (int): movement speed (0-200 mm/s)
        """
        self.calibration_parameters(class_name=self.__class__.__name__, coord_id=coord_id, coord=coord, speed=speed)
        with self.lock:
            command = ProtocolCode.COORDS_SET
            command += f" {coord_id}{coord}"
            command += f" F{speed}"
            command += ProtocolCode.END

            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_mode(self, mode):
        """set coord mode.

        Args:
            mode:
                0 - Absolute Cartesian mode

                1 - Relative Cartesian mode
        """
        self.calibration_parameters(class_name=self.__class__.__name__, mode=mode)
        with self.lock:
            if mode == 0:
                command = ProtocolCode.ABS_CARTESIAN + ProtocolCode.END
            else:
                command = ProtocolCode.REL_CARTESIAN + ProtocolCode.END

            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_speed_mode(self, speed_mode):
        """set speed mode.

        Args:
            speed_mode:
                0 - Constant speed mode

                2 - Acceleration / deceleration mode
        """
        self.calibration_parameters(class_name=self.__class__.__name__, speed_mode=speed_mode)
        with self.lock:
            command = "M16 S" + str(speed_mode) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_pwm(self, p_value):
        """PWM control.

        Args:
            p_value (int) : Duty cycle 0 ~ 255; 128 means 50%
        """
        self.calibration_parameters(class_name=self.__class__.__name__, p_value=p_value)
        with self.lock:
            command = ProtocolCode.SET_PWM + "p" + str(p_value) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gpio_state(self, state):
        """Set gpio state.

        Args:
            state (int):
                1 - open
                0 - close
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            if state:
                command = ProtocolCode.GPIO_CLOSE + ProtocolCode.END
            else:
                command = ProtocolCode.GPIO_ON + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gripper_zero(self):
        """Set gripper zero."""
        with self.lock:
            command = ProtocolCode.GRIPPER_ZERO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gripper_state(self, gripper_value, gripper_speed):
        """Set gripper angle.

        Args:
            gripper_value (int): 0 - 100

            gripper_speed: 0 - 1500
        """
        self.calibration_parameters(class_name=self.__class__.__name__, gripper_value=gripper_value, gripper_speed=gripper_speed)
        with self.lock:
            command = ProtocolCode.GIRPPER_OPEN + "A" + str(gripper_value) + "F" + str(gripper_speed) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gripper_release(self):
        """Set gripper release."""
        with self.lock:
            command = ProtocolCode.GIRPPER_RELEASE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_fan_state(self, state):
        """Set fan state.

        Args:
            state (int):
                0 - close
                1 - open
        """
        self.calibration_parameters(class_name=self.__class__.__name__, state=state)
        with self.lock:
            if state:
                command = ProtocolCode.FAN_ON + ProtocolCode.END
            else:
                command = ProtocolCode.FAN_CLOSE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_angle(self, joint_id, angle, speed):
        """Set single angle.

        Args:
            joint_id (int) : joint (1/2/3/4)
            angle :
                1 : -150° ~ +170°
                2 : -20° ~ 90°
                3 : -5° ~ 110°
                4 : -179° ~ + 179°
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id, angle=angle, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_ANGLE
            command += " J" + str(joint_id)
            command += " A" + str(angle)
            command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_angles(self, angles, speed):
        """Set all joints angles.

        Args:
            angles: A list of angles value(List[float]).
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, angles=angles, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_ANGLES
            command += " X" + str(angles[0])
            command += " Y" + str(angles[1])
            command += " Z" + str(angles[2])
            if len(angles) == 4:
                command += " E" + str(angles[3])
            command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_radians(self, degrees, speed):
        """Set all joints radians

        Args:
            degrees: A list of radians value(List[float]).
                    1 : -2.6179 ~ 2.9670
                    2 : -0.3490 ~ 1.5707
                    3 : -0.0872 ~ 1.9198
                    4 : -3.1241 ~ + 3.1241
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, radians=degrees, speed=speed)
        with self.lock:
            degrees = [round(degree * (180 / math.pi), 2) for degree in degrees]
            self.set_angles(degrees, speed)

    def set_jog_angle(self, joint_id, direction, speed):
        """Start jog movement with angle

        Args:
            joint_id : joint(1/2/3)

            direction :
                1 : Negative motion
                0 : Positive motion
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, joint_id=joint_id, direction=direction, speed=speed)
        with self.lock:
            command = ProtocolCode.SET_JOG_ANGLE
            command += " J" + str(joint_id)
            command += " D" + str(direction)
            command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_jog_coord(self, axis_id, direction, speed):
        """Start jog movement with coord

        Args:
            axis_id(int) : axis 1-X, 2-Y, 3-Z

            direction:
                1 : Negative motion
                0 : Positive motion
            speed : (int) 0-200 mm/s
        """
        self.calibration_parameters(class_name=self.__class__.__name__, axis_id=axis_id, direction=direction,
                                    speed=speed)
        with self.lock:
            command = ProtocolCode.JOG_COORD_
            command += " J" + str(axis_id)
            command += " A" + str(direction)
            command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_jog_stop(self):
        """Stop jog movement"""
        with self.lock:
            command = ProtocolCode.SET_JOG_STOP
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def play_gcode_file(self, filename):
        """Play the imported track file

        Args:
            filename (str): Path to a G-code file (.gcode or .nc or .ngc)
        """

        self.calibration_parameters(class_name=self.__class__.__name__, filename=filename)
        X = []
        try:
            with open(filename) as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip("\n")
                    X.append(line)
        except Exception:
            print("There is no such file!")

        begin, end = 0, len(X)

        self.set_speed_mode(0)  # Constant speed mode

        with self.lock:
            for i in range(begin, end):
                command = X[i] + ProtocolCode.END
                self._serial_port.write(command.encode())
                self._serial_port.flush()
                time.sleep(0.02)

                self._debug(command)

                queue_size = self._request("size")
                if 0 <= queue_size <= 90:
                    try:
                        if self.debug:
                            print("queue_size1: %s \n" % queue_size)
                    except Exception as e:
                        print(e)
                else:
                    qs = 0
                    while True:
                        try:
                            qs = self._get_queue_size()
                            if self.debug:
                                print("queue_size2: %s \n" % qs)
                            if qs <= 40:
                                begin = i
                                break
                        except Exception:
                            print("Retry receive...")
                command = ""

        self.set_speed_mode(2)  # Acceleration / deceleration mode

    def close(self):
        with self.lock:
            self._serial_port.close()
        
    def open(self):
        with self.lock:
            self._serial_port.open()
        
    def is_moving_end(self):
        with self.lock:
            """Get the current state of all home switches."""
            command = ProtocolCode.IS_MOVING_END + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            return self._request("isStop")

    def sync(self):
        while self.is_moving_end() != 1:
            pass

    def get_gripper_angle(self):
        """Get gripper angle"""
        command = ProtocolCode.GET_GRIPPER_ANGLE + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("gripper")

    def set_system_value(self, id, address, value, mode=None):
        """Set system parameters

        Args:
            id (int): 4 or 7
            address (int): 7 ~ 69
            value (int):
            mode (int): 1 or 2, can be empty, default mode is 1
                1 - setting range is 0-255, address 21 (P value) can be used
                2 - setting value range is 0-65535, address 56 (setting position) can be used
        """
        if mode:
            self.calibration_parameters(class_name=self.__class__.__name__, id=id, address=address,
                                        system_value=value, system_mode=mode)
            command = ProtocolCode.SET_SYSTEM_VALUE + " X{} ".format(id) + "Y{} ".format(address) + "Z{} ".format(
                value) + "P{} ".format(mode) + ProtocolCode.END
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=id, address=address, system_value=value)
            command = ProtocolCode.SET_SYSTEM_VALUE + " X{} ".format(id) + "Y{} ".format(address) + "Z{} ".format(
                value) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)

    def get_system_value(self, id, address, mode=None):
        """Get system parameters

        Args:
            id (int): 4 or 7
            address (int): 0 ~ 69
            mode (int): 1 or 2, can be empty, default mode is 1
                1 - read range is 0-255, address 21 (P value) can be used
                2 - read value range is 0-65535, address 56 (read position) can be used

        Returns:
            int: 0 ~ 65535
        """
        if mode:
            self.calibration_parameters(class_name=self.__class__.__name__, id=id, get_address=address, system_mode=mode)
            command = ProtocolCode.GET_SYSTEM_VALUE + " J{} ".format(id) + "S{} ".format(address) + "P{} ".format(
                mode) + ProtocolCode.END
        else:
            self.calibration_parameters(class_name=self.__class__.__name__, id=id, get_address=address)
            command = ProtocolCode.GET_SYSTEM_VALUE + " J{} ".format(id) + "S{} ".format(address) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("system")

    def get_system_version(self):
        """
        Get system firmware version

        Returns:
            (float) Firmware version
        """
        command = ProtocolCode.GET_SYSTEM_VERSION + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request('system_version')

    def get_modify_version(self):
        """
        Get firmware modify version

        Returns:
            (int) modify version
        """
        command = ProtocolCode.GET_MODIFY_VERSION + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request('modify_version')
