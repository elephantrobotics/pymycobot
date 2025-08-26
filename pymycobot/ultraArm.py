# coding: utf-8
import time
from pymycobot.common import ProtocolCode
import math
import threading


class ultraArm:
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

    def sleep(self, time):
        with self.lock:
            command = ProtocolCode.SLEEP_TIME + " S" + str(time) + ProtocolCode.END
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

    def set_init_pose(self, x=None, y=None, z=None):
        """Set the current coords to zero."""
        with self.lock:
            command = ProtocolCode.SET_JOINT
            if x is not None:
                command += " x" + str(x)
            if y is not None:
                command += " y" + str(y)
            if z is not None:
                command += " z" + str(z)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_coords(self, degrees, speed=0):
        """Set all joints coords.

        Args:
            degrees: a list of coords value(List[float]).
                x : -360 ~ 365.55 mm
                y : -365.55 ~ 365.55 mm
                z : -140 ~ 130 mm
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            length = len(degrees)
            degrees = [degree for degree in degrees]
            command = ProtocolCode.COORDS_SET
            if degrees[0] is not None:
                command += " X" + str(degrees[0])
            if degrees[1] is not None:
                command += " Y" + str(degrees[1])
            if degrees[2] is not None:
                command += " Z" + str(degrees[2])
            if length == 4:
                if degrees[3] is not None:
                    command += " E" + str(degrees[3])
            if speed > 0:
                command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_coord(self, id=None, coord=None, speed=0):
        """Set single coord.

        Args:
            coord:
                x : -360 ~ 365.55 mm
                y : -365.55 ~ 365.55 mm
                z : -140 ~ 130 mm
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            command = ProtocolCode.COORDS_SET
            if id == "x" or id == "X":
                command += " X" + str(coord)
            if id == "y" or id == "Y":
                command += " Y" + str(coord)
            if id == "z" or id == "Z":
                command += " Z" + str(coord)
            if speed > 0:
                command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_mode(self, mode=None):
        """set coord mode.

        Args:
            mode:
                0 - Absolute Cartesian mode

                1 - Relative Cartesian mode
        """
        with self.lock:
            if mode == 0:
                command = ProtocolCode.ABS_CARTESIAN + ProtocolCode.END
            else:
                command = ProtocolCode.REL_CARTESIAN + ProtocolCode.END

            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_speed_mode(self, mode=None):
        """set speed mode.

        Args:
            mode:
                0 - Constant speed mode

                2 - Acceleration / deceleration mode
        """
        with self.lock:
            if mode != None:
                command = "M16 S" + str(mode) + ProtocolCode.END
            else:
                print("Please enter the correct value!")
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_pwm(self, p=None):
        """PWM control.

        Args:
            p (int) : Duty cycle 0 ~ 255; 128 means 50%
        """
        with self.lock:
            command = ProtocolCode.SET_PWM + "p" + str(p) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gpio_state(self, state):
        """Set gpio state.

        Args:
            state:
                1 - open
                0 - close
        """
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
        with self.lock:
            command = ProtocolCode.GRIPPER_ZERO + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gripper_state(self, state, speed):
        """Set gripper state.

        Args:
            state: 0 - 100
                0 - full close
                100 - full open
            speed: 0 - 1500
        """
        with self.lock:
            command = ProtocolCode.GIRPPER_OPEN + "A" + str(state) + "F" + str(speed) + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_gripper_release(self):
        with self.lock:
            command = ProtocolCode.GIRPPER_RELEASE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_fan_state(self, state):
        """Set fan state.

        Args:
            state:
                0 - close
                1 - open
        """
        with self.lock:
            if state:
                command = ProtocolCode.FAN_ON + ProtocolCode.END
            else:
                command = ProtocolCode.FAN_CLOSE + ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_angle(self, id=None, angle=None, speed=0):
        """Set single angle.

        Args:
            id : joint (1/2/3/4)

            angle :
                1 : -150° ~ +170°
                2 : -20° ~ 90°
                3 : -5° ~ 110°
                4 : -179° ~ + 179°
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            command = ProtocolCode.SET_ANGLE
            if id is not None:
                command += " j" + str(id)
            if angle is not None:
                command += " a" + str(angle)
            if speed > 0:
                command += " f" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_angles(self, degrees, speed=0):
        """Set all joints angles.

        Args:
            degrees: a list of angles value(List[float]).
            angle :
                1 : -150° ~ +170°
                2 : -20° ~ 90°
                3 : -5° ~ 110°
                4 : -179° ~ + 179°
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            length = len(degrees)
            degrees = [degree for degree in degrees]
            command = ProtocolCode.SET_ANGLES
            if degrees[0] is not None:
                command += " X" + str(degrees[0])
            if degrees[1] is not None:
                command += " Y" + str(degrees[1])
            if degrees[2] is not None:
                command += " Z" + str(degrees[2])
            if length == 4:
                if degrees[3] is not None:
                    command += " E" + str(degrees[3])
            if speed > 0:
                command += " F" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_radians(self, degrees, speed=0):
        """Set all joints radians

        Args:
            degrees: a list of radians value(List[float]).
            angle :
                1 : 2.6179 ~ 2.9670
                2 : -0.3490 ~ 1.5707
                3 : -0.0872 ~ 1.9198
                4 : -3.1241 ~ + 3.1241
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            degrees = [round(degree * (180 / math.pi), 2) for degree in degrees]
            self.set_angles(degrees, speed)

    def set_jog_angle(self, id=None, direction=None, speed=0):
        """Start jog movement with angle

        Args:
            id : joint(1/2/3)

            direction :
                0 : positive
                1 : negative
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            command = ProtocolCode.SET_JOG_ANGLE
            if id is not None:
                command += " j" + str(id)
            if direction is not None:
                command += " d" + str(direction)
            if speed > 0:
                command += " f" + str(speed)
            command += ProtocolCode.END
            self._serial_port.write(command.encode())
            self._serial_port.flush()
            self._debug(command)
            self._respone()

    def set_jog_coord(self, axis=None, direction=None, speed=0):
        """Start jog movement with coord

        Args:
            axis : 
            1 : x
            2 : y
            3 : z
            4 : θ

            direction:
                0 : positive
                1 : negative
            speed : (int) 0-200 mm/s
        """
        with self.lock:
            command = ProtocolCode.JOG_COORD_
            if axis is not None:
                command += " j" + str(axis)
            if direction is not None:
                command += " d" + str(direction)
            if speed > 0:
                command += " f" + str(speed)
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

    def play_gcode_file(self, filename=None):
        """Play the imported track file"""
        with self.lock:
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
        """
        
        """
        command = ProtocolCode.GET_GRIPPER_ANGLE + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("gripper")

    def set_system_value(self, id, address, value, mode=None):
        """_summary_

        Args:
            id (int): 4 or 7
            address (int): 7 ~ 69
            value (int):
            mode (int): 1 or 2, can be empty, default mode is 1
                1 - setting range is 0-255, address 21 (P value) can be used
                2 - setting value range is 0-65535, address 56 (setting position) can be used
        """
        if mode:
            command = ProtocolCode.SET_SYSTEM_VALUE + " X{} ".format(id) + "Y{} ".format(address) + "Z{} ".format(
                value) + "P{} ".format(mode) + ProtocolCode.END
        else:
            command = ProtocolCode.SET_SYSTEM_VALUE + " X{} ".format(id) + "Y{} ".format(address) + "Z{} ".format(
                value) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)

    def get_system_value(self, id, address, mode=None):
        """_summary_

        Args:
            id (int): 4 or 7
            address (_type_): 0 ~ 69
            mode (int): 1 or 2, can be empty, default mode is 1
                1 - read range is 0-255, address 21 (P value) can be used
                2 - read value range is 0-65535, address 56 (read position) can be used

        Returns:
            _type_: _description_
        """
        if mode:
            command = ProtocolCode.GET_SYSTEM_VALUE + " J{} ".format(id) + "S{} ".format(address) + "P{} ".format(
                mode) + ProtocolCode.END
        else:
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
