# coding: utf-8
import time
from pymycobot.common import ProtocolCode
import math


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
                    return data[read:read + end]
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
        command = ProtocolCode.RELEASE_SERVOS + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def power_on(self):
        """Lock all joints."""
        command = ProtocolCode.LOCK_SERVOS + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def go_zero(self):
        """back to zero."""
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
        command = ProtocolCode.SLEEP_TIME + " S" + str(time) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def get_angles_info(self):
        """Get the current joint angle information."""
        command = ProtocolCode.GET_CURRENT_ANGLE_INFO + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("angle")

    def get_radians_info(self):
        """Get the current joint radian information."""
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
        command = ProtocolCode.GET_CURRENT_COORD_INFO + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("coord")

    def get_switch_state(self):
        """Get the current state of all home switches."""
        command = ProtocolCode.GET_BACK_ZERO_STATUS + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("endstop")

    def set_init_pose(self, x=None, y=None, z=None):
        """Set the current coords to zero."""
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
                x : -260 ~ 300 mm
                y : -300 ~ 300 mm
                z : -70 ~ 135 mm
            speed : (int) 0-200 mm/s
        """
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
                x : -260 ~ 300 mm
                y : -300 ~ 300 mm
                z : -70 ~ 135 mm
            speed : (int) 0-200 mm/s
        """

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
        if state:
            command = ProtocolCode.GPIO_CLOSE + ProtocolCode.END
        else:
            command = ProtocolCode.GPIO_ON + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def set_gripper_zero(self):
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
        command = ProtocolCode.GIRPPER_OPEN + "A" + str(state) + "F" + str(speed) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def set_gripper_release(self):
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
                3 : -5° ~ 60°
                4 : -180° ~ + 180°
            speed : (int) 0-200 mm/s
        """
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
                3 : -5° ~ 60°
                4 : -180° ~ + 180°
            speed : (int) 0-200 mm/s
        """
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
                rx : -2.97 ~ +2.97
                ry : 0.0 ~ 1.57
                rz : 0.0 ~ 1.31
            speed : (int) 0-200 mm/s
        """
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
        command = ProtocolCode.SET_JOG_STOP
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        self._respone()

    def play_gcode_file(self, filename=None):
        """Play the imported track file"""
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
        self._serial_port.close()

    def open(self):
        self._serial_port.open()

    def is_moving_end(self):
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

    def set_system_value(self, id, address, value):
        """_summary_

        Args:
            id (int): 4 or 7
            address (int): 0 ~ 69
            value (int): 
        """
        command = ProtocolCode.SET_SYSTEM_VALUE + " X{} ".format(id) + "Y{} ".format(address) + "Z{} ".format(
            value) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)

    def get_system_value(self, id, address):
        """_summary_

        Args:
            id (int): 4 or 7
            address (_type_): 0 ~ 69

        Returns:
            _type_: _description_
        """
        command = ProtocolCode.GET_SYSTEM_VALUE + " J{} ".format(id) + "S{} ".format(address) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        self._debug(command)
        return self._request("system")

