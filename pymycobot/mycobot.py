import sys
sys.path.append('.')
import time, serial, struct, math


class Command():
    #BASIC
    HEADER = 0xfe
    FOOTER = 0xfa

    # Overall status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    SET_FREE_MODE = 0x13

    # MDI MODE AND OPERATION
    GET_ANGLES = 0x20
    SEND_ANGLE = 0x21
    SEND_ANGLES = 0x22
    GET_COORDS = 0x23
    SEND_COORD = 0x24
    SEND_COORDS = 0x25
    PAUSE = 0x26
    IS_PAUSED = 0x27
    RESUME = 0x28
    STOP = 0x29
    IS_IN_POSITION = 0x2a
    IS_MOVING = 0x2b

    # JOG MODE AND OPERATION
    JOG_ANGLE = 0x30
    JOG_COORD = 0x32
    JOG_STOP = 0x34

    # RUNNING STATUS AND SETTINGS
    GET_SPEED = 0x40
    SET_SPEED = 0x41
    GET_JOINT_MIN_ANGLE = 0x4a
    GET_JOINT_MAX_ANGLE = 0x4b

    # SERVO CONTROL
    IS_SERVO_ENABLE = 0x50
    IS_ALL_SERVO_ENABLE = 0x51

    # ATOM IO
    SET_COLOR = 0x6a
    SET_CLAW = 0x66


class DataProcesser():
    # Functional approach
    def _encode_int8(self, data):
        return struct.pack('b', data)

    def _encode_int16(self, data):
        return list(struct.pack('>h', data))

    def _decode_int8(self, data):
        return struct.unpack('b', data)[0]

    def _decode_int16(self, data):
        return struct.unpack('>h', data)[0]

    def _angle_to_int(self, angle):
        return int(angle * 100)

    def _coord_to_int(self, coord):
        return int(coord * 10)

    def _int_to_angle(self, _int):
        return round(_int / 100.0, 3)

    def _int_to_coord(self, _int):
        return round(_int / 10.0, 2)

    def _flatten(self, _list):
        return sum(([x] if not isinstance(x, list) else self._flatten(x)
                    for x in _list), [])

    def _process_data(self, data, genre):
        if not data:
            return []

        if genre in [
                Command.SEND_ANGLES, Command.SEND_COORDS,
                Command.IS_IN_POSITION
        ]:
            _data_list = []
            for value in data[:6]:
                _data_list.extend(self._encode_int16((value)))
            for value in data[6:]:
                _data_list.append((value))
            return _data_list

        elif genre in [Command.SEND_ANGLE, Command.SEND_COORD]:
            return self._flatten([
                data[0],
                self._flatten(self._encode_int16(data[1])),
                data[2]
            ])

        elif genre in [Command.JOG_ANGLE, Command.JOG_COORD]:
            return self._flatten([
                data[0], data[1],
                data[2]
            ])

        else:
            return [data[0]]

    def _is_frame_header(self, data, pos):
        return data[pos] == Command.HEADER and data[pos + 1] == Command.HEADER

    def _process_recived(self, data, genre):
        if not data:
            return []

        data = bytearray(data)
        data_len = len(data)
        for idx in range(data_len - 1):
            if self._is_frame_header(data, idx):
                break
        else:
            return []

        data_len = data[idx + 2] - 2
        cmd_id = data[idx + 3]
        if cmd_id != genre:
            return []
        data_pos = idx + 4
        avild_data = (data[data_pos:data_pos + data_len])

        res = []
        if data_len == 12:
            for idx in range(0, len(avild_data), 2):
                a = avild_data[idx:idx + 2]
                res.append(self._decode_int16(a))
        elif data_len == 2:
            res.append(self._decode_int16(avild_data))
        else:
            res.append(self._decode_int8(avild_data))
        return res

    def _process_bool(self, data):
        return data[0] if data else -1


class MyCobot(DataProcesser):
    '''MyCobot Python API

    Possessed function:

        # Overall status
            power_on()
            power_off()
            is_power_on()
            set_free_mode()

        # MDI mode and operation
            get_angles()
            send_angle()
            send_angles()
            get_radians()
            send_radians()
            get_coords()
            send_coords()
            pause()
            resume()
            stop()
            is_paused()
            is_in_position()
            is_moving() x

        # JOG mode and operation
            jog_angle()
            jog_coord()
            jog_stop()

        # Running status and Settings
            get_speed()
            set_speed()
            get_joint_min_angle()
            get_joint_max_angle()

        # Servo control
            is_servo_enable()
            is_all_servo_enable()

        # Atom IO
            set_led_color()
            set_claw()
    '''

    def __init__(self, port, boudrate='115200', timeout=0.1):
        self._version = sys.version_info[:2][0]
        for _ in range(5):
            try:
                self._serial_port = serial.Serial(port,
                                                  boudrate,
                                                  timeout=timeout)
                break
            except Exception as e:
                print(e)
                time.sleep(5)
                continue
        else:
            print('Connect prot failed, eixt.')
            exit(0)

    def _write(self, command):
        self._serial_port.write(command)
        self._serial_port.flush()
        time.sleep(0.05)

    def _read(self, size=1024):
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
        else:
            data = None
        return data

    def __mesg(self, genre, data=None, has_reply=False):
        command_data = self._process_data(data, genre)
        LEN = len(command_data) + 2
        command = [
            Command.HEADER, Command.HEADER, LEN, genre, command_data,
            Command.FOOTER
        ]
        self._write(self._flatten(command))

        if has_reply:
            data = self._read()
            res = self._process_recived(data, genre)
            return res

    # Overall status
    def power_on(self):
        self.__mesg(Command.POWER_ON)

    def power_off(self):
        self.__mesg(Command.POWER_OFF)

    def is_power_on(self):
        '''Adjust robot arm status

        Return:
            1 : power on
            0 : power off
            -1: error data

        '''
        received = self.__mesg(Command.IS_POWER_ON, has_reply=True)
        return self._process_bool(received)

    def set_free_mode(self):
        # self._write('fefe0213fa')
        self.__mesg(Command.SET_FREE_MODE)

    # MDI mode and operation
    def get_angles(self):
        '''Get all angle return a list

        Return:
            data_list (list[angle...]):

        '''
        angles = self.__mesg(Command.GET_ANGLES, has_reply=True)
        return [self._int_to_angle(angle) for angle in angles]

    def send_angle(self, id, degree, speed):
        '''Send one angle

        Args:
            id (common.Angle):
            degree (float):
            speed (int): 0 ~100

        '''
        data = [id - 1, self._angle_to_int(degree), speed]
        self.__mesg(Command.SEND_ANGLE, data=data)

    def send_angles(self, degrees, speed):
        '''Send all angles

        Args:
            degrees (list): example [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            speed (int): 0 ~ 100

        '''
        degrees = [self._angle_to_int(degree) for degree in degrees]
        data = [degrees, speed]
        self.__mesg(Command.SEND_ANGLES, data=self._flatten(data))

    def get_radians(self):
        '''Get all angle return a list

        Return:
            data_list (list[radian...]):

        '''
        angles = self.__mesg(Command.GET_ANGLES, has_reply=True)
        return [
            round(self._int_to_angle(angle) * (math.pi / 180), 3)
            for angle in angles
        ]

    def send_radians(self, radians, speed):
        '''Send all angles

        Args:
            radians (list): example [0, 0, 0, 0, 0, 0]
            speed (int): 0 ~ 100

        '''
        degrees = [
            self._angle_to_int(radian * (180 / math.pi)) for radian in radians
        ]
        return self.__mesg(Command.SEND_ANGLES,
                          data=self._flatten([degrees, speed]))

    def get_coords(self):
        '''Get all coords.

        Return:
            data_list (list): [x, y, z, rx, ry, rz] 

        '''
        received = self.__mesg(Command.GET_COORDS, has_reply=True)
        if not received:
            return received

        res = []
        for idx in range(3):
            res.append(self._int_to_coord(received[idx]))
        for idx in range(3, 6):
            res.append(self._int_to_angle(received[idx]))
        return res

    def send_coord(self, id, coord, speed):
        '''Send one coord

        Args:
            id(common.Coord):
            coord(fload): mm
            speed(int):

        '''
        return self.__mesg(Command.SEND_COORD,
                          data=[id - 1,
                                self._coord_to_int(coord), speed])

    def send_coords(self, coords, speed, mode):
        '''Send all coords

        Args:
            coords: [x(mm), y, z, rx(angle), ry, rz]
            speed(int);
            mode(int): 0 - angluar, 1 - linear

        '''
        if len(coords) != 6:
            raise Exception('The leght of coords should be 6.')

        coord_list = []
        for idx in range(3):
            coord_list.append(self._coord_to_int(coords[idx]))
        for idx in range(3, 6):
            coord_list.append(self._angle_to_int(coords[idx]))
        self.__mesg(Command.SEND_COORDS,
                   data=self._flatten([coord_list, speed, mode]))

    def pause(self):
        self.__mesg(Command.PAUSE)

    def is_paused(self):
        return self._process_bool(self.__mesg(Command.IS_PAUSED,
                                             has_reply=True))

    def resume(self):
        self.__mesg(Command.RESUME)

    def stop(self):
        self.__mesg(Command.STOP)

    def is_in_position(self, data, id):
        '''

        Args:
            id: 1 - coords, 0 - angle

        Return:
            0 : error position
            1 : right position
            -1: error data
        '''
        if len(data) != 6:
            raise Exception('The lenght of coords is not right')

        if id == 1:
            data_list = []
            for idx in range(3):
                data_list.append(self._coord_to_int(data[idx]))
            for idx in range(3, 6):
                data_list.append(self._angle_to_int(data[idx]))
        elif id == 0:
            data_list = [self._coord_to_int(i) for i in data]
        else:
            raise Exception("id is not right, please input 0 or 1")

        received = self.__mesg(Command.IS_IN_POSITION,
                              data=data_list,
                              has_reply=True)
        return self._process_bool(received)

    def is_moving(self):
        '''

        Return:
            0 : not moving
            1 : is moving
            -1: error data
        '''
        return self._process_bool(self.__mesg(Command.IS_MOVING,
                                             has_reply=True))

    # JOG mode and operation
    def jog_angle(self, joint_id, direction, speed):
        '''Joint control

            joint_id: string
            direction: int [0, 1]
            speed: int (0 - 100)
        '''
        self.__mesg(Command.JOG_ANGLE, data=[joint_id, direction, speed])

    def jog_coord(self, coord_id, direction, speed):
        '''Coord control 

            coord: string
            direction: int [0, 1]
            speed: int (0 - 100)
        '''
        self.__mesg(Command.JOG_COORD, data=[coord_id, direction, speed])

    def jog_stop(self):
        self.__mesg(Command.JOG_STOP)

    # Running status and Settings
    def get_speed(self):
        return self._process_bool(self.__mesg(Command.GET_SPEED,
                                             has_reply=True))

    def set_speed(self, speed):
        '''Set speed value

        Args:
            speed (int): 0 - 100
        '''
        if not 0 <= speed <= 100:
            raise Exception('speed value not right, should be 0 ~ 100 ')

        self.__mesg(Command.SET_SPEED, data=[speed])

    def get_joint_min_angle(self, joint_id):
        angle = self.__mesg(Command.GET_JOINT_MIN_ANGLE,
                           data=[joint_id],
                           has_reply=True)
        return self._int_to_angle(angle[0]) if angle else 0

    def get_joint_max_angle(self, joint_id):
        angle = self.__mesg(Command.GET_JOINT_MAX_ANGLE,
                           data=[joint_id],
                           has_reply=True)
        return self._int_to_angle(angle[0]) if angle else 0

    # Servo control
    def is_servo_enable(self, servo_id):
        return self._process_bool(
            self.__mesg(Command.IS_SERVO_ENABLE, data=[servo_id]))

    def is_all_servo_enable(self):
        return self._process_bool(
            self.__mesg(Command.IS_ALL_SERVO_ENABLE, has_reply=True))

    # Atom IO
    def set_led_color(self, rgb):
        '''Set the light color

        Args:
            rgs (str): example 'ff0000'

        '''
        # rgb = struct.pack('bbb', rgb)
        command = 'fefe056a{}fa'.format(rgb)
        if self._version == 2:
            command = command.decode('hex')
        elif self._version == 3:
            command = bytes.fromhex(command)
        self._serial_port.write(command)
        self._serial_port.flush()
        time.sleep(0.05)

    def set_claw(self, flag):
        '''Set claw switch

        Args:
            flag (int): 0 - open, 1 - close

        '''
        if not flag in [0, 1]:
            raise Exception('eror flag, please input 0 or 1')

        self.__mesg(Command.SET_CLAW, data=[flag])
