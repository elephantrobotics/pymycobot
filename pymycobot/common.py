import sys
sys.path.append('.')
import struct


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

    def _process_data_command(self, data, genre):
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
            return self._flatten(
                [data[0],
                 self._flatten(self._encode_int16(data[1])), data[2]])

        elif genre in [
                Command.JOG_ANGLE, Command.JOG_COORD, Command.SET_COLOR
        ]:
            return data

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



