
import time

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
        is_in_position = False
        with self.lock:
            self.write_command.append(genre)
            self._write(self._flatten(real_command))
        if genre in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_ANGLE, ProtocolCode.JOG_COORD, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
            has_reply = True
        if has_reply:
            t = time.time()
            wait_time = 0.1   
            if genre == ProtocolCode.POWER_ON:
                wait_time = 8
            elif genre in [ProtocolCode.POWER_OFF, ProtocolCode.RELEASE_ALL_SERVOS, ProtocolCode.FOCUS_ALL_SERVOS,
                        ProtocolCode.RELEASE_SERVO, ProtocolCode.FOCUS_SERVO, ProtocolCode.STOP]:
                wait_time = 3
            elif genre in [ProtocolCode.SEND_ANGLE, ProtocolCode.SEND_ANGLES, ProtocolCode.SEND_COORD, ProtocolCode.SEND_COORDS, ProtocolCode.JOG_ANGLE, ProtocolCode.JOG_COORD, ProtocolCode.JOG_INCREMENT, ProtocolCode.JOG_INCREMENT_COORD, ProtocolCode.COBOTX_SET_SOLUTION_ANGLES]:
                wait_time = 300
                is_in_position = True
            need_break = False
            data = None
            while True and time.time() - t < wait_time:
                for v in self.read_command:
                    if is_in_position and v == b'\xfe\xfe\x04[\x01\r\x87':
                        need_break = True
                        with self.lock:
                            self.read_command.remove(v)
                            self.write_command.remove(genre)
                            return 1
                    elif genre == v[3]:
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
            unique_data = [ProtocolCode.GET_BASIC_INPUT, ProtocolCode.GET_DIGITAL_INPUT]
            if genre in unique_data:
                data_pos = 5
                data_len -= 1
            else:
                data_pos = 4
            valid_data = data[data_pos : data_pos + data_len]
            return (valid_data, data_len)
        return None
        
    def _process_received(self, data):
        if not data:
            return []
        elif data == b'\xfe\xfe\x04[\x01\r\x87':
            # 水星到位反馈
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
                    self.sock.settimeout(wait_time)
                    data = self.sock.recv(1024)
                    if isinstance(data, str):
                        datas = bytearray()
                        for i in data:   
                            datas += hex(ord(i))
                except:
                    data = b""
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
                if data:
                    res = self._process_received(data)
                    with self.lock:
                        self.read_command.append(res)
            else:
                while True and time.time() - t < wait_time:
                    # print("r", end=" ", flush=True)
                    if self._serial_port.inWaiting() > 0:
                        data = self._serial_port.read()
                        k += 1
                        # print(datas, flush=True)
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
                    # else:
                    #     print("no data", flush=True)
                else:
                    datas = b''
                if datas:
                    res = self._process_received(datas)
                    
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
                    with self.lock:
                        self.read_command.append(res)
                # return datas
        
    def bytes4_to_int(self, bytes4):
        i = 0
        res = []
        data_len = len(bytes4)
        while i < data_len:
            byte_value = int.from_bytes(bytes4[i:i+4], byteorder='big', signed=True)
            i+=4
            res.append(byte_value)
        return res