import socket
import threading
import serial

lock = threading.Lock()


def hex_to_signed_decimal(hex_str, bits=16):
    value = int(hex_str, 16)  # 先转换为无符号整数
    if value >= 2 ** (bits - 1):  # 如果超出正数范围，转换为负数
        value -= 2 ** bits
    return value


class Exoskeleton:
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=1000000)
        self.dataLen = 0

    def _parse_data(self, data):
        parsed_data = []
        for i in range(7):
            data_h = data[0 + i * 4: 2 + i * 4]
            data_l = data[2 + i * 4: 4 + i * 4]
            encode = int(data_h + data_l, 16)
            angle = 0 if encode == 2048 else (
                180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
            parsed_data.append(round(angle, 2))

        button = bin(int(data[28: 30], 16))[2:].rjust(4, "0")
        parsed_data.extend([int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[30: 32], 16),
                            int(data[32: 34], 16)])
        if self.dataLen == 31:  # 适配新固件带坐标信息
            for j in range(6):
                data_h = data[34 + j * 4: 36 + j * 4]
                data_l = data[36 + j * 4: 38 + j * 4]
                coord = hex_to_signed_decimal(data_h + data_l, 16)
                parsed_data.append(round(coord / 10, 2))
        return parsed_data

    def _parse_all_data(self, data):
        right_data = []
        left_data = []
        if self.dataLen == 60:  # 适配新固件带坐标信息
            for i in range(7):
                data_h = data[0 + i * 4: 2 + i * 4]
                data_l = data[2 + i * 4: 4 + i * 4]
                encode = int(data_h + data_l, 16)
                angle = 0 if encode == 2048 else (
                    180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
                left_data.append(round(angle, 2))

            button = bin(int(data[28: 30], 16))[2:].rjust(4, "0")
            left_data.extend([int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[30: 32], 16),
                              int(data[32: 34], 16)])

            for i in range(7):
                data_h = data[34 + i * 4: 36 + i * 4]
                data_l = data[36 + i * 4: 38 + i * 4]
                encode = int(data_h + data_l, 16)
                angle = 0 if encode == 2048 else (
                    180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
                right_data.append(round(angle, 2))

            button = bin(int(data[62: 64], 16))[2:].rjust(4, "0")
            right_data.extend(
                [int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[64: 66], 16),
                 int(data[66: 68], 16)])

            for j in range(6):
                data_h = data[68 + j * 4: 70 + j * 4]
                data_l = data[70 + j * 4: 72 + j * 4]
                coord = hex_to_signed_decimal(data_h + data_l, 16)
                left_data.append(round(coord / 10, 2))

            for j in range(6):
                data_h = data[92 + j * 4: 94 + j * 4]
                data_l = data[94 + j * 4: 96 + j * 4]
                coord = hex_to_signed_decimal(data_h + data_l, 16)
                right_data.append(round(coord / 10, 2))
        else:
            for i in range(7):
                data_h = data[0 + i * 4: 2 + i * 4]
                data_l = data[2 + i * 4: 4 + i * 4]
                encode = int(data_h + data_l, 16)
                angle = 0 if encode == 2048 else (
                    180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
                left_data.append(round(angle, 2))

            button = bin(int(data[28: 30], 16))[2:].rjust(4, "0")
            left_data.extend(
                [int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[30: 32], 16),
                 int(data[32: 34], 16)])

            for i in range(7):
                data_h = data[34 + i * 4: 36 + i * 4]
                data_l = data[36 + i * 4: 38 + i * 4]
                encode = int(data_h + data_l, 16)
                angle = 0 if encode == 2048 else (
                    180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
                right_data.append(round(angle, 2))

            button = bin(int(data[62: 64], 16))[2:].rjust(4, "0")
            right_data.extend(
                [int(button[-4]), int(button[-1]), int(button[-3]), int(button[-2]), int(data[64: 66], 16),
                 int(data[66: 68], 16)])
        return [left_data, right_data]

    def _commmon(self, command_array):
        with lock:
            commmon_id = command_array[3]
            self.ser.write(command_array)
            start1 = self.ser.read().hex()
            if start1 != "fe" or self.ser.read().hex() != "fe":
                return None
            data_len = int(self.ser.read().hex(), 16)
            count = self.ser.in_waiting
            if data_len == count:
                self.dataLen = count
                data = self.ser.read(count).hex()
                if data[-2:] == "fa" and int(data[0: 2], 10) == commmon_id:
                    return data[2: -2]
        return None

    def get_all_data(self):
        get_all_array = [0xFE, 0xFE, 0x02, 0x01, 0xFA]
        data = self._commmon(get_all_array)
        if data is None:
            return None
        return self._parse_all_data(data)

    def get_arm_data(self, arm):
        if arm not in [1, 2]:
            raise ValueError("error arm")

        send_array = [0xFE, 0xFE, 0x03, 0x02, arm, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        return self._parse_data(data)

    def get_joint_data(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x03, arm, id, 0xFA]
        data = self._commmon(send_array)
        if data is None:
            return None
        encode = int(data[0: 2] + data[2: 4], 16)
        angle = 0 if encode == 2048 else (
            180 * (encode - 2048) / 2048 if encode > 2048 else -180 * (2048 - encode) / 2048)
        return round(angle, 2)

    def set_zero(self, arm, id):
        if arm not in [1, 2] or id < 1 or id > 7:
            raise ValueError("error arm or id")

        send_array = [0xFE, 0xFE, 0x04, 0x04, arm, id, 0xFA]
        return self._commmon(send_array)

    def set_color(self, arm, red, green, blue):
        if arm not in [1, 2]:
            raise ValueError("error arm")
        send_array = [0xFE, 0xFE, 0x06, 0x05, arm, red, green, blue, 0xFA]
        return self._commmon(send_array)
    
    def get_basic_version(self):
        send_array = [0xFE, 0xFE, 0x02, 0x06, 0xFA]
        return int(self._commmon(send_array), 16)/10
    
    def get_atom_version(self, arm):
        send_array = [0xFE, 0xFE, 0x03, 0x07, arm, 0xFA]
        return int(self._commmon(send_array)[2:], 16)/10

    def set_atom_matrix(self, arm):
        send_array = [0xFE, 0xFE, 0x03, 0x08, arm, 0xFA]
        return self._commmon(send_array)