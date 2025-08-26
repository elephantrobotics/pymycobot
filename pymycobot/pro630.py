# coding=utf-8
import locale
import time
import threading
import serial
from pymycobot.close_loop import CloseLoop
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode
from pymycobot.robot_info import _interpret_status_code


def get_local_language():
    language, _ = locale.getdefaultlocale()
    if language not in ["zh_CN", "en_US"]:
        language = "en_US"
    return language


def setup_serial_connect(port, baudrate="115200", timeout=0.1):
    serial_api = serial.Serial()
    serial_api.port = port
    serial_api.baudrate = baudrate
    serial_api.timeout = timeout
    serial_api.rts = False
    serial_api.open()
    return serial_api


class Pro630Api(CloseLoop):
    joint_number = 6
    power_control_1 = 3
    power_control_2 = 4
    arm_span_maximum = 630
    pin_numbered_mapping_table = {
        1: 17,
        2: 27,
        3: 22,
        4: 5,
        5: 6,
        6: 19
    }

    def __init__(self, debug=False, save_serial_log=False, method=None):
        super(Pro630Api, self).__init__(debug)
        self._joint_max_angles = [0] * self.joint_number
        self._joint_min_angles = [0] * self.joint_number
        self.save_serial_log = save_serial_log
        try:
            import numpy as np
        except ImportError:
            raise ImportError("Please install numpy")

        self.calibration_parameters = calibration_parameters
        self.language = get_local_language()
        self.read_threading = threading.Thread(target=self.read_thread, args=(method, ))
        self.read_threading.daemon = True
        self.read_threading.start()

    def _decode_command(self, genre, read_data):
        if read_data is None:
            return None

        if isinstance(read_data, int):
            return read_data

        decode_respond = []

        valid_data, data_len = read_data

        if data_len in [8, 12, 14, 16, 26, 60]:
            if data_len == 8 and (genre == ProtocolCode.IS_INIT_CALIBRATION):
                if valid_data[0] == 1:
                    return 1
                n = len(valid_data)
                for v in range(1, n):
                    decode_respond.append(valid_data[v])
            elif data_len == 8 and genre == ProtocolCode.GET_DOWN_ENCODERS:
                decode_respond = self.bytes4_to_int(valid_data)
            elif data_len == 6 and genre in [ProtocolCode.GET_SERVO_STATUS, ProtocolCode.GET_SERVO_VOLTAGES,
                                             ProtocolCode.GET_SERVO_CURRENTS]:
                for i in range(data_len):
                    decode_respond.append(valid_data[i])
            elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
                i = 0
                res = []
                while i < data_len:
                    if i < 8:
                        res.append(valid_data[i])
                        i += 1
                    else:
                        one = valid_data[i: i + 2]
                        res.append(self._decode_int16(one))
                        i += 2
                return res
            else:
                for header_i in range(0, len(valid_data), 2):
                    one = valid_data[header_i: header_i + 2]
                    decode_respond.append(self._decode_int16(one))
        elif data_len == 2:
            if genre in [ProtocolCode.IS_SERVO_ENABLE]:
                return [self._decode_int8(valid_data[1:2])]
            elif genre in [ProtocolCode.GET_ERROR_INFO]:
                return [self._decode_int8(valid_data[1:])]
            elif genre in [ProtocolCode.GET_LIMIT_SWITCH]:
                for i in valid_data:
                    decode_respond.append(i)
            elif genre in [ProtocolCode.GET_ROBOT_ID]:
                decode_respond.append(self._decode_int16(valid_data))
        elif data_len == 3:
            decode_respond.append(self._decode_int16(valid_data[1:]))
        elif data_len == 4:
            if genre == ProtocolCode.COBOTX_GET_ANGLE:
                decode_respond = self.bytes4_to_int(valid_data)
            for i in range(1, 4):
                decode_respond.append(valid_data[i])
        elif data_len == 7:
            error_list = [i for i in valid_data]
            for i in error_list:
                if i in range(16, 23):
                    decode_respond.append(1)
                elif i in range(23, 29):
                    decode_respond.append(2)
                elif i in range(32, 112):
                    decode_respond.append(3)
                else:
                    decode_respond.append(i)
        elif data_len == 24:
            decode_respond = self.bytes4_to_int(valid_data)
        elif data_len == 40:
            i = 0
            while i < data_len:
                if i < 28:
                    decode_respond += self.bytes4_to_int(valid_data)
                    i += 4
                else:
                    one = valid_data[i: i + 2]
                    decode_respond.append(self._decode_int16(one))
                    i += 2
        elif data_len == 32:
            i = 0
            res = []
            while i < 30:
                if i < 9 or i >= 23:
                    res.append(valid_data[i])
                    i += 1
                elif i < 23:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
            return res
        elif data_len == 38:
            i = 0
            res = []
            while i < data_len:
                if i < 10 or i >= 30:
                    res.append(valid_data[i])
                    i += 1
                elif i < 38:
                    one = valid_data[i: i + 2]
                    res.append(self._decode_int16(one))
                    i += 2
            return res
        elif data_len == 56:
            for i in range(0, data_len, 8):
                byte_value = int.from_bytes(valid_data[i:i + 4], byteorder='big', signed=True)
                decode_respond.append(byte_value)
        elif data_len == 6:
            for i in valid_data:
                decode_respond.append(i)
        else:
            if genre in [
                ProtocolCode.GET_SERVO_VOLTAGES,
                ProtocolCode.GET_SERVO_STATUS,
                ProtocolCode.GET_SERVO_TEMPS,
            ]:
                for i in range(data_len):
                    data1 = self._decode_int8(valid_data[i: i + 1])
                    decode_respond.append(0xFF & data1 if data1 < 0 else data1)
            decode_respond.append(self._decode_int8(valid_data))

        return decode_respond

    def _parsing_parameters(self, genre, res):
        if res is None:
            return None

        if isinstance(res, int):
            return res

        if genre in [
            ProtocolCode.ROBOT_VERSION,
            ProtocolCode.GET_ROBOT_ID,
            ProtocolCode.IS_POWER_ON,
            ProtocolCode.IS_CONTROLLER_CONNECTED,
            ProtocolCode.IS_PAUSED,
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
            ProtocolCode.GET_ERROR_DETECT_MODE,
            ProtocolCode.POWER_ON,
            ProtocolCode.POWER_OFF,
            ProtocolCode.RELEASE_ALL_SERVOS,
            ProtocolCode.RELEASE_SERVO,
            ProtocolCode.FOCUS_ALL_SERVOS,
            ProtocolCode.FOCUS_SERVO,
            ProtocolCode.STOP,
            ProtocolCode.SET_BREAK,
            ProtocolCode.IS_BTN_CLICKED,
            ProtocolCode.GET_CONTROL_MODE,
            ProtocolCode.GET_VR_MODE,
            ProtocolCode.GET_FILTER_LEN,
            ProtocolCode.IS_SERVO_ENABLE,
            ProtocolCode.GET_POS_SWITCH,
            ProtocolCode.GET_ERROR_INFO
        ]:
            return self._process_single(res)
        elif genre in [ProtocolCode.GET_ANGLES, ProtocolCode.SOLVE_INV_KINEMATICS]:
            return [self._int3angle(angle) for angle in res]
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
        elif genre in [ProtocolCode.COBOTX_GET_ANGLE, ProtocolCode.COBOTX_GET_SOLUTION_ANGLES,
                       ProtocolCode.GET_POS_OVER]:
            return self._int2angle(res[0])
        elif genre == ProtocolCode.MERCURY_ROBOT_STATUS:
            for i in range(9, len(res)):
                if res[i] != 0:
                    data = bin(res[i])[2:]
                    res[i] = []
                    while len(data) != 16:
                        data = "0" + data
                    for j in range(16):
                        if data[j] != "0":
                            error_id = 15 - j
                            res[i].append(error_id)
                    if not res[i]:
                        res[i] = 0
            return res
        elif genre == ProtocolCode.GET_MOTORS_RUN_ERR:
            for i in range(len(res)):
                if res[i] != 0:
                    data = bin(res[i])[2:]
                    res[i] = []
                    while len(data) != 16:
                        data = "0" + data
                    for j in range(16):
                        if data[j] != "0":
                            error_id = 15 - j
                            res[i].append(error_id)
                    if not res[i]:
                        res[i] = 0
            return res
        else:
            return res

    def _mesg(self, genre, *args, **kwargs):
        read_data = super(Pro630Api, self)._mesg(genre, *args, **kwargs)
        decode_result = self._decode_command(genre, read_data)
        return self._parsing_parameters(genre, decode_result)

    def _joint_limit_init(self):
        for joint, index in enumerate(range(6), start=1):
            if self._joint_max_angles[index] is None:
                angle = self.get_joint_max_angle(joint)
                self._joint_max_angles[index] = angle
            if self._joint_min_angles[index] is None:
                angle = self.get_joint_min_angle(joint)
                self._joint_min_angles[index] = angle

    def _joint_limit_judge(self, angles):
        offset = 3
        try:
            for joint, angle in enumerate(angles, start=1):
                min_angle = self._joint_min_angles[joint - 1]
                max_angle = self._joint_max_angles[joint - 1]
                if min_angle + offset < angle < max_angle - offset:
                    continue

                if self.language == "zh_CN":
                    return f"当前关节{joint}角度为{angle}, 角度范围为: {min_angle} ~ {max_angle}"
                return f"The angle of the current joint {joint} is {angle}, and the angle range is: {min_angle} ~ {max_angle}"
        except TypeError:
            return "joint limit error"
        return "over limit error {}".format(angles)

    def _check_coords(self, new_coords, is_print=False):
        if isinstance(new_coords, list):
            return "check coords error"

        error_info = ""
        first_three = new_coords[:3]
        first_three[2] -= 83.64

        magnitude = np.linalg.norm(first_three)  # Calculate the Euclidean norm (magnitude)
        if is_print == 1:
            if self.language == "zh_CN":
                error_info += f"当前臂展为{magnitude}, 最大的臂展为{self.arm_span_maximum}"
            else:
                error_info += f"Arm span is {magnitude}, max is {self.arm_span_maximum}"

        return error_info

    def _status_explain(self, status):
        error_info = _interpret_status_code(self.language, status)
        if 0x00 < status <= 0x07:
            self._joint_limit_init()
            angles = self.get_angles()
            error_info += self._joint_limit_judge(angles)
        elif status in [32, 33]:
            error_coords = self.get_coords()
            error_info += self._check_coords(error_coords, is_print=True)
        elif status == 36:
            angles = self.get_angles()
            if self.language == "zh_CN":
                error_info += f"当前角度为{angles}, 检测到奇异点"
            else:
                error_info += f"The current angle is {angles}, and a singularity is detected"
        return error_info


class Pro630(Pro630Api):

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False, save_serial_log=False):
        self._serial_port = setup_serial_connect(port, baudrate, timeout)
        super(Pro630, self).__init__(debug, save_serial_log)
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.power_control_1, GPIO.IN)
        GPIO.setup(self.power_control_2, GPIO.OUT)

    def open(self):
        self._serial_port.open()

    def close(self):
        self._serial_port.close()

    def power_on(self, delay=2):
        import RPi.GPIO as GPIO
        GPIO.output(self.power_control_2, GPIO.HIGH)
        time.sleep(delay)
        return super(Pro630, self).power_on()

    def power_off(self):
        import RPi.GPIO as GPIO
        res = super(Pro630, self).power_off()
        GPIO.output(self.power_control_2, GPIO.LOW)
        return res

    def power_on_only(self):
        import RPi.GPIO as GPIO
        GPIO.output(self.power_control_2, GPIO.HIGH)

    def set_basic_output(self, pin_no, pin_signal):
        """Set basic output.IO low-level output high-level, high-level output high resistance state

        Args:
            pin_no: pin port number. range 1 ~ 6
            pin_signal: 0 / 1
        """
        import RPi.GPIO as GPIO
        pin_number = self.pin_numbered_mapping_table.get(pin_no, None)
        if pin_number is None:
            raise ValueError("pin_no must be in range 1 ~ 6")
        GPIO.setup(pin_number, GPIO.OUT)
        GPIO.output(pin_number, pin_signal)

    def get_basic_input(self, pin_no):
        """Get basic input.

        Args:
            pin_no: pin port number. range 1 ~ 6

        Return:
            1 - high
            0 - low
        """
        import RPi.GPIO as GPIO
        pin_number = self.pin_numbered_mapping_table.get(pin_no, None)
        if pin_number is None:
            raise ValueError("pin_no must be in range 1 ~ 6")
        GPIO.setup(pin_number, GPIO.IN)
        return GPIO.input(pin_number)

    def send_angles_sync(self, angles, speed):
        angles = [self._angle2int(angle) for angle in angles]
        self.calibration_parameters(class_name=self.__class__.__name__, angles=angles, speed=speed)
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed, no_return=True)

    def set_monitor_mode(self, mode):
        raise NotImplementedError("Pro630 does not support monitor mode")

    def get_monitor_mode(self):
        raise NotImplementedError("Pro630 does not support monitor mode")
