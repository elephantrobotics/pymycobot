# coding=utf-8
import functools
import os
import socket
import traceback
import re

from pymycobot.robot_info import RobotLimit
# In order to adapt to the atom side, ID of 0-5 or 1-6 are allowed.
# In order to support end control, ID 7 is allowed.
MIN_ID = 0
MAX_ID = 7

# In fact, most joints cannot reach plus or minus 180 angles.
# There may be a value greater than 180 when reading the angle,
# and the maximum and minimum values are expanded for compatibility.
MIN_ANGLE = -190.0
MAX_ANGLE = 190.0


class MyCobotDataException(Exception):
    pass


class MercuryDataException(Exception):
    pass


class MyAgvDataException(Exception):
    pass


class MechArmDataException(Exception):
    pass


class MyArmDataException(Exception):
    pass


class UltraArmDataException(Exception):
    pass


class MyPalletizerDataException(Exception):
    pass


class MyPalletizer260DataException(Exception):
    pass


class MyBuddyDataException(Exception):
    pass


class MyCobot280DataException(Exception):
    pass


class MyCobot320DataException(Exception):
    pass

class MercuryRobotException(Exception):
    pass

class MyCobot630ProDataException(Exception):
    pass

class MyCobotPro450DataException(Exception):
    pass

class ultraArmP340DataException(Exception):
    pass


def check_boolean(b):
    if b != 0 and b != 1:
        raise MyCobotDataException("This parameter needs to be 0 or 1")


def check_gripper_pid_value(value, exception_class, class_name):
    pid_str = ["p", "i", "d"]
    for i, v in enumerate(value):
        if not (0 <= v <= 254):
            raise exception_class(
                "The PID value for {} needs to be 0 ~ 254, but received the {} is {}".format(class_name, pid_str[i], v)
            )


def check_rgb_value(value, exception_class, class_name):
    rgb_str = ["r", "g", "b"]
    for i, v in enumerate(value):
        if not (0 <= v <= 255):
            raise exception_class(
                "The RGB value for {} needs to be 0 ~ 255, but received the {} is {}".format(class_name, rgb_str[i], v)
            )


def check_value_type(parameter, value_type, exception_class, _type):
    if value_type is not _type:
        raise exception_class(
            "The acceptable parameter {} should be an {}, but the received {}".format(parameter, _type, value_type))


def check_coords(parameter_name, value, robot_limit, class_name, exception_class, serial_port=None):
    if not isinstance(value, list):
        raise exception_class("`{}` must be a list, but the received {}".format(parameter_name, type(value)))
    if len(value) != 6:
        raise exception_class(
            "The length of `{}` must be 6, but the received length is {}".format(parameter_name, len(value)))
    if serial_port:
        if serial_port == "/dev/left_arm":
            min_coord = robot_limit[class_name]["left_coords_min"]
            max_coord = robot_limit[class_name]["left_coords_max"]
        elif serial_port == "/dev/right_arm":
            min_coord = robot_limit[class_name]["right_coords_min"]
            max_coord = robot_limit[class_name]["right_coords_max"]
    else:
        min_coord = robot_limit[class_name]["coords_min"]
        max_coord = robot_limit[class_name]["coords_max"]
    for idx, coord in enumerate(value):
        if not min_coord[idx] <= coord <= max_coord[idx]:
            raise exception_class(
                "Has invalid coord value, error on index {0}, received {3}, but coord should be {1} ~ {2}.".format(
                    idx, min_coord[idx], max_coord[idx], coord))

def check_world_tool_coords(parameter_name, value, exception_class):
    if not isinstance(value, list):
        raise exception_class("`{}` must be a list, but the received {}".format(parameter_name, type(value)))
    if len(value) != 6:
        raise exception_class(
            "The length of `{}` must be 6, but the received length is {}".format(parameter_name, len(value)))

    min_coord = [-1000, -1000, -1000, -180, -180, -180]
    max_coord = [1000, 1000, 1000, 180, 180, 180]
    for idx, coord in enumerate(value):
        if not min_coord[idx] <= coord <= max_coord[idx]:
            raise exception_class(
                "Has invalid coord value, error on index {0}, received {3}, but coord should be {1} ~ {2}.".format(
                    idx, min_coord[idx], max_coord[idx], coord))


def check_angles(angle_value, robot_limit, class_name, exception_class):
    # Check if angle_value is a list
    if not isinstance(angle_value, list):
        raise exception_class("`angles` must be a list, but the received {}".format(type(angle_value)))
    # Check angles
    if len(angle_value) != 6:
        raise exception_class("The length of `angles` must be 6, but received length is {}".format(len(angle_value)))
    for idx, angle in enumerate(angle_value):
        if not robot_limit[class_name]["angles_min"][idx] <= angle <= robot_limit[class_name]["angles_max"][idx]:
            raise exception_class(
                "Has invalid angle value, error on index {0}. Received {3} but angle should be {1} ~ {2}.".format(
                    idx, robot_limit[class_name]["angles_min"][idx], robot_limit[class_name]["angles_max"][idx], angle))


def check_0_or_1(parameter, value, range_data, value_type, exception_class, _type):
    check_value_type(parameter, value_type, exception_class, _type)
    if value not in range_data:
        error = "The data supported by parameter {} is ".format(parameter)
        len_data = len(range_data)
        for idx in range(len_data):
            error += str(range_data[idx])
            if idx != len_data - 1:
                error += " or "
        error += ", but the received value is {}".format(value)
        raise exception_class(error)


def check_id(value, id_list, exception_class):
    raise exception_class(
        "The joint_id not right, should be in {0}, but received {1}.".format(
        id_list, value
        )
    )


def public_check(parameter_list, kwargs, robot_limit, class_name, exception_class):
    for parameter in parameter_list[1:]:
        value = kwargs.get(parameter, None)
        value_type = type(value)
        if parameter == 'id' and value not in robot_limit[class_name][parameter]:
            check_id(value, robot_limit[class_name][parameter], exception_class)
        elif parameter == 'rgb':
            check_rgb_value(value, exception_class, class_name)
        elif parameter == 'value':
            check_value_type(parameter, value_type, exception_class, int)
            if value < 0 or value > 4096:
                raise exception_class(
                    "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
        elif parameter == 'pin_mode':
            check_value_type(parameter, value_type, exception_class, int)
            if value not in [0, 1, 2]:
                raise exception_class(
                    "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(parameter,
                                                                                                             value))
        elif parameter == 'pin_signal':
            check_0_or_1(parameter, value, [0, 1], value_type, exception_class, int)
        elif parameter == 'speed':
            check_value_type(parameter, value_type, exception_class, int)
            if not 1 <= value <= 100:
                raise exception_class(
                    "speed value not right, should be 1 ~ 100, the received speed is %s"
                    % value
                )
        elif parameter == 'flag':
            check_0_or_1(parameter, value, [0, 1, 254], value_type, exception_class, int)
        elif parameter == 'gripper_type':
            check_0_or_1(parameter, value, [1, 3, 4], value_type, exception_class, int)
        elif parameter == '_type_1':
            check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, exception_class, int)
            # if value not in [0, 1, 10]:
            #     raise exception_class("The data supported by parameter {} is 0 or 1 or 10, but the received value is {}".format(parameter, value))
        elif parameter == 'gripper_value':
            check_value_type(parameter, value_type, exception_class, int)
            if value < 0 or value > 100:
                raise exception_class(
                    "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
        elif parameter in ['account', 'password']:
            check_value_type(parameter, value_type, exception_class, str)
        # TODO 280/320共用MyCobot，无法进行数据限位
        # elif parameter == 'coords':
        #     check_coords(value, robot_limit, class_name, exception_class)
        elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction', 'state', 'deceleration']:
            check_0_or_1(parameter, value, [0, 1], value_type, exception_class, int)
        elif parameter == 'acceleration':
            check_value_type(parameter, value_type, exception_class, int)
            if not 1 <= value <= 100:
                raise exception_class(
                    "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                )
        elif parameter == "angles":
            check_angles(value, robot_limit, class_name, exception_class)
        elif parameter == 'angle':
            joint_id = kwargs.get('id', None)
            index = robot_limit[class_name]['id'][joint_id-1] - 1
            if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                raise exception_class(
                    "angle value not right, should be {0} ~ {1}, but received {2}".format(
                        robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                        value
                    )
                )
        elif parameter == 'encoders':
            if "MyCobot" in class_name or "MechArm" in class_name:
                if len(value) != 6:
                    raise exception_class("The length of `encoders` must be 6.")
            elif class_name in ["MyPalletizer", "MyPalletizer260"]:
                if len(value) != 4:
                    raise exception_class("The length of `encoders` must be 4.")
            elif "MyArm" in class_name:
                if len(value) != 7:
                    raise exception_class("The length of `encoders` must be 7.")
            for data in value:
                data_type = type(data)
                check_value_type(parameter, data_type, exception_class, int)
                if data < 0 or data > 4096:
                    raise exception_class("The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
        elif parameter == 'speeds':
            if "MyCobot" in class_name or "MechArm" in class_name:
                if len(value) not in [6, 7]:
                    raise exception_class(
                        "The length of `speeds` must be 6. but the received value is {}".format(value))
            elif class_name in ["MyPalletizer", "MyPalletizer260"]:
                if len(value) != 4:
                    raise exception_class("The length of `speeds` must be 4.")
            elif "MyArm" in class_name:
                if len(value) != 7:
                    raise exception_class("The length of `speeds` must be 7.")
            for data in value:
                data_type = type(data)
                check_value_type(parameter, data_type, exception_class, int)
                if data < 0 or data > 6000:
                    raise exception_class("The range of speed is 0 ~ 6000, but the received value is {}".format(data))
        elif parameter in ['servo_id_pdi', 'encode_id']:
            check_value_type(parameter, value_type, exception_class, int)
            if "MyCobot" in class_name or "MechArm" in class_name:
                if value < 1 or value > 7:
                    raise exception_class("The range of id is 1 ~ 6 or 7, but the received is {}".format(value))
            if class_name in ["MyPalletizer", "MyPalletizer260"]:
                if value < 1 or value > 4:
                    raise exception_class("The range of id is 1 ~ 4, but the received is {}".format(value))
            elif "MyArm" in class_name or "MyCobot" in class_name or "MechArm" in class_name:
                if value < 1 or value > 7:
                        raise exception_class("The range of joint_id is 1 ~ 7, but the received is {}".format(value))
        elif parameter == "torque":
            torque_min = 150
            torque_max = 980
            if value < torque_min or value > torque_max:
                raise exception_class(
                    "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
        elif parameter == "current":
            current_min = 1
            current_max = 500
            if value < current_min or value > current_max:
                raise exception_class(
                    "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max, value))
        elif parameter == 'end_direction':
            check_0_or_1(parameter, value, [1, 2, 3], value_type, exception_class, int)
        elif parameter == "pin_no":
            if class_name in ["Mercury"]:
                check_0_or_1(parameter, value, [1, 2, 3, 4, 5, 6], value_type, exception_class, int)
            elif class_name in ["MyCobot320", "MyCobot320Socket"]:
                check_0_or_1(parameter, value, [1, 2], value_type, exception_class, int)
            elif class_name in ["MyCobot280", "MyCobot280Socket", "MechArm270", "MechArmSocket", "MyPalletizer260", "MyPalletizerSocket"]:
                check_0_or_1(parameter, value, [19, 22, 23, 33], value_type, exception_class, int)
        elif parameter == "pin_no_basic":
            if class_name in ["Mercury"]:
                check_0_or_1(parameter, value, [1, 2, 3, 4, 5, 6], value_type, exception_class, int)
            elif class_name in ["MyCobot320", "MyCobot320Socket"]:
                check_0_or_1(parameter, value, [1, 2, 3, 4, 5, 6], value_type, exception_class, int)
            elif class_name in ["MyCobot280", "MyCobot280Socket", "MechArm270", "MechArmSocket", "MyPalletizer260", "MyPalletizerSocket"]:
                check_0_or_1(parameter, value, [2, 5, 35, 36], value_type, exception_class, int)


def calibration_parameters(**kwargs):
    # with open(os.path.dirname(os.path.abspath(__file__))+"/robot_limit.json") as f:
    robot_limit = RobotLimit.robot_limit
    parameter_list = list(kwargs.keys())
    class_name = kwargs.get("class_name", None)
    if class_name in ["Mercury", "MercurySocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            if parameter == 'joint_id':
                if value not in robot_limit[class_name][parameter]:
                    check_id(value, robot_limit[class_name][parameter], MercuryDataException)
            elif parameter == 'angle':
                joint_id = kwargs.get('joint_id', None)
                if joint_id in [11,12,13]:
                    index = robot_limit[class_name]['joint_id'][joint_id-4] - 4
                else:
                    index = robot_limit[class_name]['joint_id'][joint_id-1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                    raise MercuryDataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            elif parameter == 'angles':
                if len(value) not in [7, 10]:
                    raise MercuryDataException("The length of `angles` must be 7 or 10.")
                for idx, angle in enumerate(value):
                    joint_id = idx+1
                    angle_min = robot_limit[class_name]["angles_min"][idx]
                    angle_max = robot_limit[class_name]["angles_max"][idx]
                    if angle < angle_min or angle > angle_max:
                        raise MercuryDataException(
                            "Joint {} angle value of {} exceeds the limit, with a limit range of {} ~ {}.".format(joint_id, angle, angle_min, angle_max)
                        )

            elif parameter == 'coord':
                
                index = kwargs.get('coord_id', None) - 1
                if value < robot_limit[class_name]["coords_min"][index] or value > robot_limit[class_name]["coords_max"][index]:
                    raise MercuryDataException(
                        "The `coord` value of {} exceeds the limit, and the limit range is {} ~ {}".format(
                            value, robot_limit[class_name]["coords_min"][index], robot_limit[class_name]["coords_max"][index]
                        )
                    )
            elif parameter == 'base_coord':
                coord_id = kwargs.get('coord_id', None)
                
                if isinstance(coord_id, int):
                    index = coord_id - 1
                    serial_port = kwargs.get('serial_port', None)
                    if serial_port == "/dev/left_arm":
                        min_coord = robot_limit[class_name]["left_coords_min"][index]
                        max_coord = robot_limit[class_name]["left_coords_max"][index]
                    elif serial_port == "/dev/right_arm":
                        min_coord = robot_limit[class_name]["right_coords_min"][index]
                        max_coord = robot_limit[class_name]["right_coords_max"][index]
                    else:
                        min_coord = robot_limit[class_name]["coords_min"][index]
                        max_coord = robot_limit[class_name]["coords_max"][index]
                    if value < min_coord or value > max_coord:
                        raise MercuryDataException(
                            "The `base_coord` value of {} exceeds the limit, and the limit coord is {} ~ {}".format(
                                value, min_coord, max_coord
                            )
                        )
            elif parameter in ['coords', 'base_coords']:
                serial_port = kwargs.get('serial_port', None)
                check_coords(parameter, value, robot_limit, class_name, MercuryDataException, serial_port)

            elif parameter == 'speed':
                if not 1 <= value <= 100:
                    raise MercuryDataException(
                        "speed value not right, should be 1 ~ 100, the error speed is %s"
                        % value
                    )

            elif parameter == 'rgb':
                check_rgb_value(value, MercuryDataException, class_name)
            # if direction is not None:
            elif parameter in ['direction', 'flag', 'value']:
                if value not in [0, 1]:
                    raise MercuryDataException("{} only supports 0 or 1, but received {}".format(parameter, value))

            elif parameter == 'coord_id':
                if value < 1 or value > 6:
                    raise MercuryDataException("coord_id only supports 1 ~ 6, but received {}".format(value))

            elif parameter == 'solution_angle':
                if value > 90 or value < -90:
                    raise MercuryDataException("The angle range is -90° ~ 90°, but received {}".format(value))
            elif parameter == 'address':
                if value < 32 or value > 34:
                    raise MercuryDataException("The angle address is 32 ~ 34, but received {}".format(value))
            elif parameter == 'value':
                if value < 1 or value > 32000:
                    raise MercuryDataException("The angle value is 1 ~ 32000, but received {}".format(value))
            elif parameter == "servo_restore":
                if value not in [1, 2, 3, 4, 5, 6, 7, 13, 254]:
                    raise MercuryDataException(
                        "The joint_id should be in [1,2,3,4,5,6,7,13,254], but received {}".format(value))
            elif parameter == "data_len":
                if value < 1 or value > 45:
                    raise MercuryDataException(
                        "The parameter data_len data range only supports 1 ~ 45, but received {}".format(value))
            elif parameter == "max_time":
                if value < 0:
                    raise MercuryDataException("The parameter max_time must be greater than or equal to 0, but received {}".format(value))
            elif parameter == "limit_mode":
                if value not in [1,2]:
                    raise MercuryDataException("The parameter {} only supports 1, 2, but received {}".format(parameter, value))
            elif parameter == "_type":
                if value not in [1,2,3,4]:
                    raise MercuryDataException("The parameter {} only supports 1, 2, 3, 4, but received {}".format(parameter, value))
            elif parameter == "axis":
                if value not in [1,2,3]:
                    raise MercuryDataException("The parameter {} only supports 1, 2, 3, but received {}".format(parameter, value))
            elif parameter == "threshold_value":
                if value < 50 or value > 250:
                    raise MercuryDataException("The parameter {} only supports 50 ~ 250, but received {}".format(parameter, value))
            elif parameter == "comp_value":
                if value < 0 or value > 250:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 250, but received {}".format(parameter, value))
            elif parameter == "shoot_value":
                if value < -300 or value > 300:
                    raise MercuryDataException("The parameter {} only supports -300 ~ 300, but received {}".format(parameter, value))
            elif parameter == "head_id":
                if value not in [11,12]:
                    raise MercuryDataException("The parameter {} only supports 11, 12, but received {}".format(parameter, value))
            elif parameter == "err_angle":
                if value < 0 or value > 5:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 5, but received {}".format(parameter, value))
            elif parameter == "r":
                if value < 0 or value > 655.5:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 655.5, but received {}".format(parameter, value))
            elif parameter == "rank":
                if value not in [0,1,2]:
                    raise MercuryDataException("The parameter {} only supports 0 or 1 or 2, but received {}".format(parameter, value))
            elif parameter == "move_type":
                if value not in [0, 1, 2, 3, 4]:
                    raise MercuryDataException("The parameter {} only supports 0 or 4, but received {}".format(parameter, value))
            elif parameter == "trajectory":
                if value not in [0,1,2,3,4]:
                    raise MercuryDataException("The parameter {} only supports [0,1,2,3,4], but received {}".format(parameter, value))
            elif parameter in ["gripper_id", "new_hand_id"]:
                if value < 1 or value > 254:
                    raise MercuryDataException("The parameter {} only supports 1 ~ 254, but received {}".format(parameter, value))
            elif parameter == "gripper_address":
                if value < 1 or value > 44 or value in [15,17,19]:
                    raise MercuryDataException("The parameter {} only supports 1 ~ 44 (except 15, 17, and 19), but received {}".format(parameter, value))
            elif parameter == "gripper_angle":
                if value < 0 or value > 100:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 100, but received {}".format(parameter, value))
            elif parameter == "torque":
                if value < 1 or value > 100:
                    raise MercuryDataException("The parameter {} only supports 1 ~ 100, but received {}".format(parameter, value))
            elif parameter == "hand_id":
                if value < 1 or value > 6:
                    raise MercuryDataException("The parameter {} only supports 1 ~ 6, but received {}".format(parameter, value))
            elif parameter == 'pinch_mode':
                check_0_or_1(parameter, value, [0, 1, 2, 3], value_type, MercuryDataException, int)
            elif parameter == "pinch_pose":
                if value < 0 or value > 4:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 4, but received {}".format(parameter, value))
            elif parameter == "rank_mode":
                if value < 0 or value > 5:
                    raise MercuryDataException("The parameter {} only supports 0 ~ 5, but received {}".format(parameter, value))
            elif parameter == "idle_flag":
                if value != 1:
                    raise MercuryDataException("The parameter {} only supports 1, but received {}".format(parameter, value))
            else:
                public_check(parameter_list, kwargs, robot_limit, class_name, MercuryDataException)
    elif class_name == "MyAgv":
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            if parameter == 'rgb':
                check_rgb_value(value, MyAgvDataException, class_name)
            elif parameter == 'led_mode':
                if value not in [1, 2]:
                    raise MyAgvDataException("led mode only supports 1 or 2, but received is {}".format(value))
            elif parameter == 'direction_1':
                if value < 0 or value > 255:
                    raise MyAgvDataException(
                        "The range of direction_1 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'direction_2':
                if value < 0 or value > 255:
                    raise MyAgvDataException(
                        "The range of direction_2 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'direction_3':
                if value < 0 or value > 255:
                    raise MyAgvDataException(
                        "The range of direction_3 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'data':
                if value < 1 or value > 127:
                    raise MyAgvDataException(
                        "The range of {} is 1 ~ 127, but the received value is {}".format(parameter, value))

    elif class_name in ["MyCobot", "MyCobotSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MyCobotDataException)
    elif class_name in ["MyCobot280", "MyCobot280Socket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyCobot280DataException)
            elif parameter == 'servo_data_id' and value not in [1, 2, 3, 4, 5, 6, 7]:
                raise MyCobot280DataException(
                    "The id not right, should be in {0}, but received {1}.".format([1, 2, 3, 4, 5, 6, 7], value))
            elif parameter == 'address_id' and value in range(0, 5):
                raise MyCobot280DataException(
                    "The address_id is invalid: should not be in [0, 1, 2, 3, 4], but received {}.".format(value))
            elif parameter == 'rgb':
                check_rgb_value(value, MyCobot280DataException, class_name)
            elif parameter == 'value':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value < 0 or value > 4096:
                    raise MyCobot280DataException(
                        "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value not in [0, 1, 2]:
                    raise MyCobot280DataException(
                        "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(
                            parameter,
                            value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot280DataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot280DataException(
                        "speed value not right, should be 1 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'gripper_speed':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if not 0 <= value <= 100:
                    raise MyCobot280DataException(
                        "speed value not right, should be 0 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MyCobot280DataException, int)
            elif parameter == 'gripper_type':
                if value is not None:
                    check_0_or_1(parameter, value, [1, 3, 4, 5], value_type, MyCobot280DataException, int)
            elif parameter == '_type_1':
                if value is not None:
                    check_0_or_1(parameter, value, [1, 2, 3, 4, 5], value_type, MyCobot280DataException, int)
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value < 0 or value > 100:
                    raise MyCobot280DataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyCobot280DataException, str)
            elif parameter == 'coords':
                check_coords(parameter, value, robot_limit, class_name, MyCobot280DataException)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot280DataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot280DataException(
                        "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                    )
            elif parameter == "angles":
                check_angles(value, robot_limit, class_name, MyCobot280DataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][
                            index]:
                    raise MyCobot280DataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            elif parameter == 'coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > \
                        robot_limit[class_name]["coords_max"][index]:
                    raise MyCobot280DataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],
                            robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )
            elif parameter == 'encoders':

                if len(value) != 6:
                    raise MyCobot280DataException("The length of `encoders` must be 6.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyCobot280DataException, int)
                    if data < 0 or data > 4096:
                        raise MyCobot280DataException(
                            "The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
            elif parameter == 'speeds':
                if len(value) not in [6, 7]:
                    raise MyCobot280DataException(
                        "The length of `speeds` must be 6. but the received value is {}".format(value))
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyCobot280DataException, int)
                    if data < 0 or data > 6000:
                        raise MyCobot280DataException(
                            "The range of speed is 0 ~ 6000, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value < 1 or value > 7:
                    raise MyCobot280DataException("The range of id is 1 ~ 6 or 7, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MyCobot280DataException(
                        "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MyCobot280DataException(
                        "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max,
                                                                                         value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MyCobot280DataException, int)
            elif parameter == 'is_torque':
                if value is not None:
                    check_0_or_1(parameter, value, [0, 1], value_type, MyCobot280DataException, int)
            else:
                public_check(parameter_list, kwargs, robot_limit, class_name, MyCobot280DataException)
    elif class_name in ["MyCobot320", "MyCobot320Socket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            limit_info = robot_limit[class_name]
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyCobot320DataException)
            elif parameter in ("servo_id", "joint_id", "coord_id") and value not in limit_info[parameter]:
                raise ValueError(
                    f"The {parameter} not right, should be in {limit_info[parameter]}, but received {value}.")
            elif parameter == 'servo_data_id' and value not in [1, 2, 3, 4, 5, 6, 7]:
                raise MyCobot320DataException(
                    "The id not right, should be in {0}, but received {1}.".format([1, 2, 3, 4, 5, 6, 7], value))
            elif parameter == 'address_id' and value in range(0, 5):
                raise MyCobot320DataException("The address_id is invalid: should not be in [0, 1, 2, 3, 4], but received {}.".format(value))
            elif parameter == 'rgb':
                check_rgb_value(value, MyCobot320DataException, class_name)
            elif parameter == 'value':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 4096:
                    raise MyCobot320DataException("The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value not in [0, 1, 2]:
                    raise MyCobot320DataException("The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot320DataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot320DataException("speed value not right, should be 1 ~ 100, the received speed is %s" % value)
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot320DataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MyCobot320DataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MyCobot320DataException, int)
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 100:
                    raise MyCobot320DataException("The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyCobot320DataException, str)
            elif parameter == 'coords':
                check_coords(parameter, value, robot_limit, class_name, MyCobot320DataException)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot320DataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot320DataException("{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value))
            elif parameter == "angles":
                check_angles(value, robot_limit, class_name, MyCobot320DataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                    raise MyCobot320DataException("angle value not right, should be {0} ~ {1}, but received {2}".format(robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index], value))
            elif parameter == 'coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > robot_limit[class_name]["coords_max"][index]:
                    raise MyCobot320DataException("Coordinate value not right, should be {0} ~ {1}, but received {2}".format(robot_limit[class_name]["coords_min"][index], robot_limit[class_name]["coords_max"][index], value))
            elif parameter == 'encoder':
                if not 0 <= value <= 4096:
                    raise ValueError(f"The range of encoder is 0 ~ 4096, but the received value is {value}")
            elif parameter == 'encoders':
                if len(value) != 6:
                    raise MyCobot320DataException("The length of `encoders` must be 6.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyCobot320DataException, int)
                    if data < 0 or data > 4096:
                        raise MyCobot320DataException(
                            "The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
            elif parameter == 'speeds':
                if len(value) not in [6, 7]:
                    raise MyCobot320DataException("The length of `speeds` must be 6. but the received value is {}".format(value))
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyCobot320DataException, int)
                    if data < 0 or data > 6000:
                        raise MyCobot320DataException("The range of speed is 0 ~ 6000, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 6:
                    raise MyCobot320DataException("The range of id is 1 ~ 6, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MyCobot320DataException("The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MyCobot320DataException("The range of current is {} ~ {}, but the received is {}".format(current_min, current_max, value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MyCobot320DataException, int)
            elif parameter == "gripper_angle":
                gripper_id, gripper_angle = value
                if not isinstance(gripper_id, int) or not isinstance(gripper_angle, int):
                    raise MyCobot320DataException("Both 'gripper_id' and 'gripper_angle' in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, gripper_id))
                if gripper_angle < 0 or gripper_angle > 100:
                    raise MyCobot320DataException("The range of 'gripper_angle' in {} is 0 ~ 100, but the received value is {}".format(parameter, gripper_angle))
            elif parameter == "gripper_id" or parameter == "set_id":
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, value))
            elif parameter == "torque_value":
                gripper_id, torque_value = value
                if not isinstance(gripper_id, int) or not isinstance(torque_value, int):
                    raise MyCobot320DataException("Both 'gripper_id' and 'torque_value' in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, gripper_id))
                if torque_value < 0 or torque_value > 100:
                    raise MyCobot320DataException("The range of 'torque_value' in {} is 0 ~ 100, but the received value is {}".format(parameter, torque_value))

            elif parameter == "gripper_speed":
                gripper_id, speed = value
                if not isinstance(gripper_id, int) or not isinstance(speed, int):
                    raise MyCobot320DataException("Both 'gripper_id' and 'speed' in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, gripper_id))
                if speed < 1 or speed > 100:
                    raise MyCobot320DataException("The range of 'speed' in {} is 1 ~ 100, but the received value is {}".format(parameter, speed))

            elif parameter == "set_gripper_args":
                if len(value) != 3:
                    raise ValueError(f"Expected 3 arguments, but got {len(value)}")
                gripper_id, address, data = value
                if not isinstance(gripper_id, int) or not isinstance(address, int) or not isinstance(data, int):
                    raise MyCobot320DataException(
                        "All arguments in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, gripper_id))
                invalid_addresses = [1, 2, 4, 5, 6, 7, 8, 9, 12, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 28, 33, 34, 35,
                                     40, 42, 44]
                if address < 1 or address > 44:
                    raise MyCobot320DataException("The range of 'address' in {} is 1 ~ 44, but the received value is {}".format(parameter, address))
                if address in invalid_addresses:
                    raise MyCobot320DataException("'address' in {} cannot be one of the following values: {}, but the received value is {}".format(parameter, invalid_addresses, address))
                # Process value according to address
                if address in [3]:
                    if data < 1 or data > 254:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 1 ~ 254, but the received value is {}".format(parameter, address, data))
                elif address in [43]:
                    if data < 100 or data > 300:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 100 ~ 300, but the received value is {}".format(parameter, address, data))
                elif address == 10:
                    if data not in [0, 1]:
                        raise MyCobot320DataException("Error in parameter '{}': Value for address={} must be 0 or 1, but the received value is {}".format(parameter, address, data))
                elif address in [25]:
                    if data < 0 or data > 254:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 0 ~ 254, but the received value is {}".format(parameter, address, data))
                elif address in [21, 23]:
                    if data < 0 or data > 16:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 0 ~ 16, but the received value is {}".format(parameter, address, data))
                elif address in[30, 31, 41]:
                    if data < 0 or data > 100:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 0 ~ 100, but the received value is {}".format(parameter, address, data))
                elif address == 29:
                    if data not in [0, 1, 16, 17]:
                        raise MyCobot320DataException("Error in parameter '{}': The range of 'value' for address={} is 0 or 1 or 16 or 17, but the received value is {}".format(parameter, address, data))
            elif parameter == "get_gripper_args":
                gripper_id, address = value
                if not isinstance(gripper_id, int) or not isinstance(address, int):
                    raise MyCobot320DataException(
                        "All arguments in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter, gripper_id))
                invalid_addresses = [3, 5, 6, 10, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 30, 31, 32, 36, 37,
                                     38, 39, 41, 43]
                if address < 1 or address > 44:
                    raise MyCobot320DataException("The range of 'address' in {} is 1 ~ 44, but the received value is {}".format(parameter, address))
                if address in invalid_addresses:
                    raise MyCobot320DataException("'address' in {} cannot be one of the following values: {}, but the received value is {}".format(parameter, invalid_addresses, address))

            elif parameter == "gripper_joint_id":
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 6:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 6, but the received value is {}".format(parameter, value))
            elif parameter == "gripper_angles":
                if len(value) != 6:
                    raise MyCobot320DataException("{}: 'gripper_angles' must contain exactly 6 values, but received {}.".format(class_name, len(value)))

                for i, angle in enumerate(value):
                    if not isinstance(angle, int):
                        raise MyCobot320DataException("{}: Value at position {} in 'gripper_angles' must be an integer, but received {}.".format(class_name, i + 1, angle))
                    if angle < 0 or angle > 100:
                        raise MyCobot320DataException("{}: Value {} at position {} in 'gripper_angles' is out of range (0 ~ 100).".format(class_name, angle, i + 1))
            elif parameter == 'pinch_mode':
                check_0_or_1(parameter, value, [0, 1, 2, 3], value_type, MyCobot320DataException, int)
            elif parameter == "gripper_finger_id":
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 5:
                    raise MyCobot320DataException("The range of 'gripper_id' in {} is 1 ~ 6, but the received value is {}".format(parameter, value))
            elif parameter == "clockwise":
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 16:
                    raise MyCobot320DataException("The range of 'value' in {} is 0 ~ 16, but the received value is {}".format(parameter, value))
            elif parameter in ["min_pressure", "gripper_i"]:
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 254:
                    raise MyCobot320DataException("The range of 'value' in {} is 0 ~ 254, but the received value is {}".format(parameter, value))
            elif parameter in ["gripper_p", "gripper_d"]:
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 254:
                    raise MyCobot320DataException("The range of 'value' in {} is 0 ~ 254, but the received value is {}".format(parameter, value))
            elif parameter == 'pinch_pose':
                check_0_or_1(parameter, value, [0, 1, 2, 3, 4], value_type, MyCobot320DataException, int)

            elif parameter == 'rank_mode':
                pinch_pose_val = kwargs.get("pinch_pose", None)
                if pinch_pose_val == 4:
                    valid_range = list(range(1, 21))  # [1 ~ 20]
                else:
                    valid_range = list(range(0, 6))  # [0 ~ 5]
                check_0_or_1(parameter, value, valid_range, value_type, MyCobot320DataException, int)

            elif parameter == 'idle_flag':
                check_0_or_1(parameter, value, [0, 1, 2, 3, 4], value_type, MyCobot320DataException, int)

            elif parameter == 'increment_angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                span = abs(robot_limit[class_name]["angles_max"][index] - robot_limit[class_name]["angles_min"][index])

                increment_min = -span
                increment_max = span
                if value < increment_min or value > increment_max:
                    raise MyCobot320DataException("increment angle value not right, should be {0} ~ {1}, but received {2}".format(increment_min, increment_max,value))

            elif parameter == 'increment_coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID
                span = abs(robot_limit[class_name]["coords_max"][index] - robot_limit[class_name]["coords_min"][index])

                increment_min = -span
                increment_max = span
                if value < increment_min or value > increment_max:
                    raise MyCobot320DataException(
                        "Coordinate increment value not right, should be {0} ~ {1}, but received {2}".format(increment_min, increment_max,value))
            else:
                public_check(parameter_list, kwargs, robot_limit, class_name, MyCobot320DataException)
    elif class_name in ["MechArm"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MechArmDataException)
    elif class_name in ["MechArm270", "MechArmSocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MechArmDataException)
            elif parameter == 'servo_data_id' and value not in [1, 2, 3, 4, 5, 6, 7]:
                raise MechArmDataException(
                    "The id not right, should be in {0}, but received {1}.".format([1, 2, 3, 4, 5, 6, 7], value))
            elif parameter == 'address_id' and value in range(0, 5):
                raise MechArmDataException(
                    "The address_id is invalid: should not be in [0, 1, 2, 3, 4], but received {}.".format(value))
            elif parameter == 'rgb':
                check_rgb_value(value, MechArmDataException, class_name)
            elif parameter == 'value':
                check_value_type(parameter, value_type, MechArmDataException, int)
                if value < 0 or value > 4096:
                    raise MechArmDataException(
                        "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MechArmDataException, int)
                if value not in [0, 1, 2]:
                    raise MechArmDataException(
                        "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(
                            parameter,
                            value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MechArmDataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MechArmDataException, int)
                if not 1 <= value <= 100:
                    raise MechArmDataException(
                        "speed value not right, should be 1 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'flag':
                if value is not None:
                    check_0_or_1(parameter, value, [0, 1, 254], value_type, MechArmDataException, int)
            elif parameter == 'gripper_type':
                if value is not None:
                    check_0_or_1(parameter, value, [1, 3, 4, 5], value_type, MechArmDataException, int)
            elif parameter == '_type_1':
                if value is not None:
                    check_0_or_1(parameter, value, [1, 2, 3, 4, 5], value_type, MechArmDataException, int)
                # if value not in [0, 1, 10]:
                #     raise exception_class("The data supported by parameter {} is 0 or 1 or 10, but the received value is {}".format(parameter, value))
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MechArmDataException, int)
                if value < 0 or value > 100:
                    raise MechArmDataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MechArmDataException, str)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MechArmDataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MechArmDataException, int)
                if not 1 <= value <= 100:
                    raise MechArmDataException(
                        "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                    )
            if parameter == "angles":
                check_angles(value, robot_limit, class_name, MechArmDataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][
                            index]:
                    raise MechArmDataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            elif parameter == "coords":
                check_coords(parameter, value, robot_limit, class_name, MechArmDataException)
            elif parameter == 'coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > \
                        robot_limit[class_name]["coords_max"][index]:
                    raise MechArmDataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],
                            robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )
            elif parameter == 'encoders':
                if len(value) != 6:
                    raise MechArmDataException("The length of `encoders` must be 6.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MechArmDataException, int)
                    if data < 0 or data > 4096:
                        raise MechArmDataException(
                            "The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
            elif parameter == 'speeds':
                if len(value) not in [6, 7]:
                    raise MechArmDataException(
                        "The length of `speeds` must be 6. but the received value is {}".format(value))
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MechArmDataException, int)
                    if data < 0 or data > 6000:
                        raise MechArmDataException(
                            "The range of speed is 0 ~ 6000, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MechArmDataException, int)
                if value < 1 or value > 7:
                    raise MechArmDataException("The range of id is 1 ~ 6 or 7, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MechArmDataException(
                        "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MechArmDataException(
                        "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max,
                                                                                         value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MechArmDataException, int)
            elif parameter == 'is_torque':
                if value is not None:
                    check_0_or_1(parameter, value, [0, 1], value_type, MechArmDataException, int)
            else:
                public_check(parameter_list, kwargs, robot_limit, class_name, MechArmDataException)
    elif class_name in ["MyArm", "MyArmSocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyArmDataException)
            elif parameter == 'rgb':
                check_rgb_value(value, MyArmDataException, class_name)
            elif parameter == 'address_id' and value in range(0, 5):
                raise MyArmDataException(
                    "The address_id is invalid: should not be in [0, 1, 2, 3, 4], but received {}.".format(value))
            elif parameter == 'value':
                check_value_type(parameter, value_type, MyArmDataException, int)
                if value < 0 or value > 4096:
                    raise MyArmDataException(
                        "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MyArmDataException, int)
                if value not in [0, 1, 2]:
                    raise MyArmDataException(
                        "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(
                            parameter,
                            value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MyArmDataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyArmDataException, int)
                if not 1 <= value <= 100:
                    raise MyArmDataException(
                        "speed value not right, should be 1 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MyArmDataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MyArmDataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MyArmDataException, int)
                # if value not in [0, 1, 10]:
                #     raise exception_class("The data supported by parameter {} is 0 or 1 or 10, but the received value is {}".format(parameter, value))
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyArmDataException, int)
                if value < 0 or value > 100:
                    raise MyArmDataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyArmDataException, str)
            elif parameter == 'coords':
                check_coords(parameter, value, robot_limit, class_name, MyArmDataException)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyArmDataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MyArmDataException, int)
                if not 1 <= value <= 100:
                    raise MyArmDataException(
                        "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                    )
            elif parameter == "angles":
                if not isinstance(value, list):
                    raise MyArmDataException("`angles` must be a list.")
                # Check angles
                if len(value) != 7:
                    raise MyArmDataException("The length of `angles` must be 7.")
                for idx, angle in enumerate(value):
                    if not robot_limit[class_name]["angles_min"][idx] <= angle <= robot_limit[class_name]["angles_max"][
                        idx]:
                        raise MyArmDataException(
                            "Has invalid angle value, error on index {0}. Received {3} but angle should be {1} ~ {2}.".format(
                                idx, robot_limit[class_name]["angles_min"][idx],
                                robot_limit[class_name]["angles_max"][idx], angle
                            )
                        )
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][
                            index]:
                    raise MyArmDataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            elif parameter == 'coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > \
                        robot_limit[class_name]["coords_max"][index]:
                    raise MechArmDataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],
                            robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )
            elif parameter == 'encoders':
                if len(value) != 7:
                    raise MyArmDataException("The length of `encoders` must be 7.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyArmDataException, int)
                    if data < 0 or data > 4096:
                        raise MyArmDataException(
                            "The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
            elif parameter == 'speeds':
                if len(value) != 7:
                    raise MyArmDataException("The length of `speeds` must be 7.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyArmDataException, int)
                    if data < 0 or data > 6000:
                        raise MyArmDataException(
                            "The range of speed is 0 ~ 6000, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyArmDataException, int)
                if value < 1 or value > 8:
                    raise MyArmDataException("The range of id is 1 ~ 7 or 8, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MyArmDataException(
                        "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MyArmDataException(
                        "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max,
                                                                                         value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MyArmDataException, int)
    elif class_name in ["MyPalletizer260", "MyPalletizerSocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyPalletizer260DataException)
            elif parameter == 'servo_data_id' and value not in [1, 2, 3, 4, 7]:
                raise MyPalletizer260DataException(
                    "The id not right, should be in {0}, but received {1}.".format([1, 2, 3, 4, 7], value))
            elif parameter == 'address_id' and value in range(0, 5):
                raise MyCobot280DataException(
                    "The address_id is invalid: should not be in [0, 1, 2, 3, 4], but received {}.".format(value))
            elif parameter == 'rgb':
                check_rgb_value(value, MyPalletizer260DataException, class_name)
            elif parameter == 'value':
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if value < 0 or value > 4096:
                    raise MyPalletizer260DataException(
                        "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if value not in [0, 1, 2]:
                    raise MyPalletizer260DataException(
                        "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(
                            parameter,
                            value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MyPalletizer260DataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if not 1 <= value <= 100:
                    raise MyPalletizer260DataException(
                        "speed value not right, should be 1 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MyPalletizer260DataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MyPalletizer260DataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4, 5], value_type, MyPalletizer260DataException, int)
                # if value not in [0, 1, 10]:
                #     raise exception_class("The data supported by parameter {} is 0 or 1 or 10, but the received value is {}".format(parameter, value))
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if value < 0 or value > 100:
                    raise MyPalletizer260DataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyPalletizer260DataException, str)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyPalletizer260DataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if not 1 <= value <= 100:
                    raise MyPalletizer260DataException(
                        "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                    )
            elif parameter == "angles":
                if not isinstance(value, list):
                    raise MyPalletizer260DataException("`angles` must be a list.")
                # Check angles
                if len(value) != 4:
                    raise MyPalletizer260DataException("The length of `angles` must be 4.")
                for idx, angle in enumerate(value):
                    if not robot_limit[class_name]["angles_min"][idx] <= angle <= robot_limit[class_name]["angles_max"][
                        idx]:
                        raise MyPalletizer260DataException(
                            "Has invalid angle value, error on index {0}. Received {3} but angle should be {1} ~ {2}.".format(
                                idx, robot_limit[class_name]["angles_min"][idx],
                                robot_limit[class_name]["angles_max"][idx], angle
                            )
                        )
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][
                            index]:
                    raise MyPalletizer260DataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            elif parameter == "coords":
                if not isinstance(value, list):
                    raise MyPalletizer260DataException("`coords` must be a list.")
                if len(value) != 4:
                    raise MyPalletizer260DataException("The length of `coords` must be 4.")
                for idx, coord in enumerate(value):
                    if not robot_limit[class_name]["coords_min"][idx] <= coord <= robot_limit[class_name]["coords_max"][
                        idx]:
                        raise MyPalletizer260DataException(
                            "Has invalid coord value, error on index {0}. received {3} .but angle should be {1} ~ {2}.".format(
                                idx, robot_limit[class_name]["coords_min"][idx],
                                robot_limit[class_name]["coords_max"][idx], coord
                            )
                        )
            elif parameter == 'coord':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > \
                        robot_limit[class_name]["coords_max"][index]:
                    raise MechArmDataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],
                            robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )
            elif parameter == 'encoders':
                if len(value) != 4:
                    raise MyPalletizer260DataException("The length of `encoders` must be 4.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyPalletizer260DataException, int)
                    if data < 0 or data > 4096:
                        raise MyPalletizer260DataException(
                            "The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
            elif parameter == 'speeds':
                if len(value) != 4:
                    raise MyPalletizer260DataException("The length of `speeds` must be 4.")
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyPalletizer260DataException, int)
                    if data < 0 or data > 6000:
                        raise MyPalletizer260DataException(
                            "The range of speed is 0 ~ 6000, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if value < 1 or (value > 4 and value != 7):
                    raise MyPalletizer260DataException(
                        "The range of id is 1 ~ 4 or 7, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MyPalletizer260DataException(
                        "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MyPalletizer260DataException(
                        "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max,
                                                                                         value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MyPalletizer260DataException, int)
            else:
                public_check(parameter_list, kwargs, robot_limit, class_name, MyPalletizer260DataException)
    elif class_name in ["MyArmM", "MyArmC", "MyArmMControl"]:
        class_name = kwargs.pop("class_name", None)
        limit_info = robot_limit[class_name]
        for parameter in parameter_list[1:]:
            value = kwargs[parameter]
            value_type = type(value)
            if parameter in ("servo_id", "joint_id", "coord_id") and value not in limit_info[parameter]:
                raise ValueError(
                    f"The {parameter} not right, should be in {limit_info[parameter]}, but received {value}.")
            elif parameter == 'angle':
                i = kwargs['joint_id'] - 1
                min_angle = limit_info["angles_min"][i]
                max_angle = limit_info["angles_max"][i]
                if value < min_angle or value > max_angle:
                    raise ValueError(f"angle value not right, should be {min_angle} ~ {max_angle}, but received {value}")
            elif parameter == 'angles':
                if not value:
                    raise ValueError("angles value can't be empty")

                joint_length = len(limit_info["joint_id"])
                if len(value) != joint_length:
                    raise ValueError(f"The length of `angles` must be {joint_length}.")

                for i, v in enumerate(value):
                    min_angle = limit_info["angles_min"][i]
                    max_angle = limit_info["angles_max"][i]
                    if v < min_angle or v > max_angle:
                        raise ValueError(
                            f"angle value not right, should be {min_angle} ~ {max_angle}, but received {v}")
            elif parameter == 'coord':
                coord_index = kwargs['coord_id'] - 1
                min_coord = limit_info["coord_min"][coord_index]
                max_coord = limit_info["coord_max"][coord_index]
                if not min_coord <= value <= max_coord:
                    raise ValueError(f"coord value not right, should be {min_coord} ~ {max_coord}, but received {value}")
            elif parameter == 'coords':
                if len(value) != 6:
                    raise ValueError("The length of `coords` must be 6.")

                for i, v in enumerate(value):
                    min_coord = limit_info["coord_min"][i]
                    max_coord = limit_info["coord_max"][i]
                    if not min_coord <= v <= max_coord:
                        raise ValueError(f"coord value not right, should be {min_coord} ~ {max_coord}, but received {v}")
            elif parameter == 'encoder':
                i = kwargs['servo_id'] - 1
                max_encoder = limit_info["encoders_max"][i]
                min_encoder = limit_info["encoders_min"][i]
                if value < min_encoder or value > max_encoder:
                    raise ValueError(
                        f"angle value not right, should be {min_encoder} ~ {max_encoder}, but received {value}")
            elif parameter == 'encoders':
                if len(value) != 8:
                    raise ValueError("The length of `encoders` must be 8.")

                for i, v in enumerate(value):
                    max_encoder = limit_info["encoders_max"][i]
                    min_encoder = limit_info["encoders_min"][i]
                    if v < min_encoder or v > max_encoder:
                        raise ValueError(
                            f"encoder value not right, should be {min_encoder} ~ {max_encoder}, but received {v}")

                if (2048 - value[1]) + (2048 - value[2]) != 0:
                    raise ValueError("The 2 and 3 servo encoder values must be reversed")

            elif parameter == "speed":
                check_value_type(parameter, value_type, TypeError, int)
                if not 1 <= value <= 100:
                    raise ValueError(f"speed value not right, should be 1 ~ 100, the received speed is {value}")
            elif parameter == "speeds":
                assert len(value) == 8, "The length of `speeds` must be 8."
                for i, s in enumerate(value):
                    if not 1 <= s <= 100:
                        raise ValueError(f"speed value not right, should be 1 ~ 100, the received speed is {value}")
            elif parameter == "servo_addr":
                if not isinstance(value, int):
                    raise TypeError(f"The {parameter} must be an integer.")

                if value in (0, 1, 2, 3, 4):
                    if class_name == "MyArmMControl":
                        raise ValueError("modification is not allowed between 0~4, current data id: {}".format(value))
                    raise ValueError("addr 0-4 cannot be modified")
            elif parameter == "account":
                pass

            elif parameter == "password":
                if not re.match(r'^[A-Za-z0-9]{8,63}$', value):
                    raise ValueError("The password must be 8-63 characters long and contain only letters and numbers.")
            elif parameter == "pin_number":
                pin = 6
                if class_name == "MyArmC":
                    pin = 2
                if not 1 <= value <= pin:
                    raise ValueError(f"The pin number must be between 1 and {pin}.")

            elif parameter in ("direction", "mode", "pin_signal", "is_linear", "move_type", "rftype"):
                if not isinstance(value, int):
                    raise TypeError(f"The {parameter} must be an integer.")
                if value not in (0, 1):
                    raise ValueError(f"The {parameter} must be 0 or 1.")
            elif parameter == "end_direction":
                if value not in (1, 2, 3):
                    raise ValueError(f"end_direction not right, should be 1 ~ 3, the received end_direction is {value}")
            elif parameter == "gripper_flag":
                if value not in (0, 1, 254):
                    raise ValueError(f"gripper_flag not right, should be in (0, 1, 254), the received gripper_flag is {value}")
            elif parameter == "gripper_value":
                if not 0 <= value <= 100:
                    raise ValueError(f"gripper_value not right, should be 0 ~ 100, the received gripper_value is {value}")
            elif parameter == "basic_pin_number":
                if not isinstance(value, int):
                    raise TypeError(f"The {parameter} must be an integer.")

                if not 1 <= value <= 6:
                    raise ValueError("The basic pin number must be between 1 ~ 6.")
            elif parameter == "rgb":
                for v in value:
                    if not 0 <= v <= 255:
                        raise ValueError(f"rgb value not right, should be 0 ~ 255, the received rgb is {value}")

    elif class_name in ["Pro630", "Pro630Client"]:
        limit_info = robot_limit[class_name]
        for parameter, value in kwargs.items():
            value_type = type(value)
            if parameter in ("servo_id", "joint_id", "coord_id") and value not in limit_info[parameter]:
                raise ValueError(
                    f"The {parameter} not right, should be in {limit_info[parameter]}, but received {value}.")
            elif parameter == 'angle':
                i = kwargs['joint_id'] - 1
                min_angle = limit_info["angles_min"][i]
                max_angle = limit_info["angles_max"][i]
                if value < min_angle or value > max_angle:
                    raise ValueError(
                        f"angle value not right, should be {min_angle} ~ {max_angle}, but received {value}")
            elif parameter == 'angles':
                if not value:
                    raise ValueError("angles value can't be empty")

                joint_length = len(limit_info["joint_id"])
                if len(value) != joint_length:
                    raise ValueError(f"The length of `angles` must be {joint_length}.")

                for i, v in enumerate(value):
                    min_angle = limit_info["angles_min"][i]
                    max_angle = limit_info["angles_max"][i]
                    if v < min_angle or v > max_angle:
                        raise ValueError(
                            f"angle value not right, should be {min_angle} ~ {max_angle}, but received {v}")
            elif parameter == 'coord':
                coord_index = kwargs['coord_id'] - 1
                min_coord = limit_info["coords_min"][coord_index]
                max_coord = limit_info["coords_max"][coord_index]
                if not min_coord <= value <= max_coord:
                    raise ValueError(
                        f"coord value not right, should be {min_coord} ~ {max_coord}, but received {value}")
            elif parameter == 'coords':
                if len(value) != 6:
                    raise ValueError("The length of `coords` must be 6.")

                for i, v in enumerate(value):
                    min_coord = limit_info["coords_min"][i]
                    max_coord = limit_info["coords_max"][i]
                    if not min_coord <= v <= max_coord:
                        raise ValueError(
                            f"coord value not right, should be {min_coord} ~ {max_coord}, but received {v}")
            elif parameter == "speed":
                check_value_type(parameter, value_type, TypeError, int)
                if not 1 <= value <= 100:
                    raise ValueError(f"speed value not right, should be 1 ~ 100, the received speed is {value}")
            elif parameter == "speeds":
                assert len(value) == 8, "The length of `speeds` must be 8."
                for i, s in enumerate(value):
                    if not 1 <= s <= 100:
                        raise ValueError(
                            f"speed value not right, should be 1 ~ 100, the received speed is {value}")
    elif class_name in ("MyCobot280RDK-X5", ):
        robotic_limit_table = RobotLimit.robot_limit["MyCobot280RDK-X5"]
        kwargs.pop("class_name", None)
        for parameter, value in kwargs.items():
            if value is None:
                continue

            if parameter in ("servo_id", "joint_id", "coord_id"):
                servo_ids = robot_limit[class_name][parameter]
                if value not in servo_ids:
                    raise ValueError(f"The {parameter} not right, should be in {servo_ids}, but received {value}.")

            elif parameter in ('angle', 'coord'):
                if parameter == "angle":
                    _id = kwargs.get("joint_id")
                else:
                    _id = kwargs.get("coord_id")

                index = _id - 1
                minimum_position = robotic_limit_table[f"{parameter}s_min"][index]
                maximum_position = robotic_limit_table[f"{parameter}s_max"][index]
                if not minimum_position <= value <= maximum_position:
                    raise ValueError(
                        f"The {parameter} not right, should be in {minimum_position} ~ {maximum_position}, but received {value}."
                    )

            elif parameter in ('angles', 'coords'):
                if len(value) != 6:
                    raise ValueError(f"The length of `{parameter}` must be 6.")
                minimum_position = robotic_limit_table[f"{parameter}_min"]
                maximum_position = robotic_limit_table[f"{parameter}_max"]
                for index, angle in enumerate(value):
                    min_pos = minimum_position[index]
                    max_pos = maximum_position[index]
                    if not min_pos <= angle <= max_pos:
                        raise ValueError(
                            f"The {parameter} not right, should be in {min_pos} ~ {max_pos}, but received {angle}."
                        )

            elif parameter == "speed":
                if not 1 <= value <= 100:
                    raise ValueError(f"speed value not right, should be 1 ~ 100, the received speed is {value}")

            elif parameter == "speeds":
                if len(value) != 6:
                    raise ValueError("The length of `speeds` must be 6.")

                for speed in value:
                    if not 1 <= speed <= 100:
                        raise ValueError(f"speed value not right, should be 1 ~ 100, the received speed is {speed}")

            elif parameter == 'encoder':
                if not 0 <= value <= 4096:
                    raise ValueError(f"The range of encoder is 0 ~ 4096, but the received value is {value}")

            elif parameter == 'encoders':
                if not len(value) == 6:
                    raise ValueError(f"The length of encoders is 6, but the received value is {len(value)}")

                for encoder in value:
                    if not 0 <= encoder <= 4096:
                        raise ValueError(f"The range of encoder is 0 ~ 4096, but the received value is {encoder}")
            elif parameter == "drag_speeds":
                if len(value) != 6:
                    raise ValueError("The length of `speeds` must be 6.")

                for speed in value:
                    if not 1 <= speed <= 10000:
                        raise ValueError(f"speed value not right, should be 1 ~ 10000, the received speed is {speed}")

            elif parameter in (
                "monitor_state", "fresh_mode", "vision_mode", "move_type", "pin_signal", "transponder_mode",
                "reference_frame_type", "end_type", "direction", "coord_mode"
            ):
                if value not in (0, 1):
                    raise ValueError(f"The {parameter} not right, should be in (0, 1), but received {value}.")

            elif parameter == "gripper_state":
                if value not in (0, 1, 254):
                    raise ValueError(f"The {parameter} not right, should be in (0, 1, 254), but received {value}.")

            elif parameter == "gripper_speed":
                if not 0 <= value <= 100:
                    raise ValueError(f"The gripper_speed value not right, should be 0 ~ 100, the received speed is {value}")
            elif parameter == "gripper_torque":
                if not 150 <= value <= 980:
                    raise ValueError(f"The gripper_torque value not right, should be 150 ~ 980, the received speed is {value}")
            elif parameter == "gripper_type":
                gripper_types = (1, 2, 3, 4)
                if value not in gripper_types:
                    raise ValueError(f"The gripper_type not right, should be in {gripper_types}, but received {value}.")
            elif parameter == "is_torque":
                torque_gripper_types = (0, 1)
                if value not in torque_gripper_types:
                    raise ValueError(f"The is_torque not right, should be in {torque_gripper_types}, but received {value}.")
            elif parameter == "value":
                if not 0 <= value <= 4096:
                    raise ValueError(f"The value not right, should be 0 ~ 4096, but received {value}.")
            elif parameter == "protect_current":
                if not 1 <= value <= 500:
                    raise ValueError(f"The protect_current not right, should be 1 ~ 500, but received {value}.")
            elif parameter == "end_direction":
                if value not in (1, 2, 3):
                    raise ValueError(f"The end_direction not right, should be in (1, 2, 3), but received {value}.")

            elif parameter == "color":
                for color in value:
                    if not 0 <= color <= 255:
                        raise ValueError(f"The color not right, should be 0 ~ 255, but received {color}.")
    elif class_name in ["Pro450Client"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == "pin_no_base":
                check_0_or_1(parameter, value, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12], value_type, MyCobotPro450DataException, int)
            elif parameter in ["pin_no", "communicate_mode"]:
                check_0_or_1(parameter, value, [1, 2], value_type, MyCobotPro450DataException, int)
            elif parameter in ['pin_signal', 'value', 'state', 'direction', 'vr_mode', 'rftype', 'end', 'is_linear', 'mode', 'deceleration',
                               'communication_mode', 'protocol_mode', 'state']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobotPro450DataException, int)
            elif parameter == "move_type":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value not in [0, 1, 2, 3, 4]:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 4, but received {}".format(parameter, value))
            elif parameter in ['log_state']:
                check_0_or_1(parameter, value, list(range(0, 8)), value_type, MyCobotPro450DataException, int)
            elif parameter in ['max_acc']:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                mode = kwargs.get('mode', None)
                if mode == 0:
                    if not (1 <= value <= 200):
                        raise MyCobotPro450DataException(
                            f"The parameter {parameter} only supports 1 ~ 200 (angle mode), but received {value}")
                elif mode == 1:
                    if not (1 <= value <= 400):
                        raise MyCobotPro450DataException(
                            f"The parameter {parameter} only supports 1 ~ 400 (coord mode), but received {value}")
            elif parameter in ['max_speed']:
                mode = kwargs.get('mode', None)
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if mode == 0:
                    if not (1 <= value <= 150):
                        raise MyCobotPro450DataException(
                            f"The parameter {parameter} only supports 1 ~ 150 (angle mode), but received {value}")
                elif mode == 1:
                    if not (1 <= value <= 200):
                        raise MyCobotPro450DataException(
                            f"The parameter {parameter} only supports 1 ~ 200 (coord mode), but received {value}")
            elif parameter in['joint_id', 'coord_id']:
                if value not in robot_limit[class_name][parameter]:
                    check_id(value, robot_limit[class_name][parameter], MyCobotPro450DataException)
            elif parameter in ["servo_restore", "set_motor_enabled"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value not in [1, 2, 3, 4, 5, 6, 254]:
                    raise MyCobotPro450DataException(
                        "The joint_id should be in [1,2,3,4,5,6,254], but received {}".format(value))
            elif parameter in ['angle', 'degree']:
                joint_id = kwargs.get('joint_id', None)
                index = robot_limit[class_name]['joint_id'][joint_id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                    raise MyCobotPro450DataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index], value))
            elif parameter == 'coord':
                coord_id = kwargs.get('coord_id', None)
                index = robot_limit[class_name]['coord_id'][coord_id - 1] - 1  # Get the index based on the ID

                if value < robot_limit[class_name]["coords_min"][index] or value > robot_limit[class_name]["coords_max"][index]:
                    raise MyCobotPro450DataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],robot_limit[class_name]["coords_max"][index], value))
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobotPro450DataException(
                        "speed value not right, should be 1 ~ 100, the error speed is {}".format(value))
            elif parameter == "angles":
                check_angles(value, robot_limit, class_name, MyCobotPro450DataException)
            elif parameter == 'coords':
                check_coords(parameter, value, robot_limit, class_name, MyCobotPro450DataException)
            elif parameter == "rank_mode":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 4:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 4, but received {}".format(parameter, value))
            elif parameter == "get_rank_mode":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 1 or value > 4:
                    raise MyCobotPro450DataException("The parameter {} only supports 1 ~ 4, but received {}".format(parameter, value))
            elif parameter == "rank_mode_value":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 10000:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 10000, but received {}".format(parameter, value))
            elif parameter == 'rank':
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 1 or value > 5:
                    raise MyCobotPro450DataException("The parameter {} only supports 1 ~ 5, but received {}".format(parameter, value))
            elif parameter == 'rank_value':
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobotPro450DataException(
                        "rank value not right, should be 1 ~ 100, the error speed is {}".format(value))
            elif parameter == "axis":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value not in [1,2,3]:
                    raise MyCobotPro450DataException("The parameter {} only supports 1, 2, 3, but received {}".format(parameter, value))
            elif parameter == 'increment_angle':
                joint_id = kwargs.get('joint_id', None)
                index = robot_limit[class_name]['joint_id'][joint_id - 1] - 1
                span = abs(robot_limit[class_name]["angles_max"][index] - robot_limit[class_name]["angles_min"][index])

                increment_min = -span
                increment_max = span
                if value < increment_min or value > increment_max:
                    raise MyCobotPro450DataException("increment angle value not right, should be {0} ~ {1}, but received {2}".format(increment_min, increment_max,value))

            elif parameter == 'increment_coord':
                coord_id = kwargs.get('coord_id', None)
                index = robot_limit[class_name]['coord_id'][coord_id - 1] - 1  # Get the index based on the ID
                span = abs(robot_limit[class_name]["coords_max"][index] - robot_limit[class_name]["coords_min"][index])

                increment_min = -span
                increment_max = span
                if value < increment_min or value > increment_max:
                    raise MyCobotPro450DataException(
                        "Coordinate increment value not right, should be {0} ~ {1}, but received {2}".format(increment_min, increment_max,value))
            elif parameter == "threshold_value":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 50 or value > 250:
                    raise MyCobotPro450DataException("The parameter {} only supports 50 ~ 250, but received {}".format(parameter, value))
            elif parameter == "comp_value":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 250:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 250, but received {}".format(parameter, value))
            elif parameter == "trajectory":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value not in [0,1]:
                    raise MyCobotPro450DataException("The parameter {} only supports [0,1], but received {}".format(parameter, value))
            elif parameter == 'add':
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 62:
                    raise MyCobotPro450DataException(
                        "The parameter {} only supports 0 ~ 62, but received {}".format(parameter, value))
            elif parameter == 'rgb':
                check_rgb_value(value, MyCobotPro450DataException, class_name)
            elif parameter in ["gripper_id", "target_id"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 1 or value > 254:
                    raise MyCobotPro450DataException("The parameter {} only supports 1 ~ 254, but received {}".format(parameter, value))
            elif parameter in ["gripper_angle", "gripper_torque"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 100:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 100, but received {}".format(parameter, value))
            elif parameter in ["pressure_value"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 254:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 254, but received {}".format(parameter, value))
            elif parameter in ["current_value"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 100 or value > 300:
                    raise MyCobotPro450DataException("The parameter {} only supports 100 ~ 300, but received {}".format(parameter, value))
            elif parameter == "can_id":
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
            elif parameter in ["baud_rate", "timeout"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
            elif parameter in ["can_data", "data_485"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, list)
                if len(value) > 64:
                    raise MyCobotPro450DataException(
                        f"The parameter {parameter} list length exceeds 64, received {len(value)}")
                for v in value:
                    if not (0x00 <= v <= 0xFF):
                        raise MyCobotPro450DataException(
                            f"The parameter {parameter} value out of range (0x00~0xFF), received {hex(v)}")
            elif parameter in ["tool_main_version"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, str)
                if not re.fullmatch(r"\d+\.\d", str(value)):
                    raise MyCobotPro450DataException(
                        f"Invalid version format: '{value}', expected format like '1.1'")
                if float(value) < 1.0:
                    raise MyCobotPro450DataException(
                        f"Version must be >= 1.0, but received '{value}'")
            elif parameter in ["tool_modified_version"]:
                check_value_type(parameter, value_type, MyCobotPro450DataException, int)
                if value < 0 or value > 255:
                    raise MyCobotPro450DataException("The parameter {} only supports 0 ~ 255, but received {}".format(parameter, value))
            elif parameter in ["tool_coords", "world_coords"]:
                check_world_tool_coords(parameter, value, MyCobotPro450DataException)

    elif class_name in ["ultraArmP340"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == "id":
                check_0_or_1(parameter, value, [4, 7], value_type, ultraArmP340DataException, int)
            elif parameter == "system_mode":
                check_0_or_1(parameter, value, [1, 2], value_type, ultraArmP340DataException, int)
            elif parameter == "speed_mode":
                check_0_or_1(parameter, value, [0, 2], value_type, ultraArmP340DataException, int)
            elif parameter in ['mode', 'state', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, ultraArmP340DataException, int)
            elif parameter == 'joint_id':
                if value not in [1, 2, 3]:
                    check_id(value, [1, 2, 3], ultraArmP340DataException)
            elif parameter == 'axis_id':
                if value not in [1, 2, 3]:
                    raise ultraArmP340DataException(
                        "The axis_id not right, should be in [1, 2, 3], but received {}.".format(value))
            elif parameter == ["servo_restore", "set_motor_enabled"]:
                if value not in [1, 2, 3, 4, 5, 6, 254]:
                    raise MyCobotPro450DataException(
                        "The joint_id should be in [1,2,3,4,5,6,254], but received {}".format(value))
            elif parameter in ['angle']:
                joint_id = kwargs.get('joint_id', None)
                index = robot_limit[class_name]['joint_id'][joint_id - 1] - 1
                min_angle = robot_limit[class_name]["angles_min"]
                max_angle = robot_limit[class_name]["angles_max"]
                if value < min_angle[index] or value > max_angle[index]:
                    raise ultraArmP340DataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            min_angle[index], max_angle[index],value))
            elif parameter == 'coord':
                coord_id = kwargs.get('coord_id', None)
                coord_map = {"X": 0, "Y": 1, "Z": 2, "E": 3}
                index = coord_map[coord_id]

                min_val = robot_limit[class_name]["coords_min"][index]
                max_val = robot_limit[class_name]["coords_max"][index]

                if not (min_val <= value <= max_val):
                    raise ultraArmP340DataException(
                        f"Coordinate {coord_id} out of range: should be {min_val} ~ {max_val}, but received {value}")
            elif parameter == 'coord_id':
                check_value_type(parameter, value_type, ultraArmP340DataException, str)
                if value not in ["X", "Y", "Z", "E"]:
                    raise ultraArmP340DataException(
                        f"coord_id must be 'X', 'Y', or 'Z' or 'E', but received {value}")
            elif parameter == 'speed':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)

                if not (0 <= value <= 200):
                    raise ultraArmP340DataException(
                        "Speed out of range, should be 0 ~ 200, but received {}".format(value))
                if not 0 <= value <= 200:
                    raise ultraArmP340DataException(
                        "speed value not right, should be 0 ~ 200, the error speed is {}".format(value))
            elif parameter == 'wait_time':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 65535:
                    raise ultraArmP340DataException(
                        "wait time value not right, should be 0 ~ 65535, the error time is {}".format(value))
            elif parameter == 'p_value':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 255:
                    raise ultraArmP340DataException(
                        "pwm value not right, should be 0 ~ 255, the error p_value is {}".format(value))
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 100:
                    raise ultraArmP340DataException(
                        "gripper value not right, should be 0 ~ 100, the error gripper_value is {}".format(value))
            elif parameter == 'gripper_speed':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 1500:
                    raise ultraArmP340DataException(
                        "gripper speed not right, should be 0 ~ 1500, the error gripper_speed is {}".format(value))
            elif parameter == 'address':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 7 <= value <= 69:
                    raise ultraArmP340DataException(
                        "address not right, should be 7 ~ 69, the error address is {}".format(value))
            elif parameter == 'get_address':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 69:
                    raise ultraArmP340DataException(
                        "get address not right, should be 0 ~ 69, the error address is {}".format(value))
            elif parameter == 'system_value':
                check_value_type(parameter, value_type, ultraArmP340DataException, int)
                if not 0 <= value <= 4096:
                    raise ultraArmP340DataException(
                        "system value not right, should be 0 ~ 4096, the error system value is {}".format(value))
            elif parameter == "angles":
                if not isinstance(value, list):
                    raise ultraArmP340DataException("`angles` must be a list, but the received {}".format(type(value)))
                if len(value) not in [3, 4]:
                    raise ultraArmP340DataException(
                        "The length of `angles` must be 3 or 4, but received length is {}".format(len(value)))
                min_angle = robot_limit[class_name]["angles_min"]
                max_angle = robot_limit[class_name]["angles_max"]
                for idx, angle in enumerate(value):
                    if not min_angle[idx] <= angle <= max_angle[idx]:
                        raise ultraArmP340DataException(
                            "Has invalid angle value, error on index {0}. Received {3} but angle should be {1} ~ {2}.".format(
                                idx, min_angle[idx], max_angle[idx], angle))
            elif parameter == "radians":
                if not isinstance(value, list):
                    raise ultraArmP340DataException("`angles` must be a list, but the received {}".format(type(value)))
                if len(value) not in [3, 4]:
                    raise ultraArmP340DataException(
                        "The length of `radians` must be 3 or 4, but received length is {}".format(len(value)))
                min_radian = [-2.6179, -0.3490, -0.0872, -3.1241]
                max_radian = [2.9670, 1.5707, 1.9198, 3.1241]
                for idx, radian in enumerate(value):
                    if not min_radian[idx] <= radian <= max_radian[idx]:
                        raise ultraArmP340DataException(
                            "Has invalid radian value, error on index {0}. Received {3} but radian should be {1} ~ {2}.".format(
                                idx, min_radian[idx], max_radian[idx], radian))
            elif parameter == 'pose_coords':
                if not isinstance(value, list):
                    raise ultraArmP340DataException(
                        "`{}` must be a list, but the received {}".format(parameter, type(value)))
                if len(value) not in [3]:
                    raise ultraArmP340DataException(
                        "The length of `{}` must be 3, but the received length is {}".format(parameter, len(value)))
                min_coord = robot_limit[class_name]["coords_min"]
                max_coord = robot_limit[class_name]["coords_max"]
                for idx, coord in enumerate(value):
                    if not min_coord[idx] <= coord <= max_coord[idx]:
                        raise ultraArmP340DataException(
                            "Has invalid coord value, error on index {0}, received {3}, but coord should be {1} ~ {2}.".format(
                                idx, min_coord[idx], max_coord[idx], coord))
            elif parameter == 'coords':
                if not isinstance(value, list):
                    raise ultraArmP340DataException(
                        "`{}` must be a list, but the received {}".format(parameter, type(value)))
                if len(value) not in [3, 4]:
                    raise ultraArmP340DataException(
                        "The length of `{}` must be 3 or 4, but the received length is {}".format(parameter, len(value)))
                min_coord = robot_limit[class_name]["coords_min"]
                max_coord = robot_limit[class_name]["coords_max"]
                for idx, coord in enumerate(value):
                    if not min_coord[idx] <= coord <= max_coord[idx]:
                        raise ultraArmP340DataException(
                            "Has invalid coord value, error on index {0}, received {3}, but coord should be {1} ~ {2}.".format(
                                idx, min_coord[idx], max_coord[idx], coord))
            elif parameter == 'filename':
                if not isinstance(value, str):
                    raise ultraArmP340DataException(
                        "Parameter `filename` must be a string, but received {}".format(type(value)))

                if not os.path.isfile(value):
                    raise ultraArmP340DataException("The file '{}' does not exist".format(value))

                if not value.lower().endswith((".gcode", ".ngc", ".nc")):
                    raise ultraArmP340DataException(
                        "Unsupported file format, please use .gcode, .ngc, or .nc, but received {}".format(value))

def restrict_serial_port(func):
    """
    装饰器，用于限制特定串口号、socket的函数调用。
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        try:
            if hasattr(self, '_serial_port'):
                if self._serial_port.port not in ["/dev/left_arm", "/dev/right_arm"]:
                    raise MercuryRobotException(f"The {func.__name__} function cannot be called. This function is only applicable to the Mercury dual-arm robot.")
            elif hasattr(self, 'sock'):
                if not isinstance(self.sock, socket.socket):
                    raise MercuryRobotException(
                        f"The {func.__name__} function cannot be called. The connection must be a valid socket.")
            return func(self, *args, **kwargs)
        except MercuryRobotException as e:
            e = traceback.format_exc()
            print(f"MercuryRobotException: {e}")
    return wrapper
