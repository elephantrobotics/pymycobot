# coding=utf-8
import json
import os

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


def check_coords(value, robot_limit, class_name, exception_class):
    if not isinstance(value, list):
        raise exception_class("`coords` must be a list.")
    if len(value) != 6:
        raise exception_class("The length of `coords` must be 6.")
    for idx, coord in enumerate(value):
        if not robot_limit[class_name]["coords_min"][idx] <= coord <= robot_limit[class_name]["coords_max"][idx]:
            raise exception_class(
                "Has invalid coord value, error on index {0}. received {3} .but angle should be {1} ~ {2}.".format(
                    idx, robot_limit[class_name]["coords_min"][idx], robot_limit[class_name]["coords_max"][idx], coord
                )
            )


def check_angles(angle_value, robot_limit, class_name, exception_class):
    # Check if angle_value is a list
    if not isinstance(angle_value, list):
        raise exception_class("`angles` must be a list.")
    # Check angles
    if len(angle_value) != 6:
        raise exception_class("The length of `angles` must be 6.")
    for idx, angle in enumerate(angle_value):
        if not robot_limit[class_name]["angles_min"][idx] <= angle <= robot_limit[class_name]["angles_max"][idx]:
            raise exception_class(
                "Has invalid angle value, error on index {0}. Received {3} but angle should be {1} ~ {2}.".format(
                    idx, robot_limit[class_name]["angles_min"][idx], robot_limit[class_name]["angles_max"][idx], angle
                )
            )


def check_0_or_1(parameter, value, range_data, value_type, exception_class, _type):
    check_value_type(parameter, value_type, exception_class, _type)
    if value not in range_data:
        error = "The data supported by parameter {} is ".format(parameter)
        lens = len(range_data)
        for idx in range(lens):
            error += str(range_data[idx])
            if idx != lens - 1:
                error += " or "
        error += ", but the received value is {}".format(value)
        raise exception_class(error)


def check_id(value, id_list, exception_class):
    raise exception_class(
        "The id not right, should be in {0}, but received {1}.".format(
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
        elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
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
            id = kwargs.get('id', None)
            index = robot_limit[class_name]['id'][id - 1] - 1
            if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][
                index]:
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
                if data < 0 or data > 3400:
                    raise exception_class("The range of speed is 0 ~ 3400, but the received value is {}".format(data))
        elif parameter in ['servo_id_pdi', 'encode_id']:
            check_value_type(parameter, value_type, exception_class, int)
            if "MyCobot" in class_name or "MechArm" in class_name:
                if value < 1 or value > 6:
                    raise exception_class("The range of id is 1 ~ 6, but the received is {}".format(value))
            if class_name in ["MyPalletizer", "MyPalletizer260"]:
                if value < 1 or value > 4:
                    raise exception_class("The range of id is 1 ~ 4, but the received is {}".format(value))
            elif "MyArm" in class_name or "MyCobot" in class_name or "MechArm" in class_name:
                if value < 1 or value > 7:
                    raise exception_class("The range of id is 1 ~ 7, but the received is {}".format(value))
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


def calibration_parameters(**kwargs):
    robot_limit = {
        "Mercury": {
            "id": [1, 2, 3, 4, 5, 6, 7, 11, 12, 13],
            "angles_min": [-178, -74, -178, -180, -178, -20, -180, -60, -140, -120],
            "angles_max": [178, 130, 178, 10, 178, 273, 180, 0, 190, 120],
            "coords_min": [-466, -466, -240, -180, -180, -180],
            "coords_max": [466, 466, 531, 180, 180, 180]
        },
        "MercurySocket": {
            "id": [1, 2, 3, 4, 5, 6, 7, 11, 12, 13],
            "angles_min": [-178, -74, -178, -180, -178, -20, -180, -60, -140, -120],
            "angles_max": [178, 130, 178, 10, 178, 273, 180, 0, 190, 120],
            "coords_min": [-466, -466, -240, -180, -180, -180],
            "coords_max": [466, 466, 531, 180, 180, 180]
        },
        "MyCobot": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -150, -145, -165, -180],
            "angles_max": [168, 135, 150, 145, 165, 180],
            "coords_min": [-350, -350, -70, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobotSocket": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -150, -145, -165, -180],
            "angles_max": [168, 135, 150, 145, 165, 180],
            "coords_min": [-350, -350, -70, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot280": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -150, -145, -165, -180],
            "angles_max": [168, 135, 150, 145, 165, 180],
            "coords_min": [-350, -350, -70, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot280Socket": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -150, -145, -165, -180],
            "angles_max": [168, 135, 150, 145, 165, 180],
            "coords_min": [-350, -350, -70, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot320": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-170, -137, -151, -148, -169, -180],
            "angles_max": [170, 137, 142, 148, 169, 180],
            "coords_min": [-350, -350, -41, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot320Socket": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-170, -137, -151, -148, -169, -180],
            "angles_max": [170, 137, 142, 148, 169, 180],
            "coords_min": [-350, -350, -41, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MechArm": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-165, -90, -180, -165, -115, -175],
            "angles_max": [165, 90, 70, 165, 115, 175],
            "coords_min": [-272, -272, -36, -180, -180, -180],
            "coords_max": [272, 272, 408.9, 180, 180, 180]
        },
        "MechArm270": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-165, -90, -180, -165, -115, -175],
            "angles_max": [165, 90, 70, 165, 115, 175],
            "coords_min": [-272, -272, -36, -180, -180, -180],
            "coords_max": [272, 272, 408.9, 180, 180, 180]
        },
        "MechArmSocket": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-165, -90, -180, -165, -115, -175],
            "angles_max": [165, 90, 70, 165, 115, 175],
            "coords_min": [-272, -272, -36, -180, -180, -180],
            "coords_max": [272, 272, 408.9, 180, 180, 180]
        },
        "MyArm": {
            "id": [1, 2, 3, 4, 5, 6, 7, 8],
            "angles_min": [-160, -70, -170, -113, -170, -115, -180],
            "angles_max": [160, 115, 170, 75, 170, 115, 180],
            "coords_min": [-310, -310, -140, -180, -180, -180],
            "coords_max": [310, 310, 480, 180, 180, 180]
        },
        "MyArmSocket": {
            "id": [1, 2, 3, 4, 5, 6, 7, 8],
            "angles_min": [-160, -70, -170, -113, -170, -115, -180],
            "angles_max": [160, 115, 170, 75, 170, 115, 180],
            "coords_min": [-310, -310, -140, -180, -180, -180],
            "coords_max": [310, 310, 480, 180, 180, 180]
        },
        "MyPalletizer": {
            "id": [1, 2, 3, 4, 7],
            "angles_min": [-162, -2, -92, -180],
            "angles_max": [162, 90, 60, 180],
            "coords_min": [-260, -260, -15, -180],
            "coords_max": [260, 260, 357.58, 180]
        },
        "MyPalletizer260": {
            "id": [1, 2, 3, 4, 7],
            "angles_min": [-162, -2, -92, -180],
            "angles_max": [162, 90, 60, 180],
            "coords_min": [-260, -260, -15, -180],
            "coords_max": [260, 260, 357.58, 180]
        },
        "MyPalletizerSocket": {
            "id": [1, 2, 3, 4, 7],
            "angles_min": [-162, -2, -92, -180],
            "angles_max": [162, 90, 60, 180],
            "coords_min": [-260, -260, -15, -180],
            "coords_max": [260, 260, 357.58, 180]
        },
        "UltraArm": {
            "id": [1, 2, 3, 4, 7],
            "angles_min": [-150, -20, -5, -180],
            "angles_max": [170, 90, 110, 180],
            "coords_min": [-340, -340, 0, -180],
            "coords_max": [340, 340, 270.58, 180]
        },
        "MyBuddy": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-165, -165, -165, -165, -165, -175],
            "angles_max": [165, 165, 165, 165, 165, 175],
            "waist_angle_min": -120,
            "waist_angle_max": 120,
            "left_coords_min": [0, -40, 0, -180, -180, -180],
            "left_coords_max": [250, 260, 480, 180, 180, 180],
            "right_coords_min": [0, -260, 0, -180, -180, -180],
            "right_coords_max": [250, 40, 480, 180, 180, 180]
        },
        "MyBuddySocket": {
            "id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-165, -165, -165, -165, -165, -175],
            "angles_max": [165, 165, 165, 165, 165, 175],
            "waist_angle_min": -120,
            "waist_angle_max": 120,
            "left_coords_min": [0, -40, 0, -180, -180, -180],
            "left_coords_max": [250, 260, 480, 180, 180, 180],
            "right_coords_min": [0, -260, 0, -180, -180, -180],
            "right_coords_max": [250, 40, 480, 180, 180, 180]
        }
    }
    parameter_list = list(kwargs.keys())
    class_name = kwargs.get("class_name", None)
    if class_name in ["Mercury", "MercurySocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MercuryDataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                if id in [11, 12, 13]:
                    index = robot_limit[class_name]['id'][id - 4] - 4
                else:
                    index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][index]:
                    raise MercuryDataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index],
                            value
                        )
                    )
            # elif parameter == 'angles':
            #     if len(value) not in [7, 10]:
            #         raise MercuryDataException("The length of `angles` must be 7 or 10.")
            #     for idx, angle in enumerate(value):
            #         if not MIN_ANGLE <= angle <= MAX_ANGLE:
            #             raise MercuryDataException(
            #                 "Has invalid angle value, error on index {0}. angle should be {1} ~ {2}.".format(
            #                     idx, MIN_ANGLE, MAX_ANGLE
            #                 )
            #             )

            elif parameter == 'coord':

                index = kwargs.get('id', None) - 1
                if value < robot_limit[class_name]["coords_min"][index] or value > \
                        robot_limit[class_name]["coords_max"][index]:
                    raise MercuryDataException(
                        "coord value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index], robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )
            elif parameter == 'coords':
                check_coords(value, robot_limit, class_name, MercuryDataException)

            elif parameter == 'speed' and not 1 <= value <= 100:
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
                    raise MercuryDataException(
                        "The parameter max_time must be greater than or equal to 0, but received {}".format(value))
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
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MyCobot280DataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MyCobot280DataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MyCobot280DataException, int)
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value < 0 or value > 100:
                    raise MyCobot280DataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyCobot280DataException, str)
            elif parameter == 'coords':
                check_coords(value, robot_limit, class_name, MyCobot280DataException)
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
                    if data < 0 or data > 3400:
                        raise MyCobot280DataException(
                            "The range of speed is 0 ~ 3400, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyCobot280DataException, int)
                if value < 1 or value > 6:
                    raise MyCobot280DataException("The range of id is 1 ~ 6, but the received is {}".format(value))
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
    elif class_name in ["MyCobot320", "MyCobot320Socket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyCobot320DataException)
            elif parameter == 'rgb':
                check_rgb_value(value, MyCobot320DataException, class_name)
            elif parameter == 'value':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 4096:
                    raise MyCobot320DataException(
                        "The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
            elif parameter == 'pin_mode':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value not in [0, 1, 2]:
                    raise MyCobot320DataException(
                        "The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(
                            parameter,
                            value))
            elif parameter == 'pin_signal':
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot320DataException, int)
            elif parameter == 'speed':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot320DataException(
                        "speed value not right, should be 1 ~ 100, the received speed is %s"
                        % value
                    )
            elif parameter == 'flag':
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MyCobot320DataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MyCobot320DataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MyCobot320DataException, int)
            elif parameter == 'gripper_value':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 0 or value > 100:
                    raise MyCobot320DataException(
                        "The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
            elif parameter in ['account', 'password']:
                check_value_type(parameter, value_type, MyCobot320DataException, str)
            elif parameter == 'coords':
                check_coords(value, robot_limit, class_name, MyCobot320DataException)
            elif parameter in ['rftype', 'move_type', 'end', 'is_linear', 'status', 'mode', 'direction']:
                check_0_or_1(parameter, value, [0, 1], value_type, MyCobot320DataException, int)
            elif parameter == 'acceleration':
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if not 1 <= value <= 100:
                    raise MyCobot320DataException(
                        "{} value not right, should be 1 ~ 100, the received is {}".format(parameter, value)
                    )
            elif parameter == "angles":
                check_angles(value, robot_limit, class_name, MyCobot320DataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                index = robot_limit[class_name]['id'][id - 1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > \
                        robot_limit[class_name]["angles_max"][
                            index]:
                    raise MyCobot320DataException(
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
                    raise MyCobot320DataException(
                        "Coordinate value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index],
                            robot_limit[class_name]["coords_max"][index],
                            value
                        )
                    )

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
                    raise MyCobot320DataException(
                        "The length of `speeds` must be 6. but the received value is {}".format(value))
                for data in value:
                    data_type = type(data)
                    check_value_type(parameter, data_type, MyCobot320DataException, int)
                    if data < 0 or data > 3400:
                        raise MyCobot320DataException(
                            "The range of speed is 0 ~ 3400, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 6:
                    raise MyCobot320DataException("The range of id is 1 ~ 6, but the received is {}".format(value))
            elif parameter == "torque":
                torque_min = 150
                torque_max = 980
                if value < torque_min or value > torque_max:
                    raise MyCobot320DataException(
                        "The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
            elif parameter == "current":
                current_min = 1
                current_max = 500
                if value < current_min or value > current_max:
                    raise MyCobot320DataException(
                        "The range of current is {} ~ {}, but the received is {}".format(current_min, current_max,
                                                                                         value))
            elif parameter == 'end_direction':
                check_0_or_1(parameter, value, [1, 2, 3], value_type, MyCobot320DataException, int)
            elif parameter == "gripper_angle":
                gripper_id, gripper_angle = value
                if not isinstance(gripper_id, int) or not isinstance(gripper_angle, int):
                    raise MyCobot320DataException(
                        "Both 'gripper_id' and 'gripper_angle' in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException(
                        "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                          gripper_id))
                if gripper_angle < 0 or gripper_angle > 100:
                    raise MyCobot320DataException(
                        "The range of 'gripper_angle' in {} is 0 ~ 100, but the received value is {}".format(parameter,
                                                                                                             gripper_angle))
            elif parameter == "gripper_id":
                check_value_type(parameter, value_type, MyCobot320DataException, int)
                if value < 1 or value > 254:
                    raise MyCobot320DataException(
                        "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                          value))
            elif parameter == "torque_value":
                gripper_id, torque_value = value
                if not isinstance(gripper_id, int) or not isinstance(torque_value, int):
                    raise MyCobot320DataException(
                        "Both 'gripper_id' and 'torque_value' in {} must be integers".format(parameter))
                if gripper_id < 1 or gripper_id > 254:
                    raise MyCobot320DataException(
                        "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                          gripper_id))
                if torque_value < 100 or torque_value > 300:
                    raise MyCobot320DataException(
                        "The range of 'torque_value' in {} is 100 ~ 300, but the received value is {}".format(parameter,
                                                                                                              torque_value))

                elif parameter == "gripper_speed":
                    gripper_id, speed = value
                    if not isinstance(gripper_id, int) or not isinstance(speed, int):
                        raise MyCobot320DataException(
                            "Both 'gripper_id' and 'speed' in {} must be integers".format(parameter))
                    if gripper_id < 1 or gripper_id > 254:
                        raise MyCobot320DataException(
                            "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                              gripper_id))
                    if speed < 1 or torque_value > 100:
                        raise MyCobot320DataException(
                            "The range of 'speed' in {} is 1 ~ 100, but the received value is {}".format(parameter,
                                                                                                         speed))
                elif parameter == "set_gripper_args":
                    if len(value) != 3:
                        raise ValueError(f"Expected 3 arguments, but got {len(value)}")
                    gripper_id, address, data = value
                    if not isinstance(gripper_id, int) or not isinstance(address, int) or not isinstance(data, int):
                        raise MyCobot320DataException(
                            "All arguments in {} must be integers".format(parameter))
                    if gripper_id < 1 or gripper_id > 254:
                        raise MyCobot320DataException(
                            "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                              gripper_id))
                    invalid_addresses = [1, 2, 4, 5, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 27, 28, 32, 33,
                                         36,
                                         37, 38, 39, 40, 42, 44]
                    if address < 1 or address > 44:
                        raise MyCobot320DataException(
                            "The range of 'address' in {} is 1 ~ 44, but the received value is {}".format(parameter,
                                                                                                          address))
                    if address in invalid_addresses:
                        raise MyCobot320DataException(
                            "'address' in {} cannot be one of the following values: {}, but the received value is {}".format(
                                parameter, invalid_addresses, address))
                    # 根据 address 来处理 value
                    if address in [3, 43]:
                        if data < 1 or data > 254:
                            raise MyCobot320DataException(
                                "Error in parameter '{}': The range of 'value' for address={} is 1 ~ 254, but the received value is {}".format(
                                    parameter, address, data))
                    elif address == 10:
                        if data not in [0, 1]:
                            raise MyCobot320DataException(
                                "Error in parameter '{}': Value for address={} must be 0 or 1, but the received value is {}".format(
                                    parameter, address, data))
                    elif address in [25]:
                        if data < 0 or data > 254:
                            raise MyCobot320DataException(
                                "Error in parameter '{}': The range of 'value' for address={} is 0 ~ 254, but the received value is {}".format(
                                    parameter, address, data))
                    elif address in [21, 23]:
                        if data < 0 or data > 16:
                            raise MyCobot320DataException(
                                "Error in parameter '{}': The range of 'value' for address={} is 0 ~ 16, but the received value is {}".format(
                                    parameter, address, data))
                    elif address == 41:
                        if data < 0 or data > 100:
                            raise MyCobot320DataException(
                                "Error in parameter '{}': The range of 'value' for address={} is 0 ~ 100, but the received value is {}".format(
                                    parameter, address, data))

                elif parameter == "get_gripper_args":
                    gripper_id, address = value
                    if not isinstance(gripper_id, int) or not isinstance(address, int):
                        raise MyCobot320DataException(
                            "All arguments in {} must be integers".format(parameter))
                    if gripper_id < 1 or gripper_id > 254:
                        raise MyCobot320DataException(
                            "The range of 'gripper_id' in {} is 1 ~ 254, but the received value is {}".format(parameter,
                                                                                                              gripper_id))
                    invalid_addresses = [5, 6, 10, 15, 16, 17, 18, 19, 20, 21, 23, 25, 29, 30, 31, 34, 35, 41, 43]
                    if address < 1 or address > 44:
                        raise MyCobot320DataException(
                            "The range of 'address' in {} is 1 ~ 44, but the received value is {}".format(parameter,
                                                                                                          address))
                    if address in invalid_addresses:
                        raise MyCobot320DataException(
                            "'address' in {} cannot be one of the following values: {}, but the received value is {}".format(
                                parameter, invalid_addresses, address))
    elif class_name in ["MechArm", "MechArmSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MechArmDataException)
    elif class_name in ["MechArm270", "MechArmSocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MechArmDataException)
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
                check_0_or_1(parameter, value, [0, 1, 254], value_type, MechArmDataException, int)
            elif parameter == 'gripper_type':
                check_0_or_1(parameter, value, [1, 3, 4], value_type, MechArmDataException, int)
            elif parameter == '_type_1':
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MechArmDataException, int)
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
                check_coords(value, robot_limit, class_name, MechArmDataException)
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
                    if data < 0 or data > 3400:
                        raise MechArmDataException(
                            "The range of speed is 0 ~ 3400, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MechArmDataException, int)
                if value < 1 or value > 6:
                    raise MechArmDataException("The range of id is 1 ~ 6, but the received is {}".format(value))
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
    elif class_name in ["MyArm", "MyArmSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MyArmDataException)
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            value_type = type(value)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MyArmDataException)
            elif parameter == 'rgb':
                check_rgb_value(value, MyArmDataException, class_name)
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
                check_coords(value, robot_limit, class_name, MyArmDataException)
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
                    if data < 0 or data > 3400:
                        raise MyArmDataException(
                            "The range of speed is 0 ~ 3400, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyArmDataException, int)
                if value < 1 or value > 7:
                    raise MyArmDataException("The range of id is 1 ~ 7, but the received is {}".format(value))
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
                check_0_or_1(parameter, value, [1, 2, 3, 4], value_type, MyPalletizer260DataException, int)
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
                    if data < 0 or data > 3400:
                        raise MyPalletizer260DataException(
                            "The range of speed is 0 ~ 3400, but the received value is {}".format(data))
            elif parameter in ['servo_id_pdi', 'encode_id']:
                check_value_type(parameter, value_type, MyPalletizer260DataException, int)
                if value < 1 or value > 4:
                    raise MyPalletizer260DataException("The range of id is 1 ~ 4, but the received is {}".format(value))
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

    elif class_name in ["MyArmM", "MyArmC"]:
        class_name = kwargs.pop("class_name", None)
        limit_info = robot_limit[class_name]
        for parameter in parameter_list[1:]:
            value = kwargs[parameter]
            value_type = type(value)
            if parameter in ("servo_id", "joint_id") and value not in limit_info[parameter]:
                raise MyArmDataException(
                    "The id not right, should be in {0}, but received {1}.".format(limit_info[parameter], value)
                )
            elif parameter == 'angle':
                i = kwargs['joint_id'] - 1
                min_angle = limit_info["angles_min"][i]
                max_angle = limit_info["angles_max"][i]
                if value < min_angle or value > max_angle:
                    raise MyArmDataException(
                        f"angle value not right, should be {min_angle} ~ {max_angle}, but received {value}"
                    )
            elif parameter == 'angles':
                for i, v in enumerate(value):
                    min_angle = limit_info["angles_min"][i]
                    max_angle = limit_info["angles_max"][i]
                    if v < min_angle or v > max_angle:
                        raise MyArmDataException(
                            f"angle value not right, should be {min_angle} ~ {max_angle}, but received {v}"
                        )
            elif parameter == 'encoder':
                i = kwargs['servo_id'] - 1
                max_encoder = limit_info["encoders_max"][i]
                min_encoder = limit_info["encoders_min"][i]
                if value < min_encoder or value > max_encoder:
                    raise MyArmDataException(
                        f"angle value not right, should be {min_encoder} ~ {max_encoder}, but received {value}"
                    )
            elif parameter == 'encoders':
                for i, v in enumerate(value):
                    max_encoder = limit_info["encoders_max"][i]
                    min_encoder = limit_info["encoders_min"][i]
                    if v < min_encoder or v > max_encoder:
                        raise MyArmDataException(
                            f"encoder value not right, should be {min_encoder} ~ {max_encoder}, but received {v}"
                        )
            elif parameter == "speed":
                check_value_type(parameter, value_type, MyArmDataException, int)
                if not 1 <= value <= 100:
                    raise MyArmDataException(f"speed value not right, should be 1 ~ 100, the received speed is {value}")
            elif parameter == "speeds":
                assert len(value) == 8, "The length of `speeds` must be 8."
                for i, s in enumerate(value):
                    if not 1 <= s <= 100:
                        raise MyArmDataException(
                            f"speed value not right, should be 1 ~ 100, the received speed is {value}"
                        )
            elif parameter == "servo_addr":
                if value in (0, 1, 2, 3, 4):
                    raise MyArmDataException("addr 0-4 cannot be modified")
