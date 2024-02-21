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

class MyBuddyDataException(Exception):
    pass


def check_boolean(b):
    if b != 0 and b != 1:
        raise MyCobotDataException("This parameter needs to be 0 or 1")
    
def check_rgb_value(value, exception_class, class_name):
    rgb_str = ["r", "g", "b"]
    for i, v in enumerate(value):
        if not (0 <= v <= 255):
            raise exception_class(
                "The RGB value for {} needs to be 0 ~ 255, but received the {} is {}".format(class_name, rgb_str[i], v)
            )

def check_value_type(parameter, value_type, exception_class, _type):
    if value_type is not _type:
        raise exception_class("The acceptable parameter {} should be an {}, but the received {}".format(parameter, _type, value_type))
    
def check_coords(value, robot_limit, class_name, exception_class):
    if len(value) != 6:
        raise exception_class("The length of `coords` must be 6.")
    for idx, coord in enumerate(value):
        if not robot_limit[class_name]["coords_min"][idx] <= coord <= robot_limit[class_name]["coords_max"][idx]:
            raise exception_class(
                "Has invalid coord value, error on index {0}. received {3} .but angle should be {1} ~ {2}.".format(
                    idx, robot_limit[class_name]["coords_min"][idx], robot_limit[class_name]["coords_max"][idx], coord
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
                raise exception_class("The range of {} is 0 ~ 4096, but the received value is {}".format(parameter, value))
        elif parameter == 'pin_mode':
            check_value_type(parameter, value_type, exception_class, int)
            if value not in [0,1,2]:
                raise exception_class("The data supported by parameter {} is 0 or 1 or 2, but the received value is {}".format(parameter, value))
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
                raise exception_class("The range of {} is 0 ~ 100, but the received value is {}".format(parameter, value))
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
        elif parameter == 'angle':
            id = kwargs.get('id', None)
            index = robot_limit[class_name]['id'][id-1] - 1
            if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                raise exception_class(
                    "angle value not right, should be {0} ~ {1}, but received {2}".format(
                        robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index], value
                    )
                )
        elif parameter == 'encoders':
            if  "MyCobot" in class_name or "MechArm" in class_name:
                if len(value) != 6:
                    raise exception_class("The length of `encoders` must be 6.")
            elif "MyPalletizer" in class_name:
                if len(value) != 4:
                    raise exception_class("The length of `encoders` must be 4.")
            elif "MyPalletizer" in class_name:
                if len(value) != 7:
                    raise exception_class("The length of `encoders` must be 7.")
            for data in value:
                data_type = type(data)
                check_value_type(parameter, data_type, exception_class, int)
                if data < 0 or data > 4096:
                    raise exception_class("The range of encoder is 0 ~ 4096, but the received value is {}".format(data))
        elif parameter == 'speeds':
            if  "MyCobot" in class_name or "MechArm" in class_name:
                if len(value) not in [6, 7]:
                    raise exception_class("The length of `speeds` must be 6. but the received value is {}".format(value))
            elif "MyPalletizer" in class_name:
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
            # if  "MyCobot" in class_name or "MechArm" in class_name:
            #     if value < 1 or value > 6:
            #             raise exception_class("The range of id is 1 ~ 6, but the received is {}".format(value))
            if "MyPalletizer" in class_name:
                if value < 1 or value > 4:
                        raise exception_class("The range of id is 1 ~ 4, but the received is {}".format(value))
            elif "MyArm" in class_name or "MyCobot" in class_name or "MechArm" in class_name:
                if value < 1 or value > 7:
                        raise exception_class("The range of id is 1 ~ 7, but the received is {}".format(value))
        elif parameter == "torque":
            torque_min = 150
            torque_max = 980
            if value < torque_min or value > torque_max:
                raise exception_class("The range of torque is {} ~ {}, but the received is {}".format(torque_min, torque_max, value))
        elif parameter == "current":
            current_min = 1
            current_max = 500
            if value < current_min or value > current_max:
                raise exception_class("The range of current is {} ~ {}, but the received is {}".format(current_min, current_max, value))
        elif parameter == 'end_direction':
            check_0_or_1(parameter, value, [1, 2, 3], value_type, exception_class, int)

def calibration_parameters(**kwargs):
    with open(os.path.dirname(os.path.abspath(__file__))+"/robot_limit.json") as f:
        robot_limit = json.load(f)
    parameter_list = list(kwargs.keys())
    class_name =  kwargs.get("class_name", None)
    if class_name in ["Mercury", "MercurySocket"]:
        for parameter in parameter_list[1:]:
            value = kwargs.get(parameter, None)
            if parameter == 'id' and value not in robot_limit[class_name][parameter]:
                check_id(value, robot_limit[class_name][parameter], MercuryDataException)
            elif parameter == 'angle':
                id = kwargs.get('id', None)
                if id in [11,12,13]:
                    index = robot_limit[class_name]['id'][id-4] - 4
                else:
                    index = robot_limit[class_name]['id'][id-1] - 1
                if value < robot_limit[class_name]["angles_min"][index] or value > robot_limit[class_name]["angles_max"][index]:
                    raise MercuryDataException(
                        "angle value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["angles_min"][index], robot_limit[class_name]["angles_max"][index], value
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
                if value < robot_limit[class_name]["coords_min"][index] or value > robot_limit[class_name]["coords_max"][index]:
                    raise MercuryDataException(
                        "coord value not right, should be {0} ~ {1}, but received {2}".format(
                            robot_limit[class_name]["coords_min"][index], robot_limit[class_name]["coords_max"][index], value
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
                if value not in [1,2,3,4,5,6,7,13,254]:
                    raise MercuryDataException("The joint_id should be in [1,2,3,4,5,6,7,13,254], but received {}".format(value))
            elif parameter == "data_len":
                if value < 1 or value > 45:
                    raise MercuryDataException("The parameter data_len data range only supports 1 ~ 45, but received {}".format(value))
            elif parameter == "max_time":
                if value < 0:
                    raise MercuryDataException("The parameter max_time must be greater than or equal to 0, but received {}".format(value))
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
                    raise MyAgvDataException("The range of direction_1 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'direction_2':
                if value < 0 or value > 255:
                    raise MyAgvDataException("The range of direction_2 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'direction_3':
                if value < 0 or value > 255:
                    raise MyAgvDataException("The range of direction_3 is 0 ~ 255, but the received value is {}".format(value))
            elif parameter == 'data':
                if value < 1 or value > 127:
                    raise MyAgvDataException("The range of {} is 1 ~ 127, but the received value is {}".format(parameter, value))
    
    elif class_name in ["MyCobot", "MyCobotSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MyCobotDataException)
    elif class_name in ["MechArm", "MechArmSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MechArmDataException)
    elif class_name in ["MyArm", "MyArmSocket"]:
        public_check(parameter_list, kwargs, robot_limit, class_name, MyArmDataException)