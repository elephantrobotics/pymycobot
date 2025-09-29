# coding=utf-8

class Robot320Info(object):
    error_info = {
        "zh_CN": {
            "robot_error":
                {
                    0: "通信异常，请检查线路、舵机固件版本是否正常、电源是否插上、固件是否烧录正确，波特率是否正确等",
                    1: "伺服电机型号错误，需要更换电机",
                    2: "伺服电机固件版本较低，需要使用FD升级",
                    3: "伺服电机p值异常，默认32，开机正常会自动恢复",
                    4: "伺服电机D值异常，默认8，开机正常会自动恢复",
                    5: "伺服电机I值异常，默认0，开机正常会自动恢复",
                    6: "伺服电机顺时针不灵敏区参数异常，默认3，开机正常会自动恢复",
                    7: "伺服电机逆时针不灵敏区参数异常，默认3，开机正常会自动恢复",
                    8: "伺服电机相位异常，开机正常会自动恢复",
                    9: "伺服电机返回延时异常，默认0，开机正常会自动恢复",
                    10: "伺服电机最小启动力异常，默认0，开机正常会自动恢复",
                    11: "开启过温过压保护，默认44，开机正常会自动恢复",
                    12: "开启舵机LED报警，默认47，开机正常会自动恢复",
                    13: "正常值见开机自检参数，开机正常会自动恢复",
                    14: "伺服电机异常，无法控制机器运动，请查询 get_servo_status",
                    255: "未知错误"
                },
            "servo_error":
                {
                    0: "伺服电机欠压/过压，查看电压，如果为0，需要修改舵机参数；如果大于实际，可能散热片损坏",
                    1: "伺服电机磁编码异常",
                    2: "伺服电机过温",
                    3: "伺服电机过流",
                    5: "伺服电机过载",
                },
            "read_next_error":
                {
                    0: "急停被按下",
                    1: "通信有问题",
                    2: "通信不稳定",
                    3: "电压过/欠压",
                    4: "磁编码异常",
                    5: "温度过热",
                    6: "电流过流",
                    7: "负载过载",
                }
        },
        "en_US": {
            "robot_error":
                {
                    0: "Communication abnormality. Please check the wiring, servo firmware version, power supply, firmware download, baud rate, etc.",
                    1: "Servo motor model error. Replace the motor.",
                    2: "Servo motor firmware version is low. FD upgrade required.",
                    3: "Servo motor p value abnormal. Default value: 32. Automatically recovers upon power-up.",
                    4: "Servo motor D value abnormal. Default value: 8. Automatically recovers upon power-up.",
                    5: "Servo motor I value abnormal. Default value: 0. Automatically recovers upon power-up.",
                    6: "Servo motor clockwise dead zone parameter abnormal. Default value: 3. Automatically recovers upon power-up.",
                    7: "Servo motor counterclockwise dead zone parameter abnormal. Default value: 3. Automatically recovers upon power-up.",
                    8: "Servo motor phase abnormal. Automatically recovers upon power-up.",
                    9: "Servo motor return delay abnormal. Default value: 0. Automatically recovers upon power-up.",
                    10: "Servo motor minimum starting force abnormal. Default value: 0. Automatically recovers upon power-up.",
                    11: "Overtemperature and overvoltage protection is enabled, default is 44, and it will automatically recover if the power is turned on normally.",
                    12: "Servo LED alarm is enabled, default is 47, and it will automatically recover if the power is turned on normally.",
                    13: "See the power-on self-test parameters for normal values. It will automatically recover if the power is turned on normally.",
                    14: "Servo motor abnormality, unable to control machine movement, please query get_servo_status.",
                    255: "Unknown error"
                },
            "servo_error":
                {
                    0: "Servo motor under-voltage/over-voltage, check the voltage, if it is 0, you need to modify the servo parameters; if it is greater than the actual, it may be damaged",
                    1: "Servo motor magnetic encoder exception",
                    2: "Servo motor over-temperature",
                    3: "Servo motor over-current",
                    5: "Servo motor over-load",
                },
            "read_next_error":
                {
                    0: "Emergency stop pressed",
                    1: "Communication problem",
                    2: "Communication unstable",
                    3: "Overvoltage/undervoltage",
                    4: "Magnetic encoding abnormality",
                    5: "Overtemperature",
                    6: "Overcurrent",
                    7: "Load overload",
                }
        }

    }


# coding=utf-8

def _interpret_status_code(language, status_code):
    # 将状态码映射到易懂的提示信息
    status_messages = {
        "zh_CN": {
            0: "",
            1: "错误：关节1临近限位，",
            2: "错误：关节2临近限位，",
            3: "错误：关节3临近限位，",
            4: "错误：关节4临近限位，",
            5: "错误：关节5临近限位，",
            6: "错误：关节6临近限位，",
            7: "错误：关节7临近限位，",
            10: "提示：运动缓停结束。",
            11: "提示：运动急停结束。",
            32: "错误：坐标无解，请检查机器人手臂跨度是否接近极限。",
            33: "错误：直线运动无相邻解。请检查机器人手臂跨度是否接近极限。",
            34: "错误：速度融合错误。请检查机器人手臂跨度是否接近极限。",
            35: "错误：零空间运动无相邻解。请检查机器人手臂跨度是否接近极限。",
            36: "错误：奇异位置无解。请使用`send_angles()`方法离开该位置。",
            49: "错误：掉使能。请使用 `power_on()` 方法重新上电。",
            50: "错误：电机报错。",
            51: "错误：电机编码器报错。",
            52: "错误：电机执行异常。",
            53: "错误：关节位置超差。",
            64: "错误: 发送的坐标数据超出限位.",
            65: "错误：关节1位置精度异常。",
            66: "错误：关节2位置精度异常。",
            67: "错误：关节3位置精度异常。",
            68: "错误：关节4位置精度异常。",
            69: "错误：关节5位置精度异常。",
            70: "错误：关节6位置精度异常。",
            71: "错误：关节7位置精度异常。",
            81: "错误：关节1碰撞检测异常。",
            82: "错误：关节2碰撞检测异常。",
            83: "错误：关节3碰撞检测异常。",
            84: "错误：关节4碰撞检测异常。",
            85: "错误：关节5碰撞检测异常。",
            86: "错误：关节6碰撞检测异常。",
            87: "错误：关节7碰撞检测异常。",
            97: "错误：关节1 can发送失败。",
            98: "错误：关节2 can发送失败。",
            99: "错误：关节3 can发送失败。",
            100: "错误：关节4 can发送失败。",
            101: "错误：关节5 can发送失败。",
            102: "错误：关节6 can发送失败。",
            103: "错误：关节7 can发送失败。",
            113: "错误：关节1 can接收异常。",
            114: "错误：关节2 can接收异常。",
            115: "错误：关节3 can接收异常。",
            116: "错误：关节4 can接收异常。",
            117: "错误：关节5 can接收异常。",
            118: "错误：关节6 can接收异常。",
            119: "错误：关节7 can接收异常。",
            129: "错误：关节1掉使能。",
            130: "错误：关节2掉使能。",
            131: "错误：关节3掉使能。",
            132: "错误：关节4掉使能。",
            133: "错误：关节5掉使能。",
            134: "错误：关节6掉使能。",
            135: "错误：关节7掉使能。",
            145: "错误：关节1电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            146: "错误：关节2电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            147: "错误：关节3电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            148: "错误：关节4电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            149: "错误：关节5电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            150: "错误：关节6电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            151: "错误：关节7电机报错。可以使用get_motors_run_err()接口获取详细错误码。",
            161: "错误：关节1电机编码器报错。",
            162: "错误：关节2电机编码器报错。",
            163: "错误：关节3电机编码器报错。",
            164: "错误：关节4电机编码器报错。",
            165: "错误：关节5电机编码器报错。",
            166: "错误：关节6电机编码器报错。",
            167: "错误：关节7电机编码器报错。",
            193: "错误：关节1电机位置超差。",
            194: "错误：关节2电机位置超差。",
            195: "错误：关节3电机位置超差。",
            196: "错误：关节4电机位置超差。",
            197: "错误：关节5电机位置超差。",
            198: "错误：关节6电机位置超差。",
            199: "错误：关节7电机位置超差。",
            255: "错误：未知错误代码: "
        },
        "en_US": {
            0: "",
            1: "Error: Joint 1 proximity limit.",
            2: "Error: Joint 2 proximity limit.",
            3: "Error: Joint 3 proximity limit.",
            4: "Error: Joint 4 proximity limit.",
            5: "Error: Joint 5 proximity limit.",
            6: "Error: Joint 6 proximity limit.",
            7: "Error: Joint 7 proximity limit.",
            10: "Tip: Motion slow stop is over.",
            11: "Tip: Motion emergency stop is over.",
            32: "ERROR: Invkinematics no solution, please check if the robot arm span approach limit.",
            33: "Error: Straight-line motion has no adjacent solution. please check if the robot arm span approach limit",
            34: "Error: Velocity fusion error. please check if the robot arm span approach limit",
            35: "Error: Zero space motion has no adjacent solution. please check if the robot arm span approach limit",
            36: "Error: Singular position has no solution. please move away by `send_angles()`",
            49: "Error: Power failure. Please use the `power_on()` method to re-power on.",
            50: "Error: Motor error.",
            51: "Error: Motor encoder error.",
            52: "Error: Motor execution exception.",
            53: "Error：The joint position is out of tolerance.",
            64: "Error: The sent coordinates exceed the limit.",
            65: "Error: Joint 1 position precision exception.",
            66: "Error: Joint 2 position precision exception.",
            67: "Error: Joint 3 position precision exception.",
            68: "Error: Joint 4 position precision exception.",
            69: "Error: Joint 5 position precision exception.",
            70: "Error: Joint 6 position precision exception.",
            71: "Error: Joint 7 position precision exception.",
            81: "Error: Joint 1 collision detection exception.",
            82: "Error: Joint 2 collision detection exception.",
            83: "Error: Joint 3 collision detection exception.",
            84: "Error: Joint 4 collision detection exception.",
            85: "Error: Joint 5 collision detection exception.",
            86: "Error: Joint 6 collision detection exception.",
            87: "Error: Joint 7 collision detection exception.",
            97: "Error: Joint 1 can fail to send.",
            98: "Error: Joint 2 can fail to send.",
            99: "Error: Joint 3 can fail to send.",
            100: "Error: Joint 4 can fail to send.",
            101: "Error: Joint 5 can fail to send.",
            102: "Error: Joint 6 can fail to send.",
            103: "Error: Joint 7 can fail to send.",
            113: "Error: Joint 1 can receive abnormality.",
            114: "Error: Joint 2 can receive abnormality.",
            115: "Error: Joint 3 can receive abnormality.",
            116: "Error: Joint 4 can receive abnormality.",
            117: "Error: Joint 5 can receive abnormality.",
            118: "Error: Joint 6 can receive abnormality.",
            119: "Error: Joint 7 can receive abnormality.",
            129: "Error: Joint 1 is disabled.",
            130: "Error: Joint 2 is not enabled.",
            131: "Error: Joint 3 is not enabled.",
            132: "Error: Joint 4 is not enabled.",
            133: "Error: Joint 5 is not enabled.",
            134: "Error: Joint 6 is not enabled.",
            135: "Error: Joint 7 is not enabled.",
            145: "Error: Joint 1 motor reports an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            146: "Error: Joint 2 motor reports an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            147: "Error: Joint 3 motor reports an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            148: "Error: Joint 4 motor reports an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            149: "Error: Joint 5 motor reported an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            150: "Error: Joint 6 motor reported an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            151: "Error: Joint 7 motor reported an error. You can use the get_motors_run_err() interface to get the detailed error code.",
            161: "Error: Joint 1 motor encoder reported an error.",
            162: "Error: Joint 2 motor encoder reported an error.",
            163: "Error: Joint 3 motor encoder reported an error.",
            164: "Error: Joint 4 motor encoder reported an error.",
            165: "Error: Joint 5 motor encoder reported an error.",
            166: "Error: Joint 6 motor encoder reported an error.",
            167: "Error: Joint 7 motor encoder reported an error.",
            193: "Error: Joint 1 motor position is out of tolerance. ",
            194: "Error: Joint 2 motor position is out of tolerance. ",
            195: "Error: Joint 3 motor position is out of tolerance. ",
            196: "Error: Joint 4 motor position is out of tolerance. ",
            197: "Error: Joint 5 motor position is out of tolerance. ",
            198: "Error: Joint 6 motor position is out of tolerance. ",
            199: "Error: Joint 7 motor position is out of tolerance. ",
            255: "Error: Unknown error code: "
        }
    }
    return status_messages[language].get(status_code, status_messages[language].get(255) + str(status_code))


class RobotLimit:
    robot_limit = {
        "Mercury": {
            "joint_id": [1, 2, 3, 4, 5, 6, 7, 11, 12, 13],
            "angles_min": [-165, -50, -165, -165, -165, -75, -165, -55, -70, -160],
            "angles_max": [165, 120, 165, 1, 165, 255, 165, 0, 245, 160],
            "coords_min": [-459, -459, -300, -180, -180, -180],
            "coords_max": [459, 459, 542, 180, 180, 180],
            "left_coords_min": [-351.11, -272.12, -262.91, -180, -180, -180],
            "left_coords_max": [566.92, 645.91, 655.13, 180, 180, 180],
            "right_coords_min": [-351.11, -645.91, -262.91, -180, -180, -180],
            "right_coords_max": [566.92, 272.12, 655.13, 180, 180, 180]
        },
        "MercurySocket": {
            "joint_id": [1, 2, 3, 4, 5, 6, 7, 11, 12, 13],
            "angles_min": [-165, -50, -165, -165, -165, -75, -165, -55, -70, -160],
            "angles_max": [165, 120, 165, 1, 165, 255, 165, 0, 245, 160],
            "coords_min": [-459, -459, -300, -180, -180, -180],
            "coords_max": [459, 459, 542, 180, 180, 180],
            "left_coords_min": [-351.11, -272.12, -262.91, -180, -180, -180],
            "left_coords_max": [566.92, 645.91, 655.13, 180, 180, 180],
            "right_coords_min": [-351.11, -645.91, -262.91, -180, -180, -180],
            "right_coords_max": [566.92, 272.12, 655.13, 180, 180, 180]
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
            "id": [1, 2, 3, 4, 5, 6],
            "angles_min": [-168, -140, -150, -150, -155, -180],
            "angles_max": [168, 140, 150, 150, 160, 180],
            "coords_min": [-350, -350, -70, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot280RDK-X5": {
            "joint_id": [1, 2, 3, 4, 5, 6],
            "coord_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -150, -145, -155, -180],
            "angles_max": [168, 135, 150, 145, 160, 180],
            "coords_min": [-281.45, -281.45, -70, -180, -180, -180],
            "coords_max": [281.45, 281.45, 412.67, 180, 180, 180]
        },
        "MyCobot280Socket": {
            "id": [1, 2, 3, 4, 5, 6],
            "angles_min": [-168, -140, -150, -150, -155, -180],
            "angles_max": [168, 140, 150, 150, 160, 180],
            "coords_min": [-281.45, -281.45, -70, -180, -180, -180],
            "coords_max": [281.45, 281.45, 412.67, 180, 180, 180]
        },
        "MyCobot320": {
            "id": [1, 2, 3, 4, 5, 6],
            "coord_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -145, -145, -168, -180],
            "angles_max": [168, 135, 145, 145, 168, 180],
            "coords_min": [-350, -350, -41, -180, -180, -180],
            "coords_max": [350, 350, 523.9, 180, 180, 180]
        },
        "MyCobot320Socket": {
            "id": [1, 2, 3, 4, 5, 6],
            "coord_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6, 7],
            "angles_min": [-168, -135, -145, -145, -168, -180],
            "angles_max": [168, 135, 145, 145, 168, 180],
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
            "id": [1, 2, 3, 4, 5, 6],
            "angles_min": [-160, -75, -175, -155, -115, -180],
            "angles_max": [160, 120, 65, 155, 115, 180],
            "coords_min": [-272, -272, -86, -180, -180, -180],
            "coords_max": [272, 272, 408.9, 180, 180, 180]
        },
        "MechArmSocket": {
            "id": [1, 2, 3, 4, 5, 6],
            "angles_min": [-160, -75, -175, -155, -115, -180],
            "angles_max": [160, 120, 65, 155, 115, 180],
            "coords_min": [-272, -272, -86, -180, -180, -180],
            "coords_max": [272, 272, 408.9, 180, 180, 180]
        },
        "MyArm": {
            "id": [1, 2, 3, 4, 5, 6, 7, 8],
            "angles_min": [-160, -70, -170, -113, -170, -115, -180],
            "angles_max": [160, 115, 170, 75, 170, 115, 180],
            "coords_min": [-310, -310, -140, -180, -180, -180],
            "coords_max": [310, 310, 480, 180, 180, 180]
        },
        "MyArmC": {
            "joint_id": [1, 2, 3, 4, 5, 6, 7],
            "servo_id": [1, 2, 3, 4, 5, 6, 7, 8],
            "angles_min": [-165, -80, -100, -160, -90, -180],
            "angles_max": [165, 100, 80, 160, 90, 180]
        },
        "MyArmM": {
            "joint_id": [1, 2, 3, 4, 5, 6, 7],
            "servo_id": [1, 2, 3, 4, 5, 6, 7, 8],
            "angles_min": [-165, -60, -90, -150, -91, -150, -118],
            "angles_max": [165, 95, 75, 150, 88, 150, 2],
            "encoders_min": [114, 1200, 1200, 1024, 286, 1013, 307, 706],
            "encoders_max": [3981, 2991, 2991, 3002, 3788, 3048, 3788, 2068]
        },
        "MyArmMControl": {
            "joint_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6, 7, 8],
            "coord_id": [1, 2, 3, 4, 5, 6],
            "coord_min": [-833.325, -833.325, -351.11, -180, -180, -180],
            "coord_max": [833.325, 833.325, 1007.225, 180, 180, 180],
            "angles_min": [-165, -80, -100, -160, -90, -180, -118],
            "angles_max": [165, 100, 80, 160, 120, 180, 2],
            "encoders_min": [137, 1163, 1035, 1013, 248, 979, 220, 706],
            "encoders_max": [4004, 2945, 3079, 3026, 3724, 2994, 3704, 2048]
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
            "id": [1, 2, 3, 4],
            "angles_min": [-162, -2, -92, -180],
            "angles_max": [162, 90, 60, 180],
            "coords_min": [-260, -260, -15, -180],
            "coords_max": [260, 260, 357.58, 180]
        },
        "MyPalletizerSocket": {
            "id": [1, 2, 3, 4],
            "angles_min": [-162, -2, -92, -180],
            "angles_max": [162, 90, 60, 180],
            "coords_min": [-260, -260, -15, -180],
            "coords_max": [260, 260, 357.58, 180]
        },
        "UltraArm": {
            "id": [1, 2, 3, 4, 7],
            "angles_min": [-150, -20, -5, -179],
            "angles_max": [170, 90, 110, 179],
            "coords_min": [-360, -365.55, -140, -180],
            "coords_max": [365.55, 365.55, 130, 180]
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
        },
        "Pro630": {
            "joint_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6],
            "coord_id": (1, 2, 3, 4, 5, 6),
            "angles_min": [-180, -45, -165, -90, -180, -180],
            "angles_max": [180, 225, 165, 270, 180, 180],
            "coords_min": [-630, -630, -425, -180, -180, -180],
            "coords_max": [630, 630, 835, 180, 180, 180]
        },
        "Pro450Client": {
            "joint_id": [1, 2, 3, 4, 5, 6],
            "servo_id": [1, 2, 3, 4, 5, 6],
            "coord_id": (1, 2, 3, 4, 5, 6),
            "angles_min": [-165, -120, -158, -165, -165, -175],
            "angles_max": [165, 120, 158, 165, 165, 175],
            "coords_min": [-466, -466, -230, -180, -180, -180],
            "coords_max": [466, 466, 614, 180, 180, 180]
        },
        "ultraArmP340": {
            "joint_id": [1, 2, 3, 4],
            "servo_id": [1, 2, 3, 4],
            "coord_id": ['X', 'Y', 'Z', 'E'],
            "angles_min": [-150, -20, -5, -179],
            "angles_max": [170, 90, 110, 179],
            "coords_min": [-360, -365.55, -140, -180],
            "coords_max": [366.55, 365.55, 130, 180]
        }
    }
