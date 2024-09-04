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
            255: "Error: Unknown error code: "
        }
    }
    return status_messages[language].get(status_code, status_messages[language].get(255)+str(status_code))

class RobotLimit:
    robot_limit = {
        "Mercury":{
            "joint_id":[1,2,3,4,5,6,7,11,12,13],
            "angles_min":[-165, -55, -165, -180, -165, -20, -180, -60, -140, -120],
            "angles_max":[165, 95, 165, 5, 165, 273, 180, 0, 190, 120],
            "coords_min":[-441.37, -441.37, -206.52, -180, -180, -180],
            "coords_max":[441.37, 441.37, 628.02, 180, 180, 180]
        },
        "MercurySocket":{
            "joint_id":[1,2,3,4,5,6,7,11,12,13],
            "angles_min":[-165, -55, -165, -180, -165, -20, -180, -60, -140, -120],
            "angles_max":[165, 95, 165, 5, 165, 273, 180, 0, 190, 120],
            "coords_min":[-441.37, -441.37, -206.52, -180, -180, -180],
            "coords_max":[441.37, 441.37, 628.02, 180, 180, 180]
        },
        "MyCobot":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-168, -135, -150, -145, -165, -180],
            "angles_max":[168, 135, 150, 145, 165, 180],
            "coords_min":[-350, -350, -70, -180, -180, -180],
            "coords_max":[350, 350, 523.9, 180, 180, 180]
        },
        "MyCobotSocket":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-168, -135, -150, -145, -165, -180],
            "angles_max":[168, 135, 150, 145, 165, 180],
            "coords_min":[-350, -350, -70, -180, -180, -180],
            "coords_max":[350, 350, 523.9, 180, 180, 180]
        },
        "MechArm":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-165, -90, -180, -165, -115, -175],
            "angles_max":[165, 90, 70, 165, 115, 175],
            "coords_min":[-272, -272, -36, -180, -180, -180],
            "coords_max":[272, 272, 408.9, 180, 180, 180]
        },
        "MechArmSocket":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-165, -90, -180, -165, -115, -175],
            "angles_max":[165, 90, 70, 165, 115, 175],
            "coords_min":[-272, -272, -36, -180, -180, -180],
            "coords_max":[272, 272, 408.9, 180, 180, 180]
        },
        "MyArm":{
            "id":[1,2,3,4,5,6,7,8],
            "angles_min":[-160, -70, -170, -113, -170, -115, -180],
            "angles_max":[160, 115, 170, 75, 170, 115, 180],
            "coords_min":[-310, -310, -140, -180, -180, -180],
            "coords_max":[310, 310, 480, 180, 180, 180]
        },
        "MyArmSocket":{
            "id":[1,2,3,4,5,6,7,8],
            "angles_min":[-160, -70, -170, -113, -170, -115, -180],
            "angles_max":[160, 115, 170, 75, 170, 115, 180],
            "coords_min":[-310, -310, -140, -180, -180, -180],
            "coords_max":[310, 310, 480, 180, 180, 180]
        },
        "MyPalletizer":{
            "id":[1,2,3,4, 7],
            "angles_min":[-162, -2, -92, -180],
            "angles_max":[162, 90, 60, 180],
            "coords_min":[-260, -260, -15, -180],
            "coords_max":[260, 260, 357.58, 180]
        },
        "MyPalletizerSocket":{
            "id":[1,2,3,4, 7],
            "angles_min":[-162, -2, -92, -180],
            "angles_max":[162, 90, 60, 180],
            "coords_min":[-260, -260, -15, -180],
            "coords_max":[260, 260, 357.58, 180]
        },
        "UltraArm":{
            "id":[1,2,3,4, 7],
            "angles_min":[-150, -20, -5, -180],
            "angles_max":[170, 90, 110, 180],
            "coords_min":[-340, -340, 0, -180],
            "coords_max":[340, 340, 270.58, 180]
        },
        "MyBuddy":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-165, -165, -165, -165, -165, -175],
            "angles_max":[165, 165, 165, 165, 165, 175],
            "waist_angle_min":-120,
            "waist_angle_max":120,
            "left_coords_min":[0, -40, 0, -180,-180,-180],
            "left_coords_max":[250, 260, 480, 180,180,180],
            "right_coords_min":[0, -260, 0, -180,-180,-180],
            "right_coords_max":[250, 40, 480, 180,180,180]
        },
        "MyBuddySocket":{
            "id":[1,2,3,4,5,6, 7],
            "angles_min":[-165, -165, -165, -165, -165, -175],
            "angles_max":[165, 165, 165, 165, 165, 175],
            "waist_angle_min":-120,
            "waist_angle_max":120,
            "left_coords_min":[0, -40, 0, -180,-180,-180],
            "left_coords_max":[250, 260, 480, 180,180,180],
            "right_coords_min":[0, -260, 0, -180,-180,-180],
            "right_coords_max":[250, 40, 480, 180,180,180]
        }
    }
