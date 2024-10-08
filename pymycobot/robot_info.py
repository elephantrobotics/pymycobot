# coding=utf-8

class Robot320Info(object):
    error_info = {
        "zh_CN": {
            "robot_error":
                {
                    0: "通信异常，请检查线路、舵机固件版本是否正常、电源是否插上、固件是否烧录正确，波特率是否正确等",
                    1: "伺服电机型号错误，需要更换电机",
                    2: "伺服电机固件版本较低，需要使用FD升级",
                    3: "伺服电机p值异常，默认32，此异常会自动恢复",
                    4: "伺服电机D值异常，默认8，此异常会自动恢复",
                    5: "伺服电机I值异常，默认0，此异常会自动恢复",
                    6: "伺服电机顺时针不灵敏区参数异常，默认3，此异常会自动恢复",
                    7: "伺服电机逆时针不灵敏区参数异常，默认3，此异常会自动恢复",
                    8: "伺服电机相位异常，此异常会自动恢复",
                    9: "伺服电机返回延时异常，默认0，此异常会自动恢复",
                    10: "伺服电机最小启动力异常，默认0，此异常会自动恢复",
                    11: "伺服电机异常，当舵机异常时，无法控制机器运动，请查询舵机反馈接口get_servo_status，查看具体报错",
                    255: "未知错误"
                },
            "servo_error":
            {
                0: "伺服电机欠压/过压，查看电压，如果为0，需要修改舵机参数；如果大于实际，可能散热片损坏",
                1: "伺服电机磁编码异常",
                2: "伺服电机过温",
                3: "伺服电机过流",
                5: "伺服电机过载",
            }
        },
        "en_US": {
            "robot_error":
            {
                0: "Communication exception, please check the line, servo motor firmware version, power supply, firmware burning, baud rate, etc.",
                1: "Servo motor model error, need to replace the motor",
                2: "Servo motor firmware version is too low, need to use FD upgrade",
                3: "Servo motor p value exception, default 32, this exception will be automatically restored",
                4: "Servo motor D value exception, default 8, this exception will be automatically restored",
                5: "Servo motor I value exception, default 0, this exception will be automatically restored",
                6: "Servo motor clockwise insensitive zone parameter exception, default 3, this exception will be automatically restored",
                7: "Servo motor counterclockwise insensitive zone parameter exception, default 3, this exception will be automatically restored",
                8: "Servo motor phase exception, this exception will be automatically restored",
                9: "Servo motor return delay exception, default 0, this exception will be automatically restored",
                10: "Servo motor minimum starting force exception, default 0, this exception will be automatically restored",
                11: "Servo motor exception, when the servo is abnormal, the robot cannot move, please query the servo feedback interface get_servo_status, view the specific error",
                255: "Unknown error"
            },
            "servo_error":
            {
                0: "Servo motor under-voltage/over-voltage, check the voltage, if it is 0, you need to modify the servo parameters; if it is greater than the actual, it may be damaged",
                1: "Servo motor magnetic encoder exception",
                2: "Servo motor over-temperature",
                3: "Servo motor over-current",
                5: "Servo motor over-load",
            }
        }

    }
