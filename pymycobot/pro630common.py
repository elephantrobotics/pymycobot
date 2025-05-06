from enum import Enum


class Axis(Enum):
    X = 0
    Y = 1
    Z = 2
    RX = 3
    RY = 4
    RZ = 5


class Joint(Enum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3
    J5 = 4
    J6 = 5


class DI(Enum):
    """Available Digital Input Pins."""

    PIN_0 = 0
    PIN_1 = 1
    PIN_2 = 2
    PIN_3 = 3
    PIN_4 = 4
    PIN_5 = 5
    PIN_6 = 6
    PIN_7 = 7
    PIN_8 = 8
    PIN_9 = 9
    PIN_10 = 10
    PIN_11 = 11
    PIN_12 = 12
    PIN_13 = 13
    PIN_14 = 14

    CAT_PHYSICAL_START = 15
    CAT_PHYSICAL_STOP = 16
    CAT_PHYSICAL_USER_DEFINE = 17

    PIN_18 = 18
    PIN_19 = 19
    PIN_20 = 20
    PIN_21 = 21
    PIN_22 = 22
    PIN_23 = 23
    PIN_24 = 24
    PIN_25 = 25
    PIN_26 = 26
    PIN_27 = 27
    PIN_28 = 28
    PIN_29 = 29
    PIN_30 = 30
    PIN_31 = 31
    PIN_32 = 32
    PIN_33 = 33
    COLLISION_DETECTED = 33

    J1_COMMUNICATION = 34
    J2_COMMUNICATION = 35
    J3_COMMUNICATION = 36
    J4_COMMUNICATION = 37
    J5_COMMUNICATION = 38
    J6_COMMUNICATION = 39

    IO_STOP_TRIGGERED = 40
    IO_RUN_TRIGGERED = 41

    PIN_42 = 42

    HARDWARE_PAUSE_PRESSED = 43

    BRAKE_ACTIVATION_RUNNING = 44
    J1_SERVO_ENABLED = 45
    J2_SERVO_ENABLED = 46
    J3_SERVO_ENABLED = 47
    J4_SERVO_ENABLED = 48
    J5_SERVO_ENABLED = 49
    J6_SERVO_ENABLED = 50

    HARDWARE_FREE_MOVE = 51
    MOTION_ENABLE = 52

    PIN_53 = 53
    PIN_54 = 54
    PIN_55 = 55

    J1_STATUS = 56
    J2_STATUS = 57
    J3_STATUS = 58
    J4_STATUS = 59
    J5_STATUS = 60
    J6_STATUS = 61

    EMERGENCY_STOP = 62
    POWER_ON_STATUS = 63


class DO(Enum):
    """Available Digital Output Pins."""

    PIN_0 = 0
    PIN_1 = 1
    PIN_2 = 2
    PIN_3 = 3
    PIN_4 = 4
    PIN_5 = 5
    PIN_6 = 6
    PIN_7 = 7
    PIN_8 = 8
    PIN_9 = 9
    PIN_10 = 10
    PIN_11 = 11
    PIN_12 = 12
    PIN_13 = 13
    PIN_14 = 14
    PIN_15 = 15
    PIN_16 = 16
    PIN_17 = 17
    PIN_18 = 18
    PIN_19 = 19
    PIN_20 = 20
    PIN_21 = 21
    PIN_22 = 22
    PIN_23 = 23
    PIN_24 = 24
    PIN_25 = 25
    PIN_26 = 26
    PIN_27 = 27
    PIN_28 = 28
    PIN_29 = 29
    PIN_30 = 30
    PIN_31 = 31
    PIN_32 = 32
    PIN_33 = 33
    PIN_34 = 34
    PIN_35 = 35
    PIN_36 = 36
    PIN_37 = 37
    PIN_38 = 38
    PIN_39 = 39
    PIN_40 = 40
    PIN_41 = 41
    PIN_42 = 42
    PIN_43 = 43
    PIN_44 = 44
    PIN_45 = 45
    PIN_46 = 46

    BRAKE_ACTIVE_AUTO = 47
    BRAKE_MANUAL_MODE_ENABLE = 48
    J1_BRAKE_RELEASE = 49
    J2_BRAKE_RELEASE = 50
    J3_BRAKE_RELEASE = 51
    J4_BRAKE_RELEASE = 52
    J5_BRAKE_RELEASE = 53
    J6_BRAKE_RELEASE = 54

    SKIP_INIT_ERRORS = 55
    PROGRAM_AUTO_RUNNING = 56
    POWER_ON_RELAY_1 = 57
    POWER_ON_RELAY_2 = 58

    PIN_59 = 59

    SOFTWARE_FREE_MOVE = 60

    PIN_61 = 61
    PIN_62 = 62
    PIN_63 = 63


class AI(Enum):
    """Available Analog Input Pins."""

    PIN_0 = 0
    PIN_1 = 1
    PIN_2 = 2
    PIN_3 = 3
    PIN_4 = 4
    PIN_5 = 5
    PIN_6 = 6
    PIN_7 = 7
    PIN_8 = 8
    PIN_9 = 9
    PIN_10 = 10
    PIN_11 = 11
    PIN_12 = 12
    PIN_13 = 13
    PIN_14 = 14
    PIN_15 = 15
    PIN_16 = 16
    PIN_17 = 17
    PIN_18 = 18
    PIN_19 = 19
    PIN_20 = 20
    PIN_21 = 21
    PIN_22 = 22
    PIN_23 = 23
    PIN_24 = 24

    PIN_25 = 25
    AXIS_X_VELOCITY = 25
    PIN_26 = 26
    AXIS_Y_VELOCITY = 26
    PIN_27 = 27
    AXIS_Z_VELOCITY = 27
    PIN_28 = 28
    AXIS_RX_VELOCITY = 28
    PIN_29 = 29
    AXIS_RY_VELOCITY = 29
    PIN_30 = 30
    AXIS_RZ_VELOCITY = 30

    J1_VOLTAGE = 31
    J2_VOLTAGE = 32
    J3_VOLTAGE = 33
    J4_VOLTAGE = 34
    J5_VOLTAGE = 35
    J6_VOLTAGE = 36

    J1_TEMPERATURE = 37
    J2_TEMPERATURE = 38
    J3_TEMPERATURE = 39
    J4_TEMPERATURE = 40
    J5_TEMPERATURE = 41
    J6_TEMPERATURE = 42

    J1_WINDING_A_CURRENT = 43
    J2_WINDING_A_CURRENT = 44
    J3_WINDING_A_CURRENT = 45
    J4_WINDING_A_CURRENT = 46
    J5_WINDING_A_CURRENT = 47
    J6_WINDING_A_CURRENT = 48

    J1_WINDING_B_CURRENT = 49
    J2_WINDING_B_CURRENT = 50
    J3_WINDING_B_CURRENT = 51
    J4_WINDING_B_CURRENT = 52
    J5_WINDING_B_CURRENT = 53
    J6_WINDING_B_CURRENT = 54

    ROBOT = 55

    ROBOT_AVG_POWER = 56
    CONTROLLER_TEMPERATURE = 57

    J1_CURRENT = 58
    J2_CURRENT = 59
    J3_CURRENT = 60
    J4_CURRENT = 61
    J5_CURRENT = 62
    J6_CURRENT = 63


class AO(Enum):
    """Available Analog Output Pins."""

    PIN_0 = 0
    PIN_1 = 1
    PIN_2 = 2
    PIN_3 = 3
    PIN_4 = 4
    PIN_5 = 5
    PIN_6 = 6
    PIN_7 = 7
    PIN_8 = 8
    PIN_9 = 9
    PIN_10 = 10
    PIN_11 = 11
    PIN_12 = 12
    PIN_13 = 13
    PIN_14 = 14
    PIN_15 = 15

    LED_LIGHT = 16

    PIN_17 = 17
    PIN_18 = 18
    PIN_19 = 19
    PIN_20 = 20
    PIN_21 = 21
    PIN_22 = 22
    PIN_23 = 23
    PIN_24 = 24
    PIN_25 = 25
    PIN_26 = 26
    PIN_27 = 27
    PIN_28 = 28
    PIN_29 = 29
    PIN_30 = 30
    PIN_31 = 31
    PIN_32 = 32
    PIN_33 = 33
    PIN_34 = 34
    PIN_35 = 35
    PIN_36 = 36
    PIN_37 = 37
    PIN_38 = 38
    PIN_39 = 39
    PIN_40 = 40

    ACCELERATION = 41

    PIN_42 = 42
    PIN_43 = 43
    PIN_44 = 44
    PIN_45 = 45
    PIN_46 = 46
    PIN_47 = 47

    TOOL_FORCE = 48
    TOOL_SPEED = 49
    STOPPING_DISTANCE = 50
    STOPPING_TIME = 51
    POWER_LIMIT = 52

    PAYLOAD = 53

    PIN_54 = 54
    PIN_55 = 55

    J1_TORQUE = 56
    J2_TORQUE = 57
    J3_TORQUE = 58
    J4_TORQUE = 59
    J5_TORQUE = 60
    J6_TORQUE = 61

    XY_AXIS_TORQUE = 62
    Z_AXIS_TORQUE = 63
