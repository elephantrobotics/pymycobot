# Pro 450 Python Socket API
[toc]
## API usage instructions

API (Application Programming Interface), also known as Application Programming Interface functions, are predefined functions. When using the following function interfaces, please import our API library at the beginning by entering the following code, otherwise it will not run successfully:

**Note:** Before use, please make sure that the MyCobot Pro 450 server is turned on.

```python
# Example
from pymycobot import Pro450Client

mc = Pro450Client('192.168.0.232', 4500)

print(mc.get_angles())
```

### 1. System Status

#### `get_system_version()`

- **function：** get system version
- **Return value：** system version

#### `get_modified_version()`

- **function：** Read the revision number, for internal use only
- **Return value：** Correction version number

#### `get_robot_type()`

- **function：** Detection robot model

- **Return value：** Definition Rule: Actual machine model. For example, the MyCobot Pro 450 model is 4503

#### `get_atom_version()`

- **function：** Get the end version number
- **Return value：** End parameters(`float`)

#### `get_tool_modify_version()`

- **function：** Read end correction version number
- **Return value：** end correction version 

### 2. Overall Status

<!-- #### `power_on()`

- **function:** atom open communication (default open)

  - Attentions： After executing poweroff or pressing emergency stop, it takes 7 seconds to power on and restore power

- **Return value:**
  - `1` - Power on completed.
  - `0` - Power on failed

#### `power_off()`

- **function:** Power off of the robotic arm

- **Return value:**
  - `1` - Power on completed.
  - `0` - Power on failed

#### `is_power_on()`

- **function:** judge whether robot arms is powered on or not

- **Return value:**
  - `1`: power on
  - `0`: power off
  - `-1`: error -->

#### `is_init_calibration()`

- **function:** Check if the robot is initialized for calibration

- **Return value:** `bool`: True if the robot is initialized for calibration, False otherwise

#### `get_fresh_mode()`

- **function:** Query sports mode

- **Return value:** 
  - `0`: Interpolation mode
  - `1`: Refresh mode

#### `set_fresh_mode()`

- **function:** Set command refresh mode
  
- **Parameters:**
  - `1`: Always execute the latest command first.
  - `0`: Execute instructions sequentially in the form of a queue.

#### `get_debug_state()`

- **Function:** Get the current robot's debug logging mode.

- **Return value:** `int`: Current debug logging state.
  - `0`: No debug logs
  - `1`: General debug log only (_debug.log)
  - `2`: Motion-related log only (_move.log)
  - `3`: General and motion-related logs (_debug.log + _move.log)
  - `4`: Motor read/control frequency log only (_clock_rate_debug.log)
  - `5`: General and motor frequency logs (_debug.log + _clock_rate_debug.log)
  - `6`: Motion and motor frequency logs (_move.log + _clock_rate_debug.log)
  - `7`: All logs

#### `set_debug_state(log_state)`

- **Function:** Set the debug logging mode for the current robot.

- **Parameters:**
  - `log_state`: `int`, debug log state (0 to 7)
    - `0`: Do not log any debug logs
    - `1`: General debug log only (_debug.log)
    - `2`: Motion-related log only (_move.log)
    - `3`: General and motion-related logs (_debug.log + _move.log)
    - `4`: Motor read/control frequency log only (_clock_rate_debug.log)
    - `5`: General and motor frequency logs (_debug.log + _clock_rate_debug.log)
    - `6`: Motion and motor frequency logs (_move.log + _clock_rate_debug.log)
    - `7`: Log all logs

- **Return value**: `int`
  - 1 - Success
  - 0 - Failure
  - 1 - Error

### 3.Robot abnormal control

#### `get_robot_status()`

- **function:** Upper computer error security status
- **Return value:** 0 - Normal. other - Robot triggered collision detection

#### `servo_restore(joint_id)`

- **function**：Clear joint abnormalities
- **Parameters**：
  - `joint_id`: int. joint id 1 - 6, 254-All joints restored.

#### `get_comm_error_counts(joint_id)`

- **function**：Read the number of communication exceptions
- **Parameters**：
  - `joint_id`: int. joint id 1 - 6

### 4.MDI Mode and Operation

#### `get_angles()`

- **function:** get the degree of all joints
- **Return value**: `list  `a float list of all degree

<!-- #### `get_angle()`

- **function:** Get single joint angle
- **Parameters**： joint_id (int): 1 ~ 7
- **Return value:** Array of angles corresponding to joints -->

#### `send_angle(id, degree, speed)`

- **function:** send one degree of joint to robot arm
- **Parameters:**
  - `id`: Joint id(`genre.Angle`), range int 1-6
  - `degree`: degree value(`float`)
    | Joint Id | range |
    | ---- | ---- |
    | 1 | -165 ~ 165 |
    | 2 | -120 ~ 120 |
    | 3 | -158 ~ 158 |
    | 4 | -165 ~ 165 |
    | 5 | -165 ~ 165 |
    | 6 | -175 ~ 175 |

  - `speed`：the speed and range of the robotic arm's movement 1~100

#### `send_angles(angles, speed)`

- **function：** Send all angles to all joints of the robotic arm
- **Parameters:**
  - `angles`: a list of degree value(`List[float]`), length 6
  - `speed`: (`int`) 1 ~ 100

#### `get_coords()`

- **function:** Obtain robot arm coordinates from a base based coordinate system
- **Return value:** a float list of coord:[x, y, z, rx, ry, rz]

#### `send_coord(id, coord, speed)`

- **function:** send one coord to robot arm
- **Parameters:**
  - `id`:send one coord to robot arm, 1-6 corresponds to [x, y, z, rx, ry, rz]
  - `coord`: coord value(`float`)
    | Coord Id | range |
    | ---- | ---- |
    | x | -466 ~ 466 |
    | y | -466 ~ 466 |
    | z | -230 ~ 614 |
    | rx | -180 ~ 180 |
    | ry | -180 ~ 180 |
    | rz | -180 ~ 180 |
  - `speed`: (`int`) 1-100

#### `send_coords(coords, speed, mode)`

- **function:**: Send overall coordinates and posture to move the head of the robotic arm from its original point to your specified point
- **Parameters:**
  - coords: ： a list of coords value `[x,y,z,rx,ry,rz]`,length6
  - speed`(int)`: 1 ~ 100

<!-- #### `pause(deceleration=False)`

- **function:** Control the instruction to pause the core and stop all movement instructions
- **Parameters:**
  - deceleration: ： Whether to slow down and stop. Defaults to False.
- **Return value**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `is_paused()`

- **function:** Check if the program has paused the move command
- **Return value:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **function:** resume the robot movement and complete the previous command -->

#### `stop(deceleration=False)`

- **function:** stop all movements of robot
- **Parameters:**
  - deceleration: ： Whether to slow down and stop. Defaults to False.
- **Return value**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

<!-- #### `is_in_position(data, flag)`

- **function** : judge whether in the position.
- **Parameters:**
  - data: Provide a set of data that can be angles or coordinate values. If the input angle length range is 7, and if the input coordinate value length range is 6
  - flag data type (value range 0 or 1)
    - `0`: angle
    - `1`: coord
- **Return value**:
  - `1` - true
  - `0` - false
  - `-1 ` - error -->

#### `is_moving()`

- **function:** judge whether the robot is moving
- **Return value:**
  - `1` moving
  - `0` not moving
  - `-1` error

<!-- ### 4. JOG Mode and Operation

#### `jog_angle(joint_id, direction, speed)`

- **function:** jog control angle
- **Parameters**:
  - `joint_id`: Represents the joints of the robotic arm, represented by joint IDs ranging from 1 to 7
  - `direction(int)`: To control the direction of movement of the robotic arm, input `0` as negative value movement and input `1` as positive value movement
  - `speed`: 1 ~ 100

#### `jog_coord(coord_id, direction, speed)`

- **function:** jog control coord.
- **Parameters:**
  - `coord_id`: (`int`) Coordinate range of the robotic arm: 1~6
  - `direction`:(`int`) To control the direction of machine arm movement, `0` - negative value movement, `1` - positive value movement
  - `speed`: 1 ~ 100

#### `jog_increment_angle(joint_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: 1-7
  - `increment`: Incremental movement based on the current position angle
  - `speed`: 1 ~ 100

#### `jog_increment_coord(coord_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: axis id 1 - 6.
  - `increment`: Incremental movement based on the current position coord
  - `speed`: 1 ~ 100 -->

<!-- ### 5. Coordinate controlled attitude deviation angle

#### `get_solution_angles()`

- **function:** Obtain the value of zero space deflection angle
- **Return value**：Zero space deflection angle value

#### `set_solution_angles(angle, speed)`

- **function:** Obtain the value of zero space deflection angle

- **Parameters:**

  - ` angle` : Input the angle range of joint 1, angle range -90 to 90

  - `speed` : 1 - 100. -->

### 5. Joint software limit operation

#### `get_joint_min_angle(joint_id)`

- **function:** Read the minimum joint angle
- **Parameters:**
  - ` joint_id` : Enter joint ID (range 1-6)
- **Return value**：`float` Angle value

#### `get_joint_max_angle(joint_id)`

- **function:** Read the maximum joint angle
- **Parameters:**
  - ` joint_id` : Enter joint ID (range 1-6)
- **Return value:** `float` Angle value

#### `set_joint_min(id, angle)`

- **function:** Set minimum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 1-6)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be less than the minimum value

#### `set_joint_max(id, angle)`

- **function:** Set minimum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 1-6)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be greater than the maximum value

### 6. Joint motor auxiliary control

#### `get_servo_encoders()`

- **function**：Read the full joint encoder value
- **Return value**： A list of length 6

<!-- #### `is_servo_enable(servo_id)`

- **function:** Detecting joint connection status
- **Parameters:** ` servo id` 1-7
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `is_all_servo_enable()`

- **function:** Detect the status of all joint connections
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error -->

#### `set_servo_calibration(servo_id)`

- **function:** The current position of the calibration joint actuator is the angle zero point
- **Parameters**:
  - `servo_id`: 1 - 6

#### `set_break（joint_id, value）`

- **function:** Set break point
- **Parameters**：
  - `joint_id`: int. joint id 1 - 6
  - `value`: int. 0 - disable, 1 - enable
- **Return value:** 0 : faile; 1 : success

#### `set_motor_enabled(joint_id, state`

- **function:** Set the robot torque state.(Release joint interface)
- **Parameters**：
  - `joint_id`: int. joint id 1 - 6, 254-all joints
  - `state`: int. 0 - disable, 1 - enable

### 7. Run auxiliary information

#### `get_zero_pos()`

- **function**: Read the zero encoder value
- **Return value:** `list`The value of the zero encoder for seven joints

#### `get_servo_speeds()`

- **function**：Get the movement speed of all joints
- **Return value**： unit step/s

#### `get_servo_currents()`

- **function**：Get the movement current of all joints
- **Return value**： 0 ~ 5000 mA

#### `get_servo_status()`

- **function**：Get the movement status of all joints
- **Return value**： a value of 0 means no error

### 8. Robotic arm end IO control

#### `set_digital_output(pin_no, pin_signal)`

- **Function:** Set terminal IO status
- **Parameters**
  - `pin_no` (int): Pin number, range 1 to 2
  - `pin_signal` (int): 0 / 1, 0 - low level, 1 - high level
- **Return Value:**
  - `1`: Completed

#### `get_digital_input(pin_no)`

- **Function:** Get terminal IO status
- **Parameters**: `pin_no` (int), range 1 to 2
- **Return Value**: `int` 0 / 1, 0 - low level, 1 - high level

<!-- ### 9. Robotic arm end gripper control

#### `set_gripper_state(flag, speed, _type_1=None)`

- **function**: Adaptive gripper enable

- **Parameters**:

  - `flag (int) `: 0 - open 1 - close, 254 - release

  - `speed (int)`: 1 ~ 100

  - `_type_1 (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **function**: Set the gripper value

- **Parameters**:

  - `gripper_value (int) `: 0 ~ 100

  - `speed (int)`: 1 ~ 100

  - `gripper_type (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_calibration()`

- **function**: Set the current position of the gripper to zero

#### `set_gripper_enabled(value)`

- **function**: Adaptive gripper enable setting
- **Parameters**:
  - `value` 1: Enable 0: Release

#### `set_gripper_mode(mode)`

- **function**: Set gripper mode
- **Parameters**:
  - `value` :
    - 0: Transparent transmission mode
    - 1: normal mode

#### `get_gripper_mode()`

- **function**: Get gripper mode
- **Return value**:
  - 0: Transparent transmission mode
  - 1: normal mode -->

<!-- ### 10. Button function at the end of the robot arm

#### `is_btn_clicked()`

- **function**: Get the status of the button at the end of the robot arm
- **Return value**:
  - 0: no clicked
  - 1: clicked

#### `set_color(r, g, b)`

- **function**: Set the color of the end light of the robotic arm

- **Parameters**:

  - `r (int)`: 0 ~ 255

  - `g (int)`: 0 ~ 255

  - `b (int)`: 0 ~ 255 -->

<!-- ### 11. Drag Teaching

#### `drag_teach_save()`

- **function:** Start recording and dragging teaching points.
  - Note: In order to display the best sports effect, the recording time should not exceed 90 seconds

#### `drag_teach_pause()`

- **function:** Pause sampling

#### `drag_teach_execute()`

- **function:** Start dragging the teach-in point, executing it only once. -->

<!-- ### 12. Cartesian space coordinate parameter setting

#### `set_tool_reference(coords)`

- **function:** Set tool coordinate system.
- **Parameters**：`coords`: (`list`) [x, y, z, rx, ry, rz].
- **Return value:** NULL

#### `get_tool_reference(coords)`

- **function:** Get tool coordinate system.
- **Return value:** `oords`: (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **function:** Set world coordinate system.
- **Parameters**：`coords`: (`list`) [x, y, z, rx, ry, rz].
- **Return value:** NULL

#### `get_world_reference()`

- **function:** Get world coordinate system.
- **Return value:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **function:** Set base coordinate system.
- **Parameters：**`rftype`: 0 - base 1 - tool.

#### `get_reference_frame()`

- **function:** Set base coordinate system.
- **Return value:**
  - `0` - base
  - `1` - tool.

#### `set_movement_type(move_type)`

- **function:** Set movement type.
- **Parameters**：
  - `move_type`: 1 - movel, 0 - moveJ.

#### `get_movement_type()`

- **function:** Get movement type.
- **Return value:**
  - `1` - movel
  - `0` - moveJ

#### `set_end_type(end)`

- **function:** Get end coordinate system
- **Parameters:**
  - `end (int)`: `0` - flange, `1` - tool

#### `get_end_type()`

- **function:** Obtain the end coordinate system
- **Return value:**
  - `0` - flange
  - `1` - tool

### 13. Circular motion

#### `write_move_c(transpoint, endpoint, speed)`

- function：Arc trajectory motion
- Parameters：
  `transpoint(list)`：Arc passing through point coordinates
  `endpoint (list)`：Arc endpoint coordinates
  ` speed(int)`： 1 ~ 100 -->

### 9. Bottom IO control

#### `set_base_io_output(pin_no, pin_signal)`

- **function**：Set Base IO Output
- **Parameters**：
  - `pin_no` (`int`) Pin port number, range 1 ~ 12
  - `pin_signal` (`int`): 0 - low. 1 - high

#### `get_base_io_output(pin_no)`

- **function:** Read base IO input
- **Parameters:**
  - `pin_no` (`int`) pin number, range 1 ~ 12
- **Return value:** 0 - low. 1 - high

### 10. Set up 485 communication at the end of the robotic arm

<!-- #### `tool_serial_restore()`

- **function**：485 factory reset

#### `tool_serial_ready()`

- **function:** Set up 485 communication
- **Return value:** 0 : not set 1 : Setup completed

#### `tool_serial_available()`

- **function:** Set up 485 communication
- **Return value:** 0-Normal 1-Robot triggered collision detection -->

#### `tool_serial_read_data(data_len)`

- **function:** Read fixed length data. Before reading, read the buffer length first. After reading, the data will be cleared
- **Parameters**： data_len (int): The number of bytes to be read, range 1 ~ 45
- **Return value:** 0 : not set 1 : Setup completed

#### `tool_serial_write_data()`

- **function:** End 485 sends data， Data length range is 1 ~ 45 bytes
- **Return value:** 0-Normal 1-Robot triggered collision detection

<!-- #### `tool_serial_flush()`

- **function:** Clear 485 buffer
- **Return value:** 0-Normal 1-Robot triggered collision detection

#### `tool_serial_peek()`

- **function:** View the first data in the buffer, the data will not be cleared
- **Return value:** 1 byte data

#### `tool_serial_set_baud(baud)`

- **function:** Set 485 baud rate, default 115200
- **Parameters**: baud (int): baud rate
- **Return value:** NULL

#### `tool_serial_set_timeout(max_time)`

- **function:** Set 485 timeout in milliseconds, default 30ms
- **Parameters**: max_time (int): timeout
- **Return value:** NULL -->

#### `set_over_time(timeout=1000)`

- **function:** Set the timeout (unit: ms), default is 1000ms (1 second)
- **Parameters**： timeout (int): Timeout period, in ms, range 0~65535

#### `flash_tool_firmware()`

- **function:** Burn tool firmware
<!-- - **Return value:** 0-Normal 1-Robot triggered collision detection -->

### 11. Pro force-controlled gripper

#### `get_pro_gripper_firmware_version( gripper_id=14)`

- **Function**: Read the major and minor versions of the Pro Force Control Gripper firmware.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default is 14, value range is 1 to 254.

- **Return value**: (`float`) Version number, x.x

#### `get_pro_gripper_firmware_modified_version(gripper_id=14)`

- **Function**: Read the modified version of the Pro Force Control Gripper firmware.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default is 14, value range is 1 to 254.

- **Return value**: (int) Correction version number

#### `set_pro_gripper_id(target_id, gripper_id=14)`

- **Function**: Set the force-controlled gripper ID.
- **Parameter**:
  - `target_id` (int): Range: 1 to 254.
  - `gripper_id` (int): Gripper ID, default: 14, range: 1 to 254.
- **Return value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_id(gripper_id=14)`

- **Function**: Read the force-controlled gripper ID.
- **Parameter**:
  - `gripper_id` (int): Gripper ID, default: 14, range: 1 to 254.
- **Return value**: `int` Range: 1 to 254.

#### `set_pro_gripper_angle(gripper_angle，gripper_id=14)`

- **Function**: Set the force-controlled gripper angle.
- **Parameter**:
  - `gripper_angle` (`int`): Gripper angle, value range 0 ~ 100.
  - `gripper_id` (`int`) Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `get_pro_gripper_angle(gripper_id=14)`

- **Function**: Read the angle of the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**: `int` 0 ~ 100

#### `set_pro_gripper_open(gripper_id=14)`

- **Function**: Open the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `set_pro_gripper_close(gripper_id=14)`

- **Function**: Close the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `set_pro_gripper_calibration(gripper_id=14)`

- **Function**: Set the zero position of the force-controlled gripper. (The zero position needs to be set first when using it for the first time)
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `get_pro_gripper_status(gripper_id=14)`

- **Function**: Read the gripping status of the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value:**
  - `0` - Moving.
  - `1` - Stopped moving, no object was detected.
  - `2` - Stopped moving, object was detected.
  - `3` - After the object was detected, it fell.

#### `set_pro_gripper_enabled(state, gripper_id=14)`

- **Function**: Sets the force-controlled gripper enable state.
- **Parameter**:
  - `state` (`bool`): 0 or 1, 0 - disable, 1 - enable
  - `gripper_id` (`int`): Gripper ID, default 14, range 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_torque(torque_value，gripper_id=14)`

- **Function**: Set the torque of the force-controlled gripper.
- **Parameter**:
  - `torque_value` (`int`): Torque value, value range 0 ~ 100.
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `get_pro_gripper_torque(gripper_id=14)`

- **Function**: Read the torque of the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value:** (`int`) 0 ~ 100

#### `set_pro_gripper_speed(speed，gripper_id=14)`

- **Function**: Set the force-controlled gripper speed.
- **Parameter**:
  - `speed` (int): Gripper movement speed, value range 1 ~ 100.
  - `gripper_id` (`int`) Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `get_pro_gripper_speed(speed，gripper_id=14)`

- **Function**: Read the speed of the force-controlled gripper.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 ~ 254.
- **Return value**: Gripper default movement speed, range 1 ~ 100.

#### `set_pro_gripper_abs_angle(gripper_angle，gripper_id=14)`

- **Function**: Set the absolute angle of the force-controlled gripper.
- **Parameter**:
  - `gripper_angle` (`int`): Gripper angle, value range 0 ~ 100.
  - `gripper_id` (`int`) Gripper ID, default 14, value range 1 ~ 254.
- **Return value**:
  - 0 - Failed
  - 1 - Success

#### `set_pro_gripper_io_open_angle(gripper_angle, gripper_id=14)`

- **Function**: Sets the force-controlled gripper I/O opening angle.
- **Parameter**:
  - `gripper_angle` (`int`): Gripper angle, value range 0 to 100.
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_io_open_angle(gripper_id=14)`

- **Function**: Reads the force-controlled gripper I/O opening angle.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 to 254.
- **Return value**: `int` 0 to 100

#### `set_pro_gripper_io_close_angle(gripper_angle, gripper_id=14)`

- **Function**: Sets the force-controlled gripper IO closing angle.
- **Parameter**:
  - `gripper_angle` (`int`): Gripper angle, value range 0 to 100.
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 to 254.
- **Return value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_io_close_angle(gripper_id=14)`

- **Function**: Read the force-controlled gripper IO closing angle.
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, value range 1 to 254.
- **Return value**: `int` 0 to 100

#### `set_pro_gripper_mini_pressure(pressure_value, gripper_id=14)`

- **Function**: Set the minimum actuation force of the force-controlled gripper
- **Parameter**:
  - `pressure_value` (`int`): Actuation force value, range 0 to 254.
  - `gripper_id` (`int`): Gripper ID, default 14, range 1 to 254.
- **Return value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_mini_pressure(gripper_id=14)`

- **Function**: Read the minimum actuation force of the force-controlled gripper
- **Parameter**:
  - `gripper_id` (`int`): Gripper ID, default 14, range 1 to 254.
- **Return value**: (`int`) Starting force value, range 0 to 254.

#### `set_pro_gripper_protection_current(current_value, gripper_id=14)`

- **Function**: Set the gripping current of the force-controlled gripper
- **Parameter**:
  - `current_value` (`int`): Gripping current value, range 100 to 300.
  - `gripper_id` (`int`) Gripper ID, default 14, range 1 to 254.
- **Return value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_protection_current(gripper_id=14)`

- **Function**: Read the gripping current of the force-controlled gripper
- **Parameter**:
  - `gripper_id` (`int`) Gripper ID, default 14, range 1 to 254.
- **Return value**: (`int`) Clamping current value, range 100 ~ 300.