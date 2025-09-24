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

#### `set_communication_mode(communication_mode, protocol_mode=None)`

- **Function:** Sets the current robot communication mode.

- **Parameters:**
  - `communication_mode`: `int`
    - 0 - Socket communication mode
    - 1 - 485 communication mode
  - `protocol_mode`: `int`, protocol mode, optional, default: None
    - `0`: Custom protocol
    - `1`: Modbus protocol
- **Return value**: `int`
  - 1 - Success
  - 0 - Failure
  - 1 - Error

#### `get_communication_mode()`

- **Function:** Gets the current robot communication mode.

- **Return value:**
  - `communication_mode`: `int`
    - 0 - Socket communication mode
    - 1 - 485 communication mode
  - `protocol_mode`: `int`
    - `0`: Custom protocol
    - `1`: Modbus protocol

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

#### `get_error_information()`

- **Function**: Read robot error information
- **Return value**: `int`
  - `0`: No error information
  - `1-6`: The corresponding joint exceeds the limit position.
  - `16-19`: Collision protection.
  - `32`: No solution for the inverse kinematics solution.
  - `33-34`: No adjacent solution for linear motion.

#### `clear_error_information()`

- **Function**: Clear robot error information

#### `over_limit_return_zero()`

- **Function**: Return to zero for a joint exceeding the limit

#### `get_motors_run_err()`

- **Function**: Read motor error information during robot motion
- **Return value**: `list`, a list of 6, all zeros, indicating normal operation

### 4.MDI Mode and Operation

#### `set_control_mode(mode)`

- **Function**: Set the robot motion mode
- **Parameter**:
  - `mode`: `int`, 0 to 1, default 0
    - `0`: Position mode
    - `1`: Torque mode

#### `get_control_mode()`

- **Function**: Get the robot motion mode
- **Return value**:
  - `0`: Position mode
  - `1`: Torque mode

#### `get_angles()`

- **function:** get the degree of all joints
- **Return value**: `list` a float list of all degree

#### `get_angle()`

- **function:** Get single joint angle
- **Parameters**： `joint_id` (int): 1 ~ 6
- **Return value:** `float`, single joint angle

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

#### `pause(deceleration=0)`

- **Function:** Controls the core and stops all motion commands.
- **Parameters:**
  - `deceleration`: Whether to decelerate and stop. The default value is 0. 1 indicates a deceleration.
- **Return Value**:
  - `1` - stopped
  - `0` - not stopped
  - `-1` - error

#### `is_paused()`

- **Function:** Checks whether the program has paused a motion command.
- **Return Value:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **Function:** Resume robot motion and complete the previous command.

#### `stop(deceleration=0)`

- **Function:** Stops robot motion.
- **Parameters:**
  - `deceleration`: Whether to decelerate and stop. Defaults to 0. 1 indicates a slow stop.
- **Return Value**:
  - `1` - Stopped
  - `0` - Not stopped
  - `-1` - Error

#### `is_in_position(data, flag)`

- **function** : judge whether in the position.
- **Parameters:**
  - `data`: Provide a set of data that can be angles or coordinate values. If the input angle length range is 7, and if the input coordinate value length range is 6
  - `flag`: data type (value range 0 or 1)
    - `0`: angle
    - `1`: coord
- **Return value**:
  - `1` - true
  - `0` - false
  - `-1` - error

#### `is_moving()`

- **function:** judge whether the robot is moving
- **Return value:**
  - `1` moving
  - `0` not moving
  - `-1` error

### 5. JOG Mode and Operation

#### `jog_angle(joint_id, direction, speed)`

- **function:** jog control angle
- **Parameters**:
  - `joint_id`: Represents the joints of the robotic arm, represented by joint IDs ranging from 1 to 6
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
  - `joint_id`: 1-6
  - `increment`: Incremental movement based on the current position angle
  - `speed`: 1 ~ 100

#### `jog_increment_coord(coord_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: axis id 1 - 6.
  - `increment`: Incremental movement based on the current position coord
  - `speed`: 1 ~ 100

<!-- ### 5. Coordinate controlled attitude deviation angle

#### `get_solution_angles()`

- **function:** Obtain the value of zero space deflection angle
- **Return value**：Zero space deflection angle value

#### `set_solution_angles(angle, speed)`

- **function:** Obtain the value of zero space deflection angle

- **Parameters:**

  - ` angle` : Input the angle range of joint 1, angle range -90 to 90

  - `speed` : 1 - 100. -->

### 6. Speed/Acceleration Parameters

#### `get_max_speed(mode)`

- **Function:** Get the maximum speed
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular speed
    - `1`: Coordinate speed
- **Return value**: Angular speed range: 1-150°/s, coordinate speed range: 1-200mm/s

#### `set_max_speed(mode, max_speed)`

- **Function:** Set the maximum speed
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular speed
    - `1`: Coordinate speed
  - `max_speed`: Angular speed range: 1-150°/s, coordinate speed range: 1-200mm/s

#### `get_max_acc(mode)`

- **Function:** Get the maximum acceleration
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular acceleration
    - `1`: Coordinate acceleration
  - **Return value**: Angular acceleration range 1 to 150°/s, coordinate acceleration range 1 to 400 mm/s

#### `set_max_acc(mode, max_acc)`

- **Function:** Set maximum motion acceleration
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular acceleration
    - `1`: Coordinate acceleration
    - `max_acc`: Angular acceleration range 1 to 150°/s, coordinate acceleration range 1 to 400 mm/s

### 7. Joint software limit operation

#### `get_joint_min_angle(joint_id)`

- **function:** Read the minimum joint angle
- **Parameters:**
  - `joint_id` : Enter joint ID (range 1-6)
- **Return value**：`float` Angle value

#### `get_joint_max_angle(joint_id)`

- **function:** Read the maximum joint angle
- **Parameters:**
  - `joint_id` : Enter joint ID (range 1-6)
- **Return value:** `float` Angle value

#### `set_joint_min_angle(id, angle)`

- **function:** Set minimum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 1-6)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be less than the minimum value

#### `set_joint_max_angle(id, angle)`

- **function:** Set minimum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 1-6)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be greater than the maximum value

### 8. Joint motor auxiliary control

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

### 9. Drag Teach

#### `drag_teach_save()`

- **Function:** Start recording and dragging the teach point.
- Note: For optimal motion performance, the recording time should not exceed 90 seconds.

<!-- #### `drag_teach_pause()`

- **Function:** Pause sampling -->

#### `drag_teach_execute()`

- **Function:** Start dragging the teach point. Execute only once.

#### `drag_teach_clean()`

- **Function:** Clear the sampling point.

### 10. Dynamics

#### `get_collision_mode()`

- **Function**: Query the collision detection mode
- **Return value**:
  - `0`: Off
  - `1`: On

#### `set_collision_mode(mode)`

- **Function**: Set the joint collision threshold
- **Parameter**: `int`
  - `mode`:
    - `0`: Off
    - `1`: On

#### `get_collision_threshold()`

- **Function**: Get the joint collision threshold
- **Return value**: A list of all joint collision thresholds

#### `set_torque_comp(joint_id, comp_value=100)`

- **Function**: Set the torque compensation coefficient
- **Parameter**:
  - `joint_id` `int`: Joint ID, range 1 to 6
  - `comp_value`: Compensation value, range 0-250, default 100. Smaller values ​​result in more difficult joint dragging.

#### `get_torque_comp()`

- **Function**: Get torque compensation coefficients
- **Return value**: A list of torque compensation coefficients for all joints

#### `set_identify_mode(mode)`

- **Function**: Set dynamic parameter identification mode
- **Parameter**: `int`
  - `mode`:
    - `0`: Off
    - `1`: On

#### `get_identify_mode()`

- **Function**: Get dynamic parameter identification mode
- **Return value**:
  - `0`: Off
  - `1`: On

#### `fourier_trajectories(trajectory)`

- **Function**: Execute dynamic identification trajectory
- **Parameter**:
  - `trajectory`: `int`, range 0-4

#### `set_dynamic_parameters(add, data)`

- **Function**: Set dynamic parameters
- **Parameter**:
  - `add`: `int`, range 0 to 62
  - `data`: Parameter value

#### `get_dynamic_parameters(add)`

- **Function**: Read dynamic parameters
- **Parameter**:
  - `add`: (int), range 0 to 62
- **Return value**: data * 0.001

### 11. Circular Motion

#### `write_move_c(transpoint, endpoint, speed)`

- **Function**: Circular arc motion (specify transit points)
- **Parameter**:
- `transpoint(list)`: Arc transit points
- `endpoint(list)`: Arc endpoint
- `speed(int)`: 1 to 100

### 12. Run auxiliary information

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

### 13. Robotic arm end IO control

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

### 14. Bottom IO control

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

#### `set_base_external_config(communicate_mode, baud_rate, timeout)`

- **Function**: Set the bottom external device configuration
- **Parameter**:
  - `communicate_mode` (`int`) Range: 1 to 2
    - `1`: 485
    - `2`: can
  - `baud_rate` (`int`): Baud rate
  - `timeout`: Timeout

#### `get_base_external_config(communicate_mode, baud_rate, timeout)`

- **Function**: Read the bottom external device configuration
- **Return value**:
  - `communicate_mode` (`int`)
    - `1`: 485
    - `2`: can
  - `baud_rate` (`int`): Baud rate
  - `timeout`: Timeout

### 15. Set up 485 communication at the end of the robotic arm

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

### 16. Tool Coordinate System Operations

#### `set_tool_reference(coords)`

- **Function:** Set the tool coordinate system
- **Parameters**:
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

#### `get_tool_reference(coords)`

- **Function:** Get the tool coordinate system
- **Return value:** (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **Function:** Set the world coordinate system
- **Parameters**:
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

#### `get_world_reference()`

- **Function:** Get the world coordinate system.
- **Return value:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **Function:** Set the base coordinate system
- **Parameters:**
  - `rftype`: 0 - Base coordinates (default), 1 - World coordinates.

#### `get_reference_frame()`

- **Function:** Get the base coordinate system
- **Return value:** (list`) [x, y, z, rx, ry, rz].

#### `set_movement_type(move_type)`

- **Function:** Set the movement type
- **Parameters:**
  - `move_type`: 1 - moveL, 0 - moveJ.

#### `get_movement_type()`

- **Function:** Get the movement type
- **Return value:**
  - `1` - moveL
  - `0` - moveJ

#### `set_end_type(end)`

- **Function:** Set the end coordinate system
- **Parameters:**
  - `end (int)`: `0` - Flange (default), `1` - Tool

#### `get_end_type()`

- **Function:** Get the end coordinate system
- **Return value:**
  - `0` - Flange (default)
  - `1` - Tool

### 17. Algorithm Parameters

#### `get_vr_mode()`

- **Function:** Get the VR mode
- **Return value:**
  - `0`: Off
  - `1`: On

#### `set_vr_mode(move)`

- **Function:** Set the VR mode
- **Parameters:**
  - `move`: 1 - On, 0 - Off.

#### `get_model_direction()`

- **Function:** Get the joint model direction
- **Return value:** Model direction of joints 1-6
  - `1` - Same direction as the motor
  - `0` - Opposite direction from the motor

#### `set_model_direction(joint_id, direction)`

- **Function:** Set the joint model direction
- **Parameters:**
  - `joint_id (int)`: 1 to 6
  - `direction (int)`: `1` - Same direction as the motor. `0` - Opposite direction from the motor

#### `get_filter_len(rank)`

- **Function:** Get filter parameters
- **Parameters:**
  - `rank`: `int`
    - `1`: Drag teach sampling filter
    - `2`: Drag teach execution filter
    - `3`: Joint velocity fusion filter
    - `4`: Coordinate velocity fusion filter
    - `5`: Drag teach sampling period
- **Return value:** `int` 1 to 100

#### `set_filter_len(rank, value)`

- **Function:** Set filter parameters
- **Parameters:**
  - `rank (int)`: 1 to 5
  - `value (int)`: 1 to 100

#### `get_fusion_parameters(rank_mode)`

- **Function:** Get velocity fusion planning parameters
- **Parameters:**
  - `rank_mode`: 1 to 4
    - `1`: Fusion joint velocity
    - `2`: Fusion joint acceleration
    - `3`: Fusion coordinate velocity
    - `4`: Fusion coordinate acceleration
- **Return value:**
  - `1`: Fusion joint velocity
  - `2`: Fusion joint acceleration
  - `3`: Fusion coordinate velocity
  - `4`: Fusion coordinate acceleration

#### `set_fusion_parameters(rank_mode, value)`

- **Function:** Set velocity fusion planning parameters
- **Parameters:**
  - `rank_mode (int)`: 1 to 4
  - `value (int)`: 0 to 1000

### 18. Pro force-controlled gripper

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