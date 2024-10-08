# Pro 630 User Manual

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [Pro 630 User Manual](#pro-630-user-manual)
- [1. Python API](#1-python-api)
  - [API usage instructions](#api-usage-instructions)
    - [1. System Status](#1-system-status)
      - [`get_system_version()`](#get_system_version)
      - [`get_robot_type()`](#get_robot_type)
      - [`get_atom_version()`](#get_atom_version)
    - [2. Overall Status](#2-overall-status)
      - [`power_on()`](#power_on)
      - [`power_off()`](#power_off)
      - [`is_power_on()`](#is_power_on)
      - [`is_init_calibration()`](#is_init_calibration)
      - [`power_on_only()`](#power_on_only)
    - [3.Robot abnormal control](#3robot-abnormal-control)
      - [`get_error_information()`](#get_error_information)
      - [`clear_error_information()`](#clear_error_information)
      - [`get_robot_status()`](#get_robot_status)
      - [`servo_restore(joint_id)`](#servo_restorejoint_id)
      - [`get_comm_error_counts(joint_id, type)`](#get_comm_error_countsjoint_id-type)
      - [`over_limit_return_zero()`](#over_limit_return_zero)
    - [4. MDI Mode and Robot Control](#4-mdi-mode-and-robot-control)
      - [`set_control_mode(mode)`](#set_control_modemode)
      - [`get_control_mode()`](#get_control_mode)
      - [`get_angles()`](#get_angles)
      - [`send_angles(angles, speed)`](#send_anglesangles-speed)
      - [`get_angle()`](#get_angle)
      - [`send_angle(id, degree, speed)`](#send_angleid-degree-speed)
      - [`get_coords()`](#get_coords)
      - [`send_coord(id, coord, speed)`](#send_coordid-coord-speed)
      - [`send_coords(coords, speed, mode)`](#send_coordscoords-speed-mode)
      - [`pause()`](#pause)
      - [`is_paused()`](#is_paused)
      - [`resume()`](#resume)
      - [`stop()`](#stop)
      - [`is_in_position(data, flag)`](#is_in_positiondata-flag)
      - [`is_moving()`](#is_moving)
    - [5. JOG mode and operation](#5-jog-mode-and-operation)
      - [`jog_angle(joint_id, direction, speed)`](#jog_anglejoint_id-direction-speed)
      - [`jog_coord(coord_id, direction, speed)`](#jog_coordcoord_id-direction-speed)
      - [`jog_increment_angle(joint_id, increment, speed)`](#jog_increment_anglejoint_id-increment-speed)
      - [`jog_increment_coord(coord_id, increment, speed)`](#jog_increment_coordcoord_id-increment-speed)
    - [6. Software joint limit](#6-software-joint-limit)
      - [`get_joint_min_angle(joint_id)`](#get_joint_min_anglejoint_id)
      - [`get_joint_max_angle(joint_id)`](#get_joint_max_anglejoint_id)
      - [`set_joint_min(id, angle)`](#set_joint_minid-angle)
      - [`set_joint_max(id, angle)`](#set_joint_maxid-angle)
    - [7. Joint motor auxiliary control](#7-joint-motor-auxiliary-control)
      - [`is_servo_enable(servo_id)`](#is_servo_enableservo_id)
      - [`is_all_servo_enable()`](#is_all_servo_enable)
      - [`set_servo_calibration(servo_id)`](#set_servo_calibrationservo_id)
      - [`release_servo(servo_id)`](#release_servoservo_id)
      - [`focus_servo(servo_id)`](#focus_servoservo_id)
      - [`release_all_servos()`](#release_all_servos)
      - [`focus_all_servos()`](#focus_all_servos)
      - [`set_break（joint_id, value）`](#set_breakjoint_id-value)
      - [`get_servo_speeds()`](#get_servo_speeds)
      - [`get_servo_currents()`](#get_servo_currents)
      - [`get_servo_status()`](#get_servo_status)
    - [8. Drag to teach](#8-drag-to-teach)
      - [`drag_teach_save()`](#drag_teach_save)
      - [`drag_teach_pause()`](#drag_teach_pause)
      - [`drag_teach_execute()`](#drag_teach_execute)
      - [`drag_teach_clean()`](#drag_teach_clean)
    - [9. Dynamics](#9-dynamics)
      - [`set_collision_mode(mode)`](#set_collision_modemode)
      - [`set_collision_threshold(joint_id, value = 100)`](#set_collision_thresholdjoint_id-value--100)
      - [`get_collision_threshold()`](#get_collision_threshold)
      - [`set_torque_comp(joint_id, value=100)`](#set_torque_compjoint_id-value100)
      - [`get_torque_comp()`](#get_torque_comp)
    - [10. Algorithm Parameter](#10-algorithm-parameter)
    - [11. Set bottom IO input/output status](#11-set-bottom-io-inputoutput-status)
      - [`set_basic_output(pin_no, pin_signal)`](#set_basic_outputpin_no-pin_signal)
      - [`get_basic_input(pin_no)`](#get_basic_inputpin_no)
    - [12. Tool Gripper Control](#12-tool-gripper-control)
      - [`set_gripper_state(flag, speed, _type_1=None)`](#set_gripper_stateflag-speed-_type_1none)
      - [`set_gripper_value(gripper_value, speed, gripper_type=None)`](#set_gripper_valuegripper_value-speed-gripper_typenone)
      - [`set_gripper_calibration()`](#set_gripper_calibration)
      - [`set_gripper_enabled(value)`](#set_gripper_enabledvalue)
      - [`set_gripper_mode(mode)`](#set_gripper_modemode)
      - [`get_gripper_mode()`](#get_gripper_mode)
    - [13. Toll RGB](#13-toll-rgb)
      - [`set_color(r, g, b)`](#set_colorr-g-b)
    - [14. Tool io control](#14-tool-io-control)
      - [`set_digital_output(pin_no, pin_signal)`](#set_digital_outputpin_no-pin_signal)
      - [`get_digital_input(pin_no)`](#get_digital_inputpin_no)
      - [`is_btn_clicked()`](#is_btn_clicked)
    - [15. Coordinate system settings](#15-coordinate-system-settings)
      - [`get_world_reference()`](#get_world_reference)
      - [`set_reference_frame(rftype)`](#set_reference_framerftype)
      - [`get_reference_frame()`](#get_reference_frame)
      - [`set_movement_type(move_type)`](#set_movement_typemove_type)
      - [`get_movement_type()`](#get_movement_type)
      - [`set_end_type(end)`](#set_end_typeend)
      - [`get_end_type()`](#get_end_type)
    - [16. synchronous/asynchronous control mode](#16-synchronousasynchronous-control-mode)
      - [`set_pos_switch(mode)`](#set_pos_switchmode)
      - [`get_pos_switch()`](#get_pos_switch)
- [2. Instructions](#2-instructions)
  - [Serial communication](#serial-communication)
  - [TCP/IP Communication](#tcpip-communication)
    - [Server](#server)
    - [Client](#client)

<!-- /code_chunk_output -->

# 1. Python API

## API usage instructions

API (Application Programming Interface), also known as Application Programming Interface functions, are predefined functions. When using the following function interfaces, please import our API library at the beginning by entering the following code, otherwise it will not run successfully:

### 1. System Status

#### `get_system_version()`

- **function：** get system version
- **Return value：** system version

#### `get_robot_type()`

- **function：** get robot id

- **Return value：** Definition Rule: Actual machine model. For example, the Mercury A1 model is 4500

#### `get_atom_version()`

- **function：** Get the end version number
- **Return value：** End parameters(`float`)

### 2. Overall Status

#### `power_on()`

- **function:** atom open communication (default open)

  - Attentions： After executing poweroff or pressing emergency stop, it takes 8 seconds to power on and restore power

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
  - `-1`: error

#### `is_init_calibration()`

- **function** Check if the robot is initialized for calibration
- **Return value:**
  - `True`: the robot is initialized
  - `False`: otherwise

#### `power_on_only()`

- **function:** Only turn on the power

### 3.Robot abnormal control

#### `get_error_information()`

- **function:** Obtaining robot error information

  - **Return value:**
    - `0`: No error message
    - `1~6`: The corresponding joint exceeds the limit position
    - `16~19`: Collision protection
    - `32`: Kenematics inverse solution has no solution
    - `33~34`: Linear motion has no adjacent solution

#### `clear_error_information()`

- **function:** Clear robot error message

#### `get_robot_status()`

- **function:** Upper computer error security status
- **Return value:** `0` Normal; `other` Robot triggered collision detection

#### `servo_restore(joint_id)`

- **function:** Clear joint abnormalities
- **Parameters:**
  - `joint_id`: `int` joint id 1 - 6

#### `get_comm_error_counts(joint_id, type)`

- **function:** Read the number of communication exceptions
- **Parameters:**
  - `joint_id`: `int` joint id 1 - 6
  - `type`: int. Error type 1 - 4
    - `1` The number of exceptions sent by the joint
    - `2` The number of exceptions obtained by the joint
    - `3` The number of exceptions sent by the end
    - `4` The number of exceptions read by the end

#### `over_limit_return_zero()`

- **function:** Return to zero when the joint is over the limit

### 4. MDI Mode and Robot Control

#### `set_control_mode(mode)`

- **function:** Set robot motion mode
- **Parameters:**
  - **mode**
    - `0`: location mode
    - `1`: torque mode

#### `get_control_mode()`

- **function:** Get robot motion mode
- **Return value:**
  - `0`: location mode
  - `1`: torque mode

#### `get_angles()`

- **function:** get the degree of all joints
- **Return value**: `list` a float list of all degree

#### `send_angles(angles, speed)`

- **function：** Send all angles to all joints of the robotic arm
- **Parameters:**
  - `angles`: a list of degree value(`List[float]`), length 6
  - `speed`: (`int`) 1 ~ 100
- **Return value:** (`int`) 1

#### `get_angle()`

- **function:** Get single joint angle
- **Parameters**： joint_id (int): 1 ~ 6
- **Return value:** Array of angles corresponding to joints

#### `send_angle(id, degree, speed)`

- **function:** send one degree of joint to robot arm

- **Parameters:**

  - `id`: Joint id(`genre.Angle`), range int 1-6

  - `degree`: degree value(`float`)

    | Joint Id | range      |
    | -------- | ---------- |
    | 1        | -180 ~ 180 |
    | 2        | -45 ~ 225  |
    | 3        | -165 ~ 165 |
    | 4        | -90 ~ 270  |
    | 5        | -180 ~ 180 |
    | 6        | -180 ~ 180 |

  - `speed`：the speed and range of the robotic arm's movement 1~150

- **Return value:** (`int`) 1

#### `get_coords()`

- **function:** Obtain robot arm coordinates from a base based coordinate system
- **Return value:** a float list of coord:[x, y, z, rx, ry, rz]

#### `send_coord(id, coord, speed)`

- **function:** send one coord to robot arm

- **Parameters:**

  - `id`:send one coord to robot arm, 1-6 corresponds to [x, y, z, rx, ry, rz]

  - `coord`: coord value(`float`)

    | Coord Id | range      |
    | -------- | ---------- |
    | 1        | -630 ~ 630 |
    | 2        | -630 ~ 630 |
    | 3        | -425 ~ 835 |
    | 4        | -180 ~ 180 |
    | 5        | -180 ~ 180 |
    | 6        | -180 ~ 180 |

  - `speed`: (`int`) 1-200

- **Return value:** (`int`) 1

#### `send_coords(coords, speed, mode)`

- **function:**: Send overall coordinates and posture to move the head of the robotic arm from its original point to your specified point
- **Parameters:**
  - coords: ： a list of coords value `[x,y,z,rx,ry,rz]`,length6
  - speed`(int)`: 1 ~ 100
- **Return value:** (`int`) 1

#### `pause()`

- **function:** Control the instruction to pause the core and stop all movement instructions

#### `is_paused()`

- **function:** Check if the program has paused the move command
- **Return value:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **function:** resume the robot movement and complete the previous command

#### `stop()`

- **function:** stop all movements of robot
- **Return value**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `is_in_position(data, flag)`

- **function** : judge whether in the position.
- **Parameters:**
  - data: Provide a set of data that can be angles or coordinate values. If the input angle length range is 7, and if the input coordinate value length range is 6
  - flag data type (value range 0 or 1)
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

### 5. JOG mode and operation

#### `jog_angle(joint_id, direction, speed)`

- **function:** jog control angle
- **Parameters**:
  - `joint_id`: Represents the joints of the robotic arm, represented by joint IDs ranging from 1 to 6
  - `direction(int)`: To control the direction of movement of the robotic arm, input `0` as negative value movement and input `1` as positive value movement
  - `speed`: 1 ~ 100
- **Return value:** `int` 1

#### `jog_coord(coord_id, direction, speed)`

- **function:** jog control coord.
- **Parameters:**
  - `coord_id`: (`int`) Coordinate range of the robotic arm: 1~6
  - `direction`: (`int`) To control the direction of machine arm movement, `0` - negative value movement, `1` - positive value movement
  - `speed`: 1 ~ 100
- **Return value:** `int` 1

#### `jog_increment_angle(joint_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: 1-6
  - `increment`: Incremental movement based on the current position angle
  - `speed`: 1 ~ 100
- **Return value:** `int` 1

#### `jog_increment_coord(coord_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: axis id 1 - 6.
  - `increment`: Incremental movement based on the current position coord
  - `speed`: 1 ~ 100
- **Return value:** `int` 1

### 6. Software joint limit

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

### 7. Joint motor auxiliary control

#### `is_servo_enable(servo_id)`

- **function:** Detecting joint connection status
- **Parameters:** `servo id` 1-6
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `is_all_servo_enable()`

- **function:** Detect the status of all joint connections
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `set_servo_calibration(servo_id)`

- **function:** The current position of the calibration joint actuator is the angle zero point
- **Parameters**:
  - `servo_id`: 1 - 6

#### `release_servo(servo_id)`

- **function:** Set the specified joint torque output to turn off
- **Parameters**:
  - `servo_id`: 1 ~ 6
- **Return value:**
  - `1`: release successful
  - `0`: release failed
  - `-1`: error

#### `focus_servo(servo_id)`

- **function**: Set the specified joint torque output to turn on
- **Parameters**: `servo_id`: 1 ~ 6
- **Return value:**
  - `1`: focus successful
  - `0`: focus failed
  - `-1`: error

#### `release_all_servos()`

- **function:** release all robot arms

  - Attentions：After the joint is disabled, it needs to be enabled to control within 1 second

- **Return value:**

  - `1` - release completed.
  - `0` - release failed

#### `focus_all_servos()`

- **function:** Turn on robot torque output
- **Return value:**
  - `1`: complete
  - `0`: failed
  - `-1`: error

#### `set_break（joint_id, value）`

- **function:** Set break point
- **Parameters**：
  - `joint_id`: int. joint id 1 - 6
  - `value`: int. 0 - disable, 1 - enable
- **Return value:** `0` : faile; `1` : success

#### `get_servo_speeds()`

- **function**：Get the movement speed of all joints
- **Return value**： unit step/s

#### `get_servo_currents()`

- **function**：Get the movement current of all joints
- **Return value**： 0 ~ 5000 mA

#### `get_servo_status()`

- **function**：Get the movement status of all joints
- **Return value**： a value of `0` means no error

### 8. Drag to teach

#### `drag_teach_save()`

- **function:** Start recording and dragging teaching points.
  - Note: In order to display the best sports effect, the recording time should not exceed 90 seconds

#### `drag_teach_pause()`

- **function:** Pause sampling

#### `drag_teach_execute()`

- **function:** Start dragging the teach-in point, executing it only once.

#### `drag_teach_clean()`

- **function:** Clear saved recording information

### 9. Dynamics

#### `set_collision_mode(mode)`

- **function:** Set collision detection mode
- **Parameters:**
  - `mode` (int): 0 / 1

#### `set_collision_threshold(joint_id, value = 100)`

- **function:** Set joint collision threshold
- **Parameters:**
  - `joint_id` (int) 1 - 6
  - `value` (int) 50 ~ 250

#### `get_collision_threshold()`

- **function:** Get joint collision threshold
- **Return value:** Range: 50 ~ 250

#### `set_torque_comp(joint_id, value=100)`

- **function:** Set joint torque compensation
- **Parameters:**
  - `joint_id` (int) 1 ~ 6
  - `value` (int) 0 ~ 250

#### `get_torque_comp()`

- **function:** Get joint torque compensation
- **Return value:** Range: 0 ~ 250

### 10. Algorithm Parameter

`get_vr_mode()`

- **function**: Check if the robot is in VR mode

- **Return value:** `0` Open; `1` Close

`set_vr_mode(mode)`

- **function:** Set VR mode

- **Parameters:**

  - `mode` (int) 0 / 1

`get_model_direction()`

- **function:** Get the direction of the robot model

- **Return value:** `tuple[bytearray, int]`

`set_model_direction(id)`

- **function:** Set the direction of the robot model

- **Parameters:**

  - `id` (int) 1 ~ 6

  - `direction` (int) 0/1

`get_filter_len(rank)`

- **function:** Get the filter length

- **Parameters:** rank(int)

  - `1` Drag teaching sampling filter

  - `2` Drag teaching execution filter

  - `3` Joint velocity fusion filter

  - `4` Coordinate velocity fusion filter

  - `5` Drag teaching sampling period

`set_filter_len(rank, value)`

- **function:** Set the filter length

- **Parameter:**

  - rank(int)

    - `1` Drag teaching sampling filter

    - `2` Drag teaching execution filter

    - `3` Joint velocity fusion filter

    - `4` Coordinate velocity fusion filter

    - `5` Drag teaching sampling period

  - value(`int`) 1 ~ 100

### 11. Set bottom IO input/output status

#### `set_basic_output(pin_no, pin_signal)`

- **function**：Set Base IO Output
- **Parameters**：
  - `pin_no` (`int`) Pin port number, range 1 ~ 6
  - `pin_signal` (`int`): `0` low. `1` high

#### `get_basic_input(pin_no)`

- **function:** Read base IO input, range 1 ~ 6
- **Parameters:**
  - `pin_no` (`int`) pin number
- **Return value:** `0` low; `1` high

### 12. Tool Gripper Control

#### `set_gripper_state(flag, speed, _type_1=None)`

- **function**: Adaptive gripper enable

- **Parameters**:

  - `flag (int)` : 0 - open 1 - close, 254 - release

  - `speed (int)`: 1 ~ 100

  - `_type_1 (int)`:

    - `1` : Adaptive gripper (default state is 1)

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **function**: Set the gripper value

- **Parameters**:

  - `gripper_value (int)` : 0 ~ 100
  - `speed (int)`: 1 ~ 100
  - `gripper_type (int)`:
    - `1` : Adaptive gripper (default state is 1)
    - `2` : 5 finger dexterous hand
    - `3` : Parallel gripper (this parameter can be omitted)
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
  - 1: normal mode

### 13. Toll RGB

#### `set_color(r, g, b)`

- **function**: Set the color of the end light of the robotic arm
- **Parameters**:
  - `r (int)`: 0 ~ 255
  - `g (int)`: 0 ~ 255
  - `b (int)`: 0 ~ 255

### 14. Tool io control

#### `set_digital_output(pin_no, pin_signal)`

- **function:** set IO statue
- **Parameters**
  - `pin_no` (int): Pin number
  - `pin_signal` (int): 0 / 1

#### `get_digital_input(pin_no)`

- **function:** read IO statue
- **Parameters**: `pin_no` (int)
- **Return value**: signal

#### `is_btn_clicked()`

- **function**: Get the status of the button at the end of the robot arm
- **Return value**:
  - 0: no clicked
  - 1: clicked

### 15. Coordinate system settings

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

### 16. synchronous/asynchronous control mode

#### `set_pos_switch(mode)`

- **function:** Set synchronous/asynchronous control mode
- **Parameters:**
  - `mode (int)`: `0` - asynchronous, `1` - synchronous

#### `get_pos_switch()`

- **function:** Get synchronous/asynchronous control mode
- **Return value:**
  - `0` - asynchronous
  - `1` - synchronous

# 2. Instructions

## Serial communication

1. When the robot is powered on for the first time, it needs to be powered on before it can be controlled. Use the `power_on` command to control the robot to power on.

```python

from pymycobot import Pro630

p = Pro630("/dev/ttyAMA0")
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

2. If the emergency stop button is pressed and released during use, you need to power off and then power on.

```python

from pymycobot import Pro630
import time
p = Pro630("/dev/ttyAMA0")
p.power_off()
time.sleep(1)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

3. Control mode switching: The robot is in synchronous control mode by default. After sending the angle or coordinates, the control function will end after the robot moves. You can use the `set_pos_switch` interface to set it to asynchronous mode. In asynchronous mode, the function will end immediately after sending the angle or coordinates, and will not wait for the robot to move to the target position.

> Note: Please call `set_pos_switch` function to set the mode before controlling the motion

```python

from pymycobot import Pro630

p = Pro630("/dev/ttyAMA0")
# asynchronous mode
p.set_pos_switch(0)
# synchronous mode
# p.set_pos_switch(1)
p.send_angle(5, 100, 20)

```

## TCP/IP Communication

### Server

Download the [server](https://github.com/elephantrobotics/pymycobot/blob/pro630_esp32/demo/Server_Pro630.py) to the robot's system and run it.

```shell
python3 Server_Pro630.py
```

After running, the server's IP address and bound port number will be displayed:

```shell
pi@raspberrypi:~/pymycobot/demo $ python3 Server_Pro630.py 
ip: 192.168.1.169 port: 9000
Binding succeeded!
This is asynchronous mode
waiting connect!------------------

```

### Client

Update `pymycobot` to v3.5.0a2 or later:

```shell
pip uninstall pymycobot
pip install pymycobot==3.5.0a2
```

1. When the robot is powered on for the first time, it needs to be powered on before it can be controlled. Use the `power_on` command to control the robot to power on.

```python

from pymycobot import Pro630Client

p = Pro630Client("192.168.1.169", 9000)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

2. If the emergency stop button is pressed and released during use, you need to power off and then power on.

```python

from pymycobot import Pro630Client

import time
p = Pro630Client("192.168.1.169", 9000)

p.power_off()
time.sleep(1)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

3. Control mode switching: The robot is in synchronous control mode by default. After sending the angle or coordinates, the control function will end after the robot moves. You can use the `set_pos_switch` interface to set it to asynchronous mode. In asynchronous mode, the function will end immediately after sending the angle or coordinates, and will not wait for the robot to move to the target position.

> Note: Please call `set_pos_switch` function to set the mode before controlling the motion

```python

from pymycobot import Pro630Client

p = Pro630Client("192.168.1.169", 9000)

# asynchronous mode
p.set_pos_switch(0)
# synchronous mode
# p.set_pos_switch(1)
p.send_angle(5, 100, 20)

```