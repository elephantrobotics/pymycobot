# pymycobot

**This is python API for mycobot.**

We support Python2, Python3.5 or later.

<details>
<summary>Catalogue:</summary>

<!-- vim-markdown-toc GFM -->

- [pymycobot](#pymycobot)
- [MyCobot](#mycobot)
  - [Overall status](#overall-status)
    - [power_on](#power_on)
    - [power_off](#power_off)
    - [is_power_on](#is_power_on)
    - [release_all_servos](#release_all_servos)
    - [is_controller_connected](#is_controller_connected)
  - [MDI mode and operation](#mdi-mode-and-operation)
    - [get_angles](#get_angles)
    - [send_angle](#send_angle)
    - [send_angles()](#send_angles)
    - [get_radians](#get_radians)
    - [send_radians](#send_radians)
    - [get_coords](#get_coords)
    - [send_coord](#send_coord)
    - [send_coords](#send_coords)
    - [sync_send_angles](#sync_send_angles)
    - [sync_send_coords](#sync_send_coords)
    - [is_in_position](#is_in_position)
  - [JOG mode and operation](#jog-mode-and-operation)
    - [jog_angle](#jog_angle)
    - [jog_coord](#jog_coord)
    - [jog_stop](#jog_stop)
    - [pause](#pause)
    - [resume](#resume)
    - [stop](#stop)
    - [is_paused](#is_paused)
    - [set_encoder](#set_encoder)
    - [get_encoder](#get_encoder)
    - [set_encoders](#set_encoders)
    - [get_encoders](#get_encoders)
  - [Running status and Settings](#running-status-and-settings)
    - [get_speed](#get_speed)
    - [set_speed](#set_speed)
    - [set_joint_min](#set_joint_min)
    - [set_joint_max](#set_joint_max)
    - [get_joint_min_angle](#get_joint_min_angle)
    - [get_joint_max_angle](#get_joint_max_angle)
  - [Servo control](#servo-control)
    - [is_servo_enable](#is_servo_enable)
    - [is_all_servo_enable](#is_all_servo_enable)
    - [set_servo_data](#set_servo_data)
    - [get_servo_data](#get_servo_data)
    - [set_servo_calibration](#set_servo_calibration)
    - [release_servo](#release_servo)
    - [focus_servo](#focus_servo)
  - [Atom IO](#atom-io)
    - [set_color](#set_color)
    - [set_pin_mode](#set_pin_mode)
    - [set_digital_output()](#set_digital_output)
    - [get_digital_input()](#get_digital_input)
    - [set_pwm_output()](#set_pwm_output)
    - [get_gripper_value](#get_gripper_value)
    - [set_gripper_state](#set_gripper_state)
    - [set_gripper_value](#set_gripper_value)
    - [set_gripper_ini](#set_gripper_ini)
    - [is_gripper_moving](#is_gripper_moving)
    - [get_basic_input](#get_basic_input)
    - [set_basic_output](#set_basic_output)
    - [set_ssid_pwd](#set_ssid_pwd)
    - [get_ssid_pwd](#get_ssid_pwd)
    - [set_server_port](#set_server_port)
    - [get_tof_distance](#get_tof_distance)
    - [get_tool_reference](#get_tool_reference)
    - [set_tool_reference](#set_tool_reference)
    - [set_world_reference](#set_world_reference)
    - [get_world_reference](#get_world_reference)
    - [set_reference_frame](#set_reference_frame)
    - [get_reference_frame](#get_reference_frame)
    - [set_movement_type](#set_movement_type)
    - [get_movement_type](#get_movement_type)
    - [set_end_type](#set_end_type)
    - [get_end_type](#get_end_type)
    - [get_plan_speed](#get_plan_speed)
    - [get_plan_acceleration](#get_plan_acceleration)
    - [set_plan_speed](#set_plan_speed)
    - [set_plan_acceleration](#set_plan_acceleration)
    - [get_servo_speeds](#get_servo_speeds)
    - [get_servo_currents](#get_servo_currents)
    - [get_servo_voltages](#get_servo_voltages)
    - [get_servo_status](#get_servo_status)
    - [get_servo_temps](#get_servo_temps)
  - [Raspberry pi -- GPIO](#raspberry-pi----gpio)
    - [gpio_init](#gpio_init)
    - [gpio_output](#gpio_output)
- [Angle](#angle)
- [Coord](#coord)
- [utils (Module)](#utils-module)
  - [get_port_list](#get_port_list)
  - [detect_port_of_basic](#detect_port_of_basic)
- [MyCobotSocket](#mycobotsocket)
    - [Client](#client)
    - [Server](#server)
  - [socket control](#socket-control)
    - [connect](#connect)
    - [set_gpio_mode](#set_gpio_mode)
    - [set_gpio_out](#set_gpio_out)
    - [set_gpio_output](#set_gpio_output)
    - [get_gpio_in](#get_gpio_in)
- [MyBuddy](#mybuddy)
    - [focus_servo(id, servo_id)](#focus_servoid-servo_id)
    - [get_acceleration(id)](#get_accelerationid)
    - [get_angles(id)](#get_anglesid)
    - [get_basic_input(pin_no)](#get_basic_inputpin_no)
    - [get_coords(id)](#get_coordsid)
    - [get_digital_input(id, pin_no)](#get_digital_inputid-pin_no)
    - [get_encoder(id, joint_id)](#get_encoderid-joint_id)
    - [get_encoders(id)](#get_encodersid)
    - [get_end_type(id)](#get_end_typeid)
    - [get_gripper_value(id)](#get_gripper_valueid)
    - [get_joint_current(id, joint_id)](#get_joint_currentid-joint_id)
    - [get_joint_max_angle(id, joint_id)](#get_joint_max_angleid-joint_id)
    - [get_joint_min_angle(id, joint_id)](#get_joint_min_angleid-joint_id)
    - [get_movement_type(id)](#get_movement_typeid)
    - [get_plan_acceleration()](#get_plan_acceleration-1)
    - [get_plan_speed()](#get_plan_speed-1)
    - [get_reference_frame(id)](#get_reference_frameid)
    - [get_robot_id()](#get_robot_id)
    - [get_robot_version()](#get_robot_version)
    - [get_servo_currents(id)](#get_servo_currentsid)
    - [get_servo_data(id, servo_no, data_id)](#get_servo_dataid-servo_no-data_id)
    - [get_servo_status(id)](#get_servo_statusid)
    - [get_servo_temps(id)](#get_servo_tempsid)
    - [get_servo_voltages(id)](#get_servo_voltagesid)
    - [get_speed(id)](#get_speedid)
    - [get_system_version()](#get_system_version)
    - [get_tool_reference(id)](#get_tool_referenceid)
    - [get_world_reference(id)](#get_world_referenceid)
    - [is_all_servo_enable(id)](#is_all_servo_enableid)
    - [is_controller_connected()](#is_controller_connected-1)
    - [is_free_mode(id)](#is_free_modeid)
    - [is_gripper_moving(id)](#is_gripper_movingid)
    - [is_in_position(id, mode, data)](#is_in_positionid-mode-data)
    - [is_moving(id)](#is_movingid)
    - [is_paused(id)](#is_pausedid)
    - [is_power_on()](#is_power_on-1)
    - [is_servo_enable(id, servo_id)](#is_servo_enableid-servo_id)
    - [jog_absolute(id, joint_id, angle, speed)](#jog_absoluteid-joint_id-angle-speed)
    - [jog_angle(id, joint_id, direction, speed)](#jog_angleid-joint_id-direction-speed)
    - [jog_coord(id, coord_id, direction, speed)](#jog_coordid-coord_id-direction-speed)
    - [jog_increment(id, joint_id, increment, speed)](#jog_incrementid-joint_id-increment-speed)
    - [jog_stop(id)](#jog_stopid)
    - [joint_brake(id, joint_id)](#joint_brakeid-joint_id)
    - [pause(id)](#pauseid)
    - [power_off()](#power_off-1)
    - [power_on()](#power_on-1)
    - [release_all_servos()](#release_all_servos-1)
    - [release_servo(id, servo_id)](#release_servoid-servo_id)
    - [resume(id)](#resumeid)
    - [send_angle(id, joint, angle, speed)](#send_angleid-joint-angle-speed)
    - [send_angles(id, degrees, speed, mode)](#send_anglesid-degrees-speed-mode)
    - [send_coord(id, coord, data, speed)](#send_coordid-coord-data-speed)
    - [send_coords(id, coords, speed, mode)](#send_coordsid-coords-speed-mode)
    - [set_acceleration(id, acc)](#set_accelerationid-acc)
    - [set_basic_mode(pin_no, pin_mode)](#set_basic_modepin_no-pin_mode)
    - [set_basic_output(pin_no, pin_signal)](#set_basic_outputpin_no-pin_signal)
    - [set_color(r=0, g=0, b=0)](#set_colorr0-g0-b0)
    - [set_digital_output(id, pin_no, pin_signal)](#set_digital_outputid-pin_no-pin_signal)
    - [set_encoder(id, joint_id, encoder, speed)](#set_encoderid-joint_id-encoder-speed)
    - [set_encoders(id, encoders, speed, mode)](#set_encodersid-encoders-speed-mode)
    - [set_end_type(id, end)](#set_end_typeid-end)
    - [set_free_mode(id)](#set_free_modeid)
    - [set_gripper_calibration(id)](#set_gripper_calibrationid)
    - [set_gripper_state(id, flag)](#set_gripper_stateid-flag)
    - [set_gripper_value(id, value)](#set_gripper_valueid-value)
    - [set_joint_current(id, joint_id, current)](#set_joint_currentid-joint_id-current)
    - [set_joint_max(id, joint_id, angle)](#set_joint_maxid-joint_id-angle)
    - [set_joint_min(id, joint_id, angle)](#set_joint_minid-joint_id-angle)
    - [set_movement_type(id, move_type)](#set_movement_typeid-move_type)
    - [set_pin_mode(id, pin_no, pin_mode)](#set_pin_modeid-pin_no-pin_mode)
    - [set_plan_acceleration(acceleration, is_linear)](#set_plan_accelerationacceleration-is_linear)
    - [set_plan_speed(speed, is_linear)](#set_plan_speedspeed-is_linear)
    - [set_pwm_output(id, channel, frequency, pin_val)](#set_pwm_outputid-channel-frequency-pin_val)
    - [set_reference_frame(id, rftype)](#set_reference_frameid-rftype)
    - [set_robot_id(id)](#set_robot_idid)
    - [set_servo_calibration(id, servo_no)](#set_servo_calibrationid-servo_no)
    - [set_servo_data(id, servo_no, data_id, value)](#set_servo_dataid-servo_no-data_id-value)
    - [set_speed(id, speed)](#set_speedid-speed)
    - [set_tool_reference(id, coords)](#set_tool_referenceid-coords)
    - [set_world_reference(id, coords)](#set_world_referenceid-coords)
    - [stop(id)](#stopid)
  
<!-- vim-markdown-toc -->
</details>

# MyCobot

**Import to your project**:

```python
# for mycobot
from pymycobot import MyCobot

# for mypalletizer
# from pymycobot import MyPalletizer
```

> Note: If no parameter is given, there is no parameter; if no return value is given, there is no return value

## Overall status

### power_on

- **Prototype**: `power_on()`

- **Description**:Atom open communication (default open).

### power_off

- **Prototype**: `power_off()`

- **Description**: Atom turn off communication.

### is_power_on

- **Prototype**: `is_power_on()`

- **Description**: Adjust robot arm whether power on.

- **Returns**

  - `1`: power on
  - `0`: power off
  - `-1`: error

### release_all_servos

- **Prototype**: `release_all_servos()`

- **Description**: Set robot arm into free moving mode.

### is_controller_connected

- **Prototype**: `is_controller_connected()`

- **Description**: Check if connected with Atom.

- **Returns**

  - `1`: connected
  - `0`: not connected
  - `-1`: error

## MDI mode and operation

### get_angles

- **Prototype**: `get_angles()`

- **Description**: Get the degree of all joints.

- **Returns**: `list`: A float list of all degree.

### send_angle

- **Prototype**: `send_angles(id, degree, speed)`

- **Description**: Send one degree of joint to robot arm.

- **Parameters**

  - `id`: Joint id(`genre.Angle`) / int 1-6
  - `degree`: degree value(`float`) (about -170 ～ 170)
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angle(Angle.J2.value, 10, 50)
  ```

### send_angles()

- **Prototype**: `send_angles(degrees, speed)`

- **Description**: Send the degrees of all joints to robot arm.

- **Parameters**

  - `degrees`: a list of degree value(`List[float]`), length 6.

  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles([0,0,0,0,0,0], 80)
  ```

### get_radians

- **Prototype**: `get_radians()`

- **Description**: Get the radians of all joints.

- **Returns**: `list`: A float list of radian.

### send_radians

- **Prototype**: `send_radians(radians, speed)`

- **Description**: Send the radians of all joint to robot arm.

- **Parameters**:

  - `radians`: a list of radian value(`List[float]`), length 6.
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles_by_radian([1,1,1,1,1,1], 70)
  ```

### get_coords

- **Prototype**: `get_coords()`

- **Description**: Get the Coords from robot arm, coordinate system based on base.

- **Returns**: `list`: A float list of coord - `[x, y, z, rx, ry, rz]`

### send_coord

- **Prototype**: `send_coords(id, coord, speed)`

- **Description**: Send one coord to robot arm.

- **Parameters**

  - `id`: coord id(`genre.Coord`) / int 1-6
  - `coord`: coord value(`float`)
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coord(Coord.X.value, -40, 70)
  ```

### send_coords

- **Prototype**: `send_coords(coords, speed, mode)`

- **Description**: Send all coords to robot arm.

- **Parameters**

  - `coords`: a list of coords value(`List[float]`), length 6.
  - `speed`: (`int`) 0 ~ 100
  - `mode`: (`int`): `0` - angular, `1` - linear

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
  ```

### sync_send_angles

- **Prototype**: `sync_send_angles(degrees, speed, timeout=7)`

- **Description**: Send the angle in synchronous state and return when the target point is reached

- **Parameters**

  - `degrees`: a list of degree value(`List[float]`), length 6.
  - `speed`: (`int`) 0 ~ 100
  - `timeout`: default 7s.

### sync_send_coords

- **Prototype**: `sync_send_coords(coords, speed, mode, timeout=7)`

- **Description**: Send the coord in synchronous state and return when the target point is reached

- **Parameters**

  - `coords`: a list of coords value(`List[float]`)
  - `speed`: (`int`) 0 ~ 100
  - `mode`: (`int`): `0` - angular, `1` - linear
  - `timeout`: default 7s.

### is_in_position

- **Prototype**: `is_in_position(data, flag)`

- **Description**: Judge whether in the position.

- **Parameters**

  - `data`: A data list, angles or coords, length 6.
  - `flag`: Tag the data type, `0` - angles, `1` - coords.

- **Returns**

  - `1` - true
  - `0` - false
  - `-1` - error

## JOG mode and operation

### jog_angle

- **Prototype**: `jog_angle(joint_id, direction, speed)`

- **Description**: Jog control angle

- **Parameters**

  - `joint_id`: (`int`) 1 ~ 6
  - `direction`: `0` - decrease, `1` - increase
  - `speed`: 0 ~ 100

### jog_coord

- **Prototype**: `jog_coord(coord_id, direction, speed)`

- **Description**: Jog control coord.

- **Parameters**

  - `coord_id`: (`int`) 1 ~ 6
  - `direction`: `0` - decrease, `1` - increase
  - `speed`: 0 ~ 100

### jog_stop

- **Prototype**: `jog_stop()`

- **Description**: Stop jog moving.

### pause

- **Prototype**: `pause()`

- **Description**: Pause movement.

### resume

- **Prototype**: `resume()`

- **Description**: Recovery movement.

### stop

- **Prototype**: `stop()`

- **Description**: Stop moving.

### is_paused

- **Prototype**: `is_paused()`

- **Description**: Judge whether the manipulator pauses or not.

- **Returns** :

  - `1` - paused
  - `0` - not paused
  - `-1` - error

### set_encoder

- **Prototype**: `set_encoder(joint_id, encoder)`

- **Description**: Set a single joint rotation to the specified potential value.

- **Parameters**

  - `joint_id`: (`int`) 1 ~ 6
  - `encoder`: 0 ~ 4096

### get_encoder

- **Prototype**: `get_encoder(joint_id)`

- **Description**:Obtain the specified joint potential value.

- **Parameters**: `joint_id`: (`int`) 1 ~ 6

- **Returns**: `encoder`: 0 ~ 4096

### set_encoders

- **Prototype**: `set_encoders(encoders, sp)`

- **Description**: Set the six joints of the manipulator to execute synchronously to the specified position.

- **Parameters**:
  - `encoders`: A encoder list, length 6.
  - `sp`: speed 0 - 100

### get_encoders

- **Prototype**: `get_encoders()`

- **Description**: Get the six joints of the manipulator.

- **Returns**: the list of encoder (`list`)

## Running status and Settings

### get_speed

- **Prototype**: `get_speed()`

- **Description**: Get speed.

- **Returns**: speed: (`int`)

### set_speed

- **Prototype**: `set_speed(speed)`

- **Description**: Set speed.

- **Parameters**: speed: (`int`) 0 ~ 100

### set_joint_min

- **Prototype**: `set_joint_min(id, angle)`
- **Description**: Sets the minimum angle for the specified joint.

- **Parameters**:
  - `id`: (`int`) joint id 1-6.
  - `angle`: 0 - 180.

### set_joint_max

- **Prototype**: `set_joint_max(id, angle)`
- **Description**: Sets the maximum angle of the specified joint.

- **Parameters**:
  - `id`: (`int`) joint id 1-6.
  - `angle`: 0 - 180.

### get_joint_min_angle

- **Prototype**: `get_joint_min_angle()`
- **Description**: Gets the minimum movement angle of the specified joint

- **Parameters**: `joint_id`: (`int`)

- **Returns**: angle value (`float`)

### get_joint_max_angle

- **Prototype**: `get_joint_max_angle()`

- **Description**: Gets the maximum movement angle of the specified joint

- **Parameters**: `joint_id`: (`int`)

- **Returns**: angle value (`float`)

## Servo control

### is_servo_enable

- **Prototype**: `is_servo_enable(servo_id)`

- **Description**: Determine whether all steering gears are connected

- **Parameters**: `servo_id` (`int`) 1 ~ 6

- **Returns**

  - `0`: disable
  - `1`: enable
  - `-1`: error

### is_all_servo_enable

- **Prototype**: `is_all_servo_enable()`

- **Description**: Determine whether the specified steering gear is connected

- **Returns**

  - `0`: disable
  - `1`: enable
  - `-1`: error

### set_servo_data

- **Prototype**: `set_servo_data(servo_no, data_id, value)`
- **Description**: Set the data parameters of the specified address of the steering gear.

- **Parameters**:
  - `servo_no`: Serial number of articulated steering gear, 1 - 6.
  - `data_id`: Data address.
  - `value`: 0 - 4096

### get_servo_data

- **Prototype**: `get_servo_data(servo_no, data_id)`
- **Description**: Read the data parameter of the specified address of the steering gear.

- **Parameters**:

  - `servo_no`: Serial number of articulated steering gear, 1 - 6.
  - `data_id`: Data address.

- **Returns**: `value`: 0 - 4096

  - `0`: disable
  - `1`: enable
  - `-1`: error

### set_servo_calibration

- **Prototype**: `set_servo_calibration(servo_no)`
- **Description**: The current position of the calibration joint actuator is the angle zero point, and the corresponding potential value is 2048.

- **Parameters**:
  - `servo_no`: Serial number of articulated steering gear, 1 - 6.

### release_servo

- **Prototype**: `release_servo(servo_id)`

- **Description**: Power off designated servo

- **Parameters**: `servo_id`: 1 ~ 6

### focus_servo

- **Prototype**: `focus_servo(servo_id)`

- **Description**: Power on designated servo

- **Parameters**: `servo_id`: 1 ~ 6

## Atom IO

### set_color

- **Prototype**: `set_color(r, g, b)`

- **Description**: Set the color of the light on the top of the robot arm.

- **Parameters**

  - `r`: 0 ~ 255
  - `g`: 0 ~ 255
  - `b`: 0 ~ 255

### set_pin_mode

- **Prototype**: `set_pin_mode(pin_no, pin_mode)`
- **Description**: Set the state mode of the specified pin in atom.
- **Parameters**

  - `pin_no` (int): Pin number.
  - `pin_mode` (int): 0 - input, 1 - output, 2 - input_pullup

### set_digital_output()

- **Parameters**

  - `pin_no` (int):
  - `pin_signal` (int): 0 / 1

### get_digital_input()

- **Parameters**: `pin_no` (int)

- **Return**: signal value

<!-- ### set_pwm_mode()

- **Description**
- **Parameters**-->

### set_pwm_output()

- **Prototype**: `set_pwm_output(channel, frequency, pin_val)`

- **Description**: PWM control.

- **Parameters**

  - `channel` (`int`): IO number.
  - `frequency` (`int`): clock frequency
  - `pin_val` (`int`): Duty cycle 0 ~ 256; 128 means 50%

### get_gripper_value

- **Prototype**: `get_gripper_value()`

- **Description**: Get gripper value

- **Return**: gripper value (int)

### set_gripper_state

- **Prototype**: `set_gripper_state(flag, speed)`

- **Description**: Set gripper switch state

- **Parameters**

  - `flag` (`int`): 0 - open, 1 - close
  - `speed` (`int`): 0 ~ 100

### set_gripper_value

- **Prototype**: `set_gripper_value(value, speed)`

- **Description**: Set gripper value

- **Parameters**

  - `value` (int): 0 ~ 100
  - `speed` (int): 0 ~ 100

### set_gripper_ini

- **Prototype**: `set_gripper_ini()`

- **Description**: Set the current position to zero, set current position value is `2048`.

### is_gripper_moving

- **Prototype**: `is_gripper_moving()`

- **Description**: Judge whether the gripper is moving or not

- **Returns**

  - `0` : not moving
  - `1` : is moving
  - `-1`: error data

### get_basic_input

- **Prototype**: `get_basic_input(pin_no)`

- **Description**: Get bottom pin.

- **Parameters**

  - `pin_no` (`int`) Pin number.

- **Return**: `pin_signal` (`int`) 0 / 1

### set_basic_output

- **Prototype**: `set_basic_output(pin_no, pin_signal)`

- **Description**: Set bottom pin.

- **Parameters**

  - `pin_no` (`int`) Pin number.
  - `pin_signal` (`int`): 0 / 1

### set_ssid_pwd

- **Prototype**: `set_ssid_pwd(account, password)`

- **Description**: Change connected wifi.(Apply to m5 or seeed)

- **Parameters**

  - `account` (`str`) new wifi account.
  - `password` (`str`): new wifi password.

### get_ssid_pwd

- **Prototype**: `get_ssid_pwd()`

- **Description**: Get connected wifi account and password.(Apply to m5 or seeed)

- **Return**: `(account, password)`.

### set_server_port

- **Prototype**: `set_server_port(port)`

- **Description**: Change the connection port of the server.

- **Parameters**

  - `port`: (`int`) The new connection port of the server.

### get_tof_distance

- **Prototype**: `get_tof_distance()`

- **Description**: Get the detected distance (Requires external distance detector).

- **Return**: `int` The unit is mm.

### get_tool_reference

- **Prototype**: `get_tool_reference()`

- **Description**: Get tool coordinate system.

- **Return**: `list` [x, y, z, rx, ry, rz].

### set_tool_reference

- **Prototype**: `set_tool_reference(coords)`

- **Description**: Set tool coordinate system.

- **Parameters**:
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

### set_world_reference

- **Prototype**: `set_world_reference(coords)`

- **Description**: Set world coordinate system.

- **Parameters**:
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

### get_world_reference

- **Prototype**: `get_world_reference()`

- **Description**: Get world coordinate system.

- **Return**: `list` [x, y, z, rx, ry, rz].

### set_reference_frame

- **Prototype**: `set_reference_frame(rftype)`

- **Description**: Set base coordinate system.

- **Parameters**:
  - `rftype`: 0 - base 1 - tool.

### get_reference_frame

- **Prototype**: `get_reference_frame()`

- **Description**: Get base coordinate system.

- **Return**: 0 - base 1 - tool.

### set_movement_type

- **Prototype**: `set_movement_type(move_type)`

- **Description**: Set movement type.

- **Parameters**:
  - `move_type`: 1 - movel, 0 - moveJ.

### get_movement_type

- **Prototype**: `get_movement_type()`

- **Description**: Get movement type.

- **Return**: 1 - movel, 0 - moveJ.

### set_end_type

- **Prototype**: `set_end_type(end)`

- **Description**: Set end coordinate system.

- **Parameters**:
  - `end`:  0 - flange, 1 - tool.

### get_end_type

- **Prototype**: `get_end_type()`

- **Description**: Get end coordinate system.

- **Return**: 0 - flange, 1 - tool.

### get_plan_speed

- **Prototype**: `get_plan_speed()`

- **Description**: Get planning speed.

- **Return**: [ `movel planning speed`, `movej planning speed`].

### get_plan_acceleration

- **Prototype**: `get_plan_acceleration()`

- **Description**: Get planning acceleration.

- **Return**: [ `movel planning acceleration`, `movej planning acceleration`].

### set_plan_speed

- **Prototype**: `set_plan_speed(speed, is_linear)`

- **Description**: Set planning speed.

- **Parameters**

  - `speed` (`int`) 0 - 100.
  - `is_linear` (`int`): 0 / 1 (0 ->joint, 1 -> line)

### set_plan_acceleration

- **Prototype**: `set_plan_acceleration(acceleration, is_linear)`

- **Description**: Set planning acceleration.

- **Parameters**

  - `acceleration` (`int`) 0 - 100.
  - `is_linear` (`int`): 0 / 1 (0 ->joint, 1 -> line)

### get_servo_speeds

- **Prototype**: `get_servo_speeds()`

- **Description**: Get joint velocity.

- **Return**: `list` Speed of each joint.

### get_servo_currents

- **Prototype**: `get_servo_currents()`

- **Description**: Get joint current.

- **Return**: `list` Current of each joint.

### get_servo_voltages

- **Prototype**: `get_servo_voltages()`

- **Description**: Get joint voltage.

- **Return**: `list` Voltage of each joint.

### get_servo_status

- **Prototype**: `get_servo_status()`

- **Description**: Get the state of each joint.

- **Return**: `list` the state of each joint.

### get_servo_temps

- **Prototype**: `get_servo_temps()`

- **Description**: Get the temperature of each joint.

- **Return**: `list` temperature of each joint.

## Raspberry pi -- GPIO

### gpio_init

- **Prototype**: `gpio_init()`

- **Description**: Init GPIO module, and set BCM mode.

### gpio_output

- **Prototype**: `gpio_output(pin, v)`

- **Description**: Set GPIO port output value.

- **Parameters**

  - `pin` (`int`) Pin number.
  - `v` (`int`): 0 / 1

# Angle

```python
from pymycobot import Angle
```

**Description**

Instance class of joint. It's recommended to use this class to select joint.

# Coord

```python
from pymycobot import Coord
```

**Description**

Instance class of coord. It's recommended to use this class to select coord.

# utils (Module)

This module support some help method.

**Usage:**

```python
from pymycobot import utils
```

## get_port_list

- **Prototype**: `get_port_list()`

- **Description**: Get the all serial port list.

- **Return**: serial port list (`list`)

## detect_port_of_basic

- **Prototype**: `detect_port_of_basic()`

- **Description**: Returns the serial port string of the first detected M5 Basic. If it is not found, it returns `None`.

- **Return**: detected port (`str`) or `None`

- **Example**:

  ```python
  from pymycobot import MyCobot, utils

  port = utils.detect_port_of_basic()
  if port is None:
   raise Exception('Detection failed.')
  mycobot = MyCobot(port, 115200)
  ```

# MyCobotSocket

> Note:
> raspberryPi version Only supports python3
> The robotic arm that uses this class of premise has a server and has been turned on.

Use TCP/IP to control the robotic arm

### Client

```python
# demo
from pymycobot import MyCobotSocket
# Port 9000 is used by default
mc = MyCobotSocket("192.168.10.10","9000")

mc.connect("/dev/ttyAMA0","1000000")

res = mc.get_angles()
print(res)

mc.send_angles([0,0,0,0,0,0],20)
...

```

### Server

Server file is in the demo folder

## socket control

> Note:
> Most of the methods are the same as the class mycobot, only the new methods are listed here.

### connect

- **Prototype**: `connect(serialport, baudrate, timeout)`

- **Description**: Connect the robot arm through the serial port and baud rate

- **Parameters**

  - `serialport`: (`str`) default `/dev/ttyAMA0`.
  - `baudrate`: default `1000000`.
  - `timeout`: default `0.1`.

### set_gpio_mode

- **Prototype**: `set_gpio_mode(mode)`

- **Description**: Set pin coding method.

- **Parameters**

  - `mode` (`str`) "BCM" or "BOARD".

### set_gpio_out

- **Prototype**: `set_gpio_out(pin_no, mode)`

- **Description**: Set the pin as input or output.

- **Parameters**

  - `pin_no` (`int`) pin id.
  - `mode` (`str`) "in" or "out"

### set_gpio_output

- **Prototype**: `set_gpio_output(pin_no, state)`

- **Description**: Set the pin to high or low level.

- **Parameters**

  - `pin_no` (`int`) pin id.
  - `state` (`int`) 0 or 1

### get_gpio_in

- **Prototype**: `get_gpio_in(pin_no)`

- **Description**: Get pin level status.

- **Parameters**

  - `pin_no` (`int`) pin id.

```python
# Set up the demo of the suction pump
import time
from pymycobot import MyCobotSocket
# connect server
m = MyCobotSocket("192.168.10.10", '9000')
# default serialport:/dev/ttyAMA0 Baudrate:1000000
m.connect()

m.set_gpio_mode("BCM")
m.set_gpio_out(20, "out")
m.set_gpio_out(21, "out")
# open
m.set_gpio_output(20, 0)
m.set_gpio_output(21, 0)
time.sleep(3)
# close
m.set_gpio_output(20, 1)
m.set_gpio_output(21, 1)

```

# MyBuddy

MyBuddy Python API

### focus_servo(id, servo_id)

Power on designated servo

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** – 1 - 6

### get_acceleration(id)

Read acceleration during all moves

- **Parameters**

    **id** – 1/2/3 (L/R/W)

### get_angles(id)

Get the degree of all joints.

- **Parameters**

    **id** – 1/2/3 (L/R/W)

- **Returns**

    A float list of all degree.

- **Return type**

    list

### get_basic_input(pin_no)

Get basic input for M5 version.

- **Parameters**

    **pin_no** – (int) pin port number (0 - 20).

### get_coords(id)

- **Parameters**

    **id** – 1/2/3 (L/R/W).

### get_digital_input(id, pin_no)

singal value

- **Parameters**

  - **id** – 1/2 (L/R)

  - **pin_no** (_int_) – 1 - 5

### get_encoder(id, joint_id)

Obtain the specified joint potential value.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** – (int) 1 ~ 6

- **Returns**

    0 ~ 4096

### get_encoders(id)

Get the six joints of the manipulator

- **Parameters**

    **id** – 1/2/3 (L/R/W).

- **Returns**

    The list of encoders

### get_end_type(id)

Get end coordinate system

- **Parameters**

    **id** – 0/1/2 (ALL/L/R)

- **Returns**

    0 - flange
    1 - tool

### get_gripper_value(id)

Get the value of gripper.

- **Parameters**

    **id** – 1/2 (L/R)

- **Returns**

    gripper value (int)

### get_joint_current(id, joint_id)

Get Collision Current

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **joint_id** – 1 - 6

### get_joint_max_angle(id, joint_id)

Gets the maximum movement angle of the specified joint

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** – (int) 1 - 6

- **Returns**

    angle value(float)

### get_joint_min_angle(id, joint_id)

Gets the minimum movement angle of the specified joint

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** – (int) 1 - 6

- **Returns**

    angle value(float)

### get_movement_type(id)

Get movement type

- **Parameters**

    **id** – 0/1/2 (ALL/L/R)

- **Returns**

    1 - movel
    0 - moveJ

### get_plan_acceleration()

Get planning acceleration

- **Returns**

    [movel planning acceleration, movej planning acceleration].

### get_plan_speed()

Get planning speed

- **Returns**

    [movel planning speed, movej planning speed].

### get_reference_frame(id)

Get the base coordinate system

- **Parameters**

    **id** – 0/1/2 (ALL/L/R)

- **Returns**

    0 - base 1 - tool.

### get_robot_id()

Detect this robot id

### get_robot_version()

Get cobot version

- **Returns**

    1
    mycobotPro: 101

- **Return type**

    mycobot

### get_servo_currents(id)

Get joint current

- **Parameters**

    **id** – 1/2/3 (L/R/W)

- **Returns**

    value mA

### get_servo_data(id, servo_no, data_id)

Read the data parameter of the specified address of the steering gear.

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

  - **data_id** – Data address.

- **Returns**

    0 - 4096
    0 - disable
    1 - enable
    -1 - error

- **Return type**

    values

### get_servo_status(id)

Get joint status

- **Parameters**

    **id** – 1/2/3 (L/R/W)

- **Returns**

    [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error

### get_servo_temps(id)

Get joint temperature

- **Parameters**

    **id** – 1/2/3 (L/R/W)

### get_servo_voltages(id)

Get joint voltages

- **Parameters**

    **id** – 1/2/3 (L/R/W)

- **Returns**

    volts < 24 V

### get_speed(id)

Get speed

- **Parameters**

    **id** – 1/2/3 (L/R/W).

- **Returns**

    (int)

- **Return type**

    speed

### get_system_version()

Get cobot version

- **Returns**

    1
    mycobotPro: 101

- **Return type**

    mycobot

### get_tool_reference(id)

Get tool coordinate system

- **Parameters**

    **id** – 0/1/2 (ALL/L/R)

### get_world_reference(id)

Get the world coordinate system

- **Parameters**

    **id** – 0/1/2 (ALL/L/R)

### is_all_servo_enable(id)

Determine whether the specified steering gear is connected

- **Parameters**

    **id** – 1/2/3 (L/R/W)

- **Returns**

    0 - disable
    1 - enable
    -1 - error

### is_controller_connected()

Wether connected with Atom.

### is_free_mode(id)

Check if it is free mode

- **Parameters**

    **id** – 0/1/2/3 (ALL/L/R/W)

- **Returns**

    0/1

### is_gripper_moving(id)

Judge whether the gripper is moving or not

- **Parameters**

    **id** – 1/2 (L/R)

- **Returns**

    not moving
    1 : is moving
    -1: error data

- **Return type**

    0

### is_in_position(id, mode, data)

Judge whether in the position.

- **Parameters**

  - **id** – ALL/L/R/W (0/1/2/3).

  - **mode** – 1 - coords, 0 - angles

  - **data** – A data list, angles or coords, length 6.

- **Returns**

    True
    0 : False
    -1: Error

- **Return type**

    1

### is_moving(id)

- **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

- **Returns**

    not moving
    1 : is moving
    -1 : error data

- **Return type**

    0

### is_paused(id)

Judge whether the manipulator pauses or not.

- **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

- **Returns**

    1 - paused
    0 - not paused
    -1 - error

### is_power_on()

Adjust robot arm status

- **Returns**

    power on
    0 : power off
    -1: error data

- **Return type**

    1

### is_servo_enable(id, servo_id)

Determine whether all steering gears are connected

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** – (int) 1 ~ 6

- **Returns**

    0 - disable
    1 - enable
    -1 - error

### jog_absolute(id, joint_id, angle, speed)

Absolute joint control

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** – int 1-6.

  - **angle** – int

  - **speed** – int (0 - 100)

### jog_angle(id, joint_id, direction, speed)

Jog control angle.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** – int 1-6.

  - **direction** – 0 - decrease, 1 - increase

  - **speed** – int (0 - 100)

### jog_coord(id, coord_id, direction, speed)

Jog control coord.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **coord_id** – int 1-6 (x/y/z/rx/ry/rz).

  - **direction** – 0 - decrease, 1 - increase

  - **speed** – int (0 - 100)

### jog_increment(id, joint_id, increment, speed)

step mode

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** – int 1-6.

  - **increment** – # TODO 未注明

  - **speed** – int (1 - 100)

### jog_stop(id)

Stop jog moving
:param id: 1/2/3 (L/R/W).

### joint_brake(id, joint_id)

Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** – 1 - 6

### pause(id)

Pause movement

- **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

### power_off()

Close communication with Atom.

### power_on()

Open communication with Atom.

### release_all_servos()

### release_servo(id, servo_id)

Power off designated servo

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** – 1 - 6.

### resume(id)

Recovery movement

- **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

### send_angle(id, joint, angle, speed)

Send one degree of joint to robot arm.

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint** – 1 ~ 6

  - **angle** – int

  - **speed** – 1 ~ 100

### send_angles(id, degrees, speed, mode)

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **degrees** – [angle_list]

  - **speed** – 1 - 100

  - **mode** – 0 - with interpolation 1 - No interpolation 2 - reserved.

### send_coord(id, coord, data, speed)

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **coord** – 1 ~ 6 (x/y/z/rx/ry/rz)

  - **data** – int

  - **speed** – 0 ~ 100

### send_coords(id, coords, speed, mode)

Send all coords to robot arm.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **coords** – a list of coords value(List[float]), length 6, [x(mm), y, z, rx(angle), ry, rz]

  - **speed** – (int) 0 ~ 100

  - **mode** – (int) 0 - moveJ, 1 - moveL, 2 - moveC

### set_acceleration(id, acc)

Read acceleration during all moves

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **acc** – 1 - 100

### set_basic_mode(pin_no, pin_mode)

Set base IO, input or output mode

- **Parameters**

  - **pin_no** – 1 - 5

  - **pin_mode** – 0 - input 1 - output

### set_basic_output(pin_no, pin_signal)

Set basic output.

- **Parameters**

  - **pin_no** – pin port number (0 - 20).

  - **pin_signal** – 0 / 1 (0 - low, 1 - high)

### set_color(r=0, g=0, b=0)

Set the light color on the top of the robot arm.

- **Parameters**

  - **r** (_int_) – 0 ~ 255

  - **g** (_int_) – 0 ~ 255

  - **b** (_int_) – 0 ~ 255

### set_digital_output(id, pin_no, pin_signal)

- **Parameters**

  - **id** – 1/2 (L/R)

  - **pin_no** (_int_) – 1 - 5

  - **pin_signal** (_int_) – 0 / 1

### set_encoder(id, joint_id, encoder, speed)

Set a single joint rotation to the specified potential value.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** – 1 - 6.

  - **encoder** – The value of the set encoder.

### set_encoders(id, encoders, speed, mode)

Set the six joints of the manipulator to execute synchronously to the specified position.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **encoders** – A encoder list, length 6.

  - **speed** – speed 1 ~ 100

  - **mode** – 0 - with interpolation 1 - No interpolation 2 - reserved.

### set_end_type(id, end)

Set end coordinate system

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **end** – 0 - flange, 1 - tool

### set_free_mode(id)

set free mode

- **Parameters**

    **id** – 0/1/2/3 (ALL/L/R/W)

### set_gripper_calibration(id)

Set the current position to zero, set current position value is 2048.
:param id: 1/2 (L/R)

### set_gripper_state(id, flag)

Set gripper switch state

- **Parameters**

  - **id** – 1/2 (L/R)

  - **flag** (_int_) – 0 - close, 1 - open

### set_gripper_value(id, value)

Set gripper value

- **Parameters**

  - **id** – 1/2 (L/R)

  - **value** (_int_) – 0 ~ 100

### set_joint_current(id, joint_id, current)

Set Collision Current

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **joint_id** – 1 - 6

  - **current** – current value

### set_joint_max(id, joint_id, angle)

Set the joint maximum angle

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** – int 1-6.

  - **angle** – 0 ~ 180

### set_joint_min(id, joint_id, angle)

Set the joint minimum angle

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** – int 1-6.

  - **angle** – 0 ~ 180

### set_movement_type(id, move_type)

Set movement type

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **move_type** – 1 - movel, 0 - moveJ

### set_pin_mode(id, pin_no, pin_mode)

Set the state mode of the specified pin in atom.

- **Parameters**

  - **id** – 1/2 (L/R)

  - **pin_no** (_int_) – pin number (1 - 5).

  - **pin_mode** (_int_) – 0 - input, 1 - output

### set_plan_acceleration(acceleration, is_linear)

Set planning acceleration

- **Parameters**

  - **acceleration** (_int_) – (0 ~ 100).

  - **is_linear** – 0 -> joint 1 -> straight line

### set_plan_speed(speed, is_linear)

Set planning speed

- **Parameters**

  - **speed** (_int_) – (0 ~ 100).

  - **is_linear** – 0 -> joint 1 -> straight line

### set_pwm_output(id, channel, frequency, pin_val)

PWM control

- **Parameters**

  - **id** – 1/2 (L/R)

  - **channel** (_int_) – IO number (1 - 5).

  - **frequency** (_int_) – clock frequency (0/1: 0 - 1Mhz 1 - 10Mhz)

  - **pin_val** (_int_) – Duty cycle 0 ~ 100: 0 ~ 100%

### set_reference_frame(id, rftype)

Set the base coordinate system

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **rftype** – 0 - base 1 - tool.

### set_robot_id(id)

Set this robot id

### set_servo_calibration(id, servo_no)

The current position of the calibration joint actuator is the angle zero point,

    and the corresponding potential value is 2048.

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

### set_servo_data(id, servo_no, data_id, value)

Set the data parameters of the specified address of the steering gear

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

  - **data_id** – Data address.

  - **value** – 0 - 4096

### set_speed(id, speed)

Set speed value

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **speed** (_int_) – 0 - 100

### set_tool_reference(id, coords)

Set tool coordinate system

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **coords** – a list of coords value(List[float]), length 6.
    for mycobot :[x(mm), y, z, rx(angle), ry, rz]

### set_world_reference(id, coords)

Set the world coordinate system

- **Parameters**

  - **id** – 0/1/2 (ALL/L/R)

  - **coords** – a list of coords value(List[float]), length 6 [x(mm), y, z, rx(angle), ry, rz]

### stop(id)

Stop moving

- **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

---
More demo can go to [here](../demo).
