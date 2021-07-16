# pymycobot

**This is python API for mycobot.**

We support Python2, Python3.5 or later.

<details>
<summary>Class:</summary>

<!-- vim-markdown-toc GFM -->

* [MyCobot](#mycobot)
    * [Overall status](#overall-status)
        * [power_on](#power_on)
        * [power_off](#power_off)
        * [is_power_on](#is_power_on)
        * [release_all_servos](#release_all_servos)
        * [is_controller_connected](#is_controller_connected)
    * [MDI mode and operation](#mdi-mode-and-operation)
        * [get_angles](#get_angles)
        * [send_angle](#send_angle)
        * [send_angles()](#send_angles)
        * [get_radians](#get_radians)
        * [send_radians](#send_radians)
        * [get_coords](#get_coords)
        * [send_coord](#send_coord)
        * [send_coords](#send_coords)
        * [sync_send_angles](#sync_send_angles)
        * [sync_send_coords](#sync_send_coords)
        * [pause](#pause)
        * [resume](#resume)
        * [stop](#stop)
        * [is_paused](#is_paused)
        * [is_in_position](#is_in_position)
    * [JOG mode and operation](#jog-mode-and-operation)
        * [jog_angle](#jog_angle)
        * [jog_coord](#jog_coord)
        * [jog_stop](#jog_stop)
        * [set_encoder](#set_encoder)
        * [get_encoder](#get_encoder)
        * [set_encoders](#set_encoders)
    * [Running status and Settings](#running-status-and-settings)
        * [get_speed](#get_speed)
        * [set_speed](#set_speed)
        * [get_joint_min_angle](#get_joint_min_angle)
        * [get_joint_max_angle](#get_joint_max_angle)
    * [Servo control](#servo-control)
        * [is_servo_enable](#is_servo_enable)
        * [is_all_servo_enable](#is_all_servo_enable)
        * [set_servo_data](#set_servo_data)
        * [get_servo_data](#get_servo_data)
    * [set_servo_calibration](#set_servo_calibration)
        * [release_servo](#release_servo)
        * [focus_servo](#focus_servo)
    * [Atom IO](#atom-io)
        * [set_color](#set_color)
        * [set_pin_mode](#set_pin_mode)
        * [set_digital_output()](#set_digital_output)
        * [get_digital_input()](#get_digital_input)
        * [set_pwm_output()](#set_pwm_output)
        * [get_gripper_value](#get_gripper_value)
        * [set_gripper_state](#set_gripper_state)
        * [set_gripper_value](#set_gripper_value)
        * [set_gripper_ini](#set_gripper_ini)
        * [is_gripper_moving](#is_gripper_moving)
    * [Basic](#basic)
        * [set_basic_output](#set_basic_output)
* [Angle](#angle)
* [Coord](#coord)

<!-- vim-markdown-toc -->
</details>

# MyCobot

```python
from pymycobot.mycobot import MyCobot
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

- **Prototype**: `get_coords(id, coord, speed)`

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

## Running status and Settings

### get_speed

- **Prototype**: `get_speed()`

- **Description**: Get speed.

- **Returns**: speed: (`int`)

### set_speed

- **Prototype**: `set_speed(speed)`

- **Description**: Set speed.

- **Parameters**: speed: (`int`) 0 ~ 100

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

## set_servo_calibration

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

  - `value` (int): 1400 ~ 2048
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

## Basic

### set_basic_output

- **Prototype**: `set_basic_output(pin_no, pin_signal)`

- **Description**: Set bottom pin.

- **Parameters**

  - `pin_no` (`int`) Pin number.
  - `pin_signal` (`int`): 0 / 1

# Angle

```python
from pymycobot.genre import Angle
```

**Description**

Instance class of joint. It's recommended to use this class to select joint.

# Coord

```python
from pymycobot.genre import Coord
```

**Description**

Instance class of coord. It's recommended to use this class to select coord.

---
