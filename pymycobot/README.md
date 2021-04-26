# pymycobot

**This is python API for mycobot.**

We support Python2, Python3.5 or later.

<details>
<summary>Class:</summary>

<!-- vim-markdown-toc GFM -->

* [MyCobot](#mycobot)
    * [Overall status](#overall-status)
        * [power_on()](#power_on)
        * [power_off()](#power_off)
        * [is_power_on()](#is_power_on)
        * [set_free_mode()](#set_free_mode)
    * [MDI mode and operation](#mdi-mode-and-operation)
        * [get_angles()](#get_angles)
        * [send_angle()](#send_angle)
        * [send_angles()](#send_angles)
        * [get_radians()](#get_radians)
        * [send_radians()](#send_radians)
        * [get_coords()](#get_coords)
        * [send_coord()](#send_coord)
        * [send_coords()](#send_coords)
        * [sync_send_angles()](#sync_send_angles)
        * [sync_send_coords()](#sync_send_coords)
        * [pause()](#pause)
        * [resume()](#resume)
        * [stop()](#stop)
        * [is_paused()](#is_paused)
        * [is_in_position()](#is_in_position)
    * [JOG mode and operation](#jog-mode-and-operation)
        * [jog_angle()](#jog_angle)
        * [jog_coord()](#jog_coord)
        * [jog_stop()](#jog_stop)
    * [Running status and Settings](#running-status-and-settings)
        * [get_speed()](#get_speed)
        * [set_speed()](#set_speed)
        * [get_joint_min_angle()](#get_joint_min_angle)
        * [get_joint_max_angle()](#get_joint_max_angle)
    * [Servo control](#servo-control)
        * [is_servo_enable()](#is_servo_enable)
        * [is_all_servo_enable()](#is_all_servo_enable)
        * [release_servo()](#release_servo)
        * [focus_servo()](#focus_servo)
    * [Atom IO](#atom-io)
        * [set_color()](#set_color)
        * [set_pin_mode()](#set_pin_mode)
        * [set_digital_output()](#set_digital_output)
        * [get_digital_input()](#get_digital_input)
        * [set_pwm_output()](#set_pwm_output)
        * [get_gripper_value()](#get_gripper_value)
        * [set_gripper_state()](#set_gripper_state)
        * [set_gripper_value()](#set_gripper_value)
        * [set_gripper_ini()](#set_gripper_ini)
        * [is_gripper_moving()](#is_gripper_moving)
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

### power_on()

- **Description**: Robot arm power up.

### power_off()

- **Description**: Robot arm power down.

### is_power_on()

- **Description**: Adjust robot arm whether power on.

- **Returns**

  - `1`: power on

  - `0`: power off
  - `-1`: error

### release_all_servos()

- **Description**: Set robot arm into free moving mode.

## MDI mode and operation

### get_angles()

- **Description**: Get the degree of all joints.

- **Returns**: `list`: A float list of all degree.

### send_angle()

- **Description**: Send one degree of joint to robot arm.

- **Parameters**

  - `id`: Joint id(`genre.Angle`)
  - `degree`: degree value(`float`)
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angle(Angle.J2.value, 10, 50)
  ```

### send_angles()

- **Description**: Send the degrees of all joints to robot arm.

- **Parameters**

  `degrees`: a list of degree value(`List[float]`)

  `speed`: (`int`)

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles([0,0,0,0,0,0], 80)
  ```

### get_radians()

- **Description**: Get the radians of all joints.

- **Returns**: `list`: A float list of radian.

### send_radians()

- **Description**: Send the radians of all joint to robot arm.

- **Parameters**:

  - `degrees`: a list of radian value(`List[float]`)
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles_by_radian([1,1,1,1,1,1], 70)
  ```

### get_coords()

- **Description**: Get the Coords from robot arm, coordinate system based on base.

- **Returns**: `list`: A float list of coord - `[x, y, z, rx, ry, rz]`

### send_coord()

- **Description**: Send one coord to robot arm.

- **Parameters**

  - `id`: coord id(`genre.Coord`)
  - `coord`: coord value(`float`)
  - `speed`: (`int`) 0 ~ 100

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coord(Coord.X.value, -40, 70)
  ```

### send_coords()

- **Description**: Send all coords to robot arm.

- **Parameters**

  - `coords`: a list of coords value(`List[float]`)
  - `speed`: (`int`) 0 ~ 100
  - `mode`: (`int`): `0` - angluar, `1` - linear

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
  ```

### sync_send_angles()

- **Description**: Send the angle in synchronous state and return when the target point is reached

- **Parameters**

  - `id`: Joint id(`genre.Angle`)
  - `degree`: degree value(`float`)
  - `speed`: (`int`) 0 ~ 100

### sync_send_coords()

- **Description**: Send the coord in synchronous state and return when the target point is reached

- **Parameters**

  - `coords`: a list of coords value(`List[float]`)
  - `speed`: (`int`) 0 ~ 100
  - `mode`: (`int`): `0` - angluar, `1` - linear

### pause()

- **Description**: Pause movement.

### resume()

- **Description**: Recovery movement.

### stop()

- **Description**: Stop moving.

### is_paused()

- **Description**: Judge whether the manipulator pauses or not.

- **Returns** :

  - `1` - paused
  - `0` - not paused
  - `-1` - error

### is_in_position()

- **Description**: Judge whether in the position.

- **Parameters**

  - `data`: A data list, angles or coords.
  - `flag`: Tag the data type, `0` - angles, `1` - coords.

- **Returns**

  - `1` - true
  - `0` - false
  - `-1` - error

## JOG mode and operation

### jog_angle()

- **Description**: Jog control angle

- **Parameters**

  - `joint_id`: (`int`) 1 ~ 6
  - `direction`: `0` - decrease, `1` - increase
  - `speed`: 0 ~ 100

### jog_coord()

- **Description**: Jog control coord.

- **Parameters**

  - `coord_id`: (`int`) 1 ~ 6
  - `direction`: `0` - decrease, `1` - increase
  - `speed`: 0 ~ 100

### jog_stop()

- **Description**: Stop jog moving.

## Running status and Settings

### get_speed()

- **Description**: Get speed.

- **Returns**: speed: (`int`)

### set_speed()

- **Description**: Set speed.

- **Parameters**: speed: (`int`) 0 ~ 100

### get_joint_min_angle()

- **Description**: Gets the minimum movement angle of the specified joint

- **Parameters**: `joint_id`: (`int`)

- **Returns**: angle value (`float`)

### get_joint_max_angle()

- **Description**: Gets the maximum movement angle of the specified joint

- **Parameters**: `joint_id`: (`int`)

- **Returns**: angle value (`float`)

## Servo control

### is_servo_enable()

- **Description**: Determine whether all steering gears are connected

- **Parameters**: `servo_id` (`int`) 1 ~ 6

- **Returns**

  - `0`: disable
  - `1`: enbale
  - `-1`: error

### is_all_servo_enable()

- **Description**: Determine whether the specified steering gear is connected

- **Returns**

  - `0`: disable
  - `1`: enbale
  - `-1`: error

### release_servo()

- **Description**: Power off designated servo

- **Parameters**: `servo_id`: 1 ~ 6

### focus_servo()

- **Description**: Power on designated servo

- **Parameters**: `servo_id`: 1 ~ 6

## Atom IO

### set_color()

- **Description**: Set the color of the light on the top of the robot arm.

- **Parameters**

  - `r`: 0 ~ 255
  - `g`: 0 ~ 255
  - `b`: 0 ~ 255

### set_pin_mode()

- **Parameters**

  - `pin_no` (int):
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
- **Parameters**

### set_pwm_output()

- **Description**
- **Parameters** -->

### get_gripper_value()

- **Description**: Get gripper value

- **Return**: gripper value (int)

### set_gripper_state()

- **Description**: Set gripper switch state

- **Parameters**

  - `flag` (`int`): 0 - open, 1 - close
  - `speed` (`int`): 0 ~ 100

### set_gripper_value()

- **Description**: Set gripper value

- **Parameters**

  - `value` (int): 1400 ~ 2048
  - `speed` (int): 0 ~ 100

### set_gripper_ini()

- **Description**: Set the current position to zero, set current position value is `2048`.

### is_gripper_moving()

- **Description**: Judge whether the gripper is moving or not

- **Returns**

  - `0` : not moving
  - `1` : is moving
  - `-1`: error data

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
