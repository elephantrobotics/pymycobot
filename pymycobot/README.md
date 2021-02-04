# pymycobot

**This is python API for mycobot.**

We support Python2, Python3.5 or later.

<!--If you want to use the api, make sure `pyserial` is installed.-->

<!--```bash-->
<!--pip2 install pyserial-->
<!--# or-->
<!--pip3 install pyserial-->
<!--```-->

**Class**:

- [MyCobot](#MyCobot)
  - [Overall status](#Overall-status)
  - [MDI mode and operation](#MDI-mode-and-operation)
  - [JOG mode and operation](#JOG-mode-and-operation)
  - [Running status and Settings](#Running-status-and-Settings)
  - [Servo control](#Servo-control)
  - [Atom IO](#Atom-IO)
- [Angle](#Angle)
- [Coord](#Coord)

# MyCobot

```python
from pymycobot.mycobot import MyCobot
```

## Overall status

### power_on()

- **Description**

  Robot arm power up.

### power_off()

- **Description**

  Robot arm power down.

### is_power_on()

- **Description**

  Adjust robot arm whether power on.

- **Returns**

  - `1`: power on
  - `0`: power off
  - `-1`: error

### set_free_mode()

- **Description**

  Robot arm into free moving mode.

## MDI mode and operation

### get_angles()

- **Description**

  Get the degree of all joints.

- **Returns**

  `list`: A float list of degree.

### send_angle()

- **Description**

  Send one degree of joint to robot arm.

- **Parameters**

  id: Joint id(`genre.Angle`)

  degree: degree value(`float`)

  speed: (`int`)

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angle(Angle.J2.value, 10, 50)
  ```

### send_angles()

- **Description**

  Send the degrees of all joints to robot arm.

- **Parameters**

  degrees: a list of degree value(`List[float]`)

  speed: (`int`)

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles([0,0,0,0,0,0], 80)
  ```

### get_radians()

- **Description**

  Get the radians of all joints.

- **Returns**

  `list`: A float list of radian.

### send_radians()

- **Description**

  Send the radians of all joint to robot arm.

- **Parameters**

  degrees: a list of radian value(`List[float]`)

  speed: (`int`)

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Angle


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_angles_by_radian([1,1,1,1,1,1], 70)
  ```

### get_coords()

- **Description**

  Get the Coords from robot arm.

- **Returns**

  `list`: A float list of coord.

### send_coord()

- **Description**

  Send one coord to robot arm.

- **Parameters**

  id: coord id(`genre.Coord`)

  coord: coord value(`float`)

  speed: (`int`)

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coord(Coord.X.value, -40, 70)
  ```

### send_coords()

- **Description**

  Send all coords to robot arm.

- **Parameters**

  coords: a list of coords value(`List[float]`)

  speed: (`int`)
  
  mode: (`int`): `0` - angluar, `1` - linear

- **Example**

  ```python
  from pymycobot.mycobot import MyCobot
  from pymycobot.genre import Coord


  mycobot = MyCobot('/dev/ttyUSB0')
  mycobot.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
  ```

### pause()

- **Description**

  Pause movement.

### resume()

- **Description**

  Recovery movement.

### stop()

- **Description**

  Stop moving.

### is_paused()

- **Description**

  Judge whether the manipulator pauses or not.

- **Returns**

  `bool`:

  - `1` - paused
  - `0` - not paused
  - `-1` - error

### is_in_position()

- **Description**

  Judge whether in the position.

- **Returns**

  `bool`:

  - `1` - true
  - `0` - false
  - `-1` - error

### is_moving()

- **Description**

  Judge whether the manipulator is moving or not.

- **Returns**

  `bool`: `True` - moving, `False` - not moving.

## JOG mode and operation

### jog_angle()

- **Description**

  Jog control angle

- **Parameters**

  joint id: (`int`) 1 ~ 6

### jog_coord()

- **Description**

  Jog control coord.

- **Parameters**

  coord id: (`int`) 1 ~ 6

### jog_stop()

- **Description**

  Stop jog move.

## Running status and Settings

### get_speed()

- **Description**

  Get speed.

- **Returns**

  speed: (`int`)

### set_speed()

- **Description**

  Set speed.

- **Parameters**

  speed: (`int`) 0 ~ 100

### get_joint_min_angle()

- **Description**

  Gets the minimum movement angle of the specified joint

- **Parameters**

  joint id: (`int`)

- **Returns**

  angle: (`float`)

### get_joint_max_angle()

- **Description**

  Gets the maximum movement angle of the specified joint

- **Parameters**

  joint id: (`int`)

- **Returns**

  angle: (`float`)

## Servo control

### is_servo_enable()

- **Description**

  Determine whether all steering gears are connected

- **Parameters**

  servo id: (`int`)

- **Returns**

  flag: (`int`)

  - `0`: disable
  - `1`: enbale
  - `-1`: error

### is_all_servo_enable()

- **Description**

  Determine whether the specified steering gear is connected

- **Returns**

  flag: (`int`)

  - `0`: disable
  - `1`: enbale
  - `-1`: error

### release_servo()

- **Description**

  Power on designated servo

- **Parameters**

  servo_id: 1 ~ 6

### focus_servo()

- **Description**

  Power off designated servo

- **Parameters**

  servo_id: 1 ~ 6

## Atom IO

### set_led_color()

- **Description**

  Set the color of the light on the top of the robot arm.

- **Parameters**

  rgb: (`string`) like: "FF0000"

### set_pin_mode()

- **Parameters**

  pin_no (int):
  pin_mode (int): 0 - input, 1 - output, 2 - input_pullup

### set_digital_output()

- **Parameters**

  pin_no (int):

  pin_signal (int): 0 / 1

### get_digital_input()

- **Description**

- **Parameters**

### set_pwm_mode()

- **Description**

- **Parameters**

### set_pwm_output()

- **Description**

- **Parameters**

### get_gripper_value()

- **Description**

  Get gripper value

### set_gripper_state()

- **Description**

  Set gripper switch

- **Parameters**

  flag (`int`): 0 - open, 1 - close

  speed (`int`): 0 ~ 100

### set_gripper_value()

- **Description**

  Set gripper value

- **Parameters**

  value (int): 0 ~ 496

  speed (int): 0 ~ 100

### set_gripper_ini()

- **Description**

  Set the current position to zero

  Current position value is `248`.

### is_gripper_moving()

- **Description**

  Judge whether the gripper is moving or not

- Returns

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
