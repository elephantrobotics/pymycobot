# Introduction to API

API or Application Programming Interface refers to a number of preset programs. Before utilization, it is required to import API library:

```python
# for myBuddy
from pymycobot.mybuddy import MyBuddy
```

> **Notice:** Functions with return value are required to use `print()` to print value. For example, if you want to get the speed value, type `print(get_speed())`, instead of `get_speed()`.

## myBuddy

### 1 Overall Status

**1.1** `power_off(id=0)`

- **Fuction:** Close communication with Atom.

- **Parameters:**

  **id** – 0/1/2/3 (ALL/L/R/W)

**1.2** ` power_on(id=0)`

- **Function：** Open comminication with Atom.

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

**1.3** ` read_next_error(id=0)`

- **Function：** Robot Error Detection

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

**1.4** ` release_all_servos(id=0)`

- **Function：** Robot turns off torque output

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

**1.5** ` is_power_on(id=0)`

- **Function：** Adjust robot arm status

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

- **Return Value：**

  1 - power on
  0 - power off
  -1 - error data

**1.6** ` is_controller_connected(id=0)`

- **Function：** Wether connected with Atom.

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

- **Return Value：**

  0 - Not connected
  1 - Connected

**1.7** ` set_free_mode(id, value)`

- **Function：** set free mode

- **Parameter**

  - **id** – 0/1/2/3 (ALL/L/R/W)

  - **value** - 0 - close 1 - open

**1.8** ` set_fresh_mode(id, mode)`

- **Function：** set command refresh mode

- **Parameter**

  - **id** – 1/2（L/R）.

  - **mode** - int
    0 - Always execute the latest command first.
    1 - Execute instructions sequentially in the form of a queue.

**1.9** `release_servo（id，servo_id）`

- **Function：** Power off designated servo

- **Parameter**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** - 1 - 6.

**1.10** ` is_free_mode(id)`

- **Function：** check if it is free mode

- **Parameter**

  **id** – 0/1/2/3 (ALL/L/R/W)

- **Return Value：**

  0 - No
  1 - Yes

### 2 Operating Mode

**2.1** ` stop（id）`

- **Function：** Stop moving

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W).

**2.2** ` resume（id）`

- **Function：** Recovery movement

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W).

**2.3** ` is_paused(id)`

- **Function：** Judge whether the manipulator pauses or not.

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W).

- **Return Value：**

  1 - paused
  0 - not paused
  -1 - error

**2.4** ` get_speed(id)`

- **Function：** get speed

- **Parameters：**

  **id** – 1/2/3（L/R/W）.

- **Return Value：**

  speed

- **Return Value：Type**

  int

**2.5** ` set_speed（id，speed)`

- **Function：** set speed value

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **speed** (_int_) - 0 - 100

**2.6** ` get_joint_min_angle（id，joint_id）`

- **Function：** Gets the minimum movement angle of the specified joint

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** - (int) 1 - 6

- **Return Value：**

  angle value(float)

**2.7** ` is_servo_enable（id，servo_id）`

- **Function：** Determine whether all steering gears are connected

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** - (int) 1 ~ 6

- **Return Value：**

  0 - disable
  1 - enable
  -1 - error

**2.8** ` is_all_servo_enable(id)`

- **Function：** Determine whether the specified steering gear is connected

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

- **Return Value：**

  0 - disable
  1 - enable
  -1 - error

**2.9** ` set_joint_min（id，joint_id，angle)`

- **Function：** Set the joint minimum angle

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** - int 1-6.

  - **angle** - 0 ~ 180


**2.10** ` get_system_version(id)`

- **Function：** get system version

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W)

**2.11** ` get_joint_max_angle（id，joint_id`

- **Function:** Gets the maximum movement angle of the specified joint

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** - (int) 1 - 6

- **Return Value：**

  angle(float)


**2.12** ` joint_brake(id, joint_id)`

- **Function：** Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** - 1 - 6

### 3 MDI Mode

**3.1** ` get_angles(id)`

- **Function：** Get the degree of all joints.

- **Parameters**

  **id** – 1/2（L/R）

- **Return Value：**

  A float list of all degree.

- **Return Value：**

  list

**3.2** ` send_angle(id, joint, angle, speed)`

- **Function：** Send one degree of joint to robot arm.

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint** – 1 ~ 6

  - **angle** - int

  - **speed** – 1 ~ 100

- **Return Value：**
  - None

**3.3** ` send_angles（id，degrees，speed)`

- **Function：** Send all angles to the robotic arm

- **Parameters**

  - **id** – 1/2（L/R）.

  - **degrees** - [angle_list] len 6

  - **speed** - 1 - 100

**3.4** ` set_joint_max（id，joint_id，angle)`

- **Function：** set the joint maximum angle

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **joint_id** - int 1-6.

  - **角度** - 0 ~ 180

**3.5** ` send_coord(id, coord, data, speed)`

- **Function：** Send a single coordinate to the robotic arm

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **coord** – 1 ~ 6 (x/y/z/rx/ry/rz)

  - **data** - int

  - **speed** - 0 ~ 100

**3.6** ` send_coords(id, coords, speed, mode)`

- **Function：** Send all coordinates to robotic arm

- **Parameters**

  - **id** – 1/2（L/R）.

  - **coords** – a list of coords（List[float]），length 6，[x(mm), y, z, rx(angle), ry, rz]

  - **speed** - (int) 1 ~ 100

  - **mode** - (int) 0 - moveJ, 1 - moveL, 2 - moveC

**3.7** ` get_coord（id，joint_id）`

- **Function：** Get the coordinates of the robotic arm

- **Parameters**

  - **id** (_int_) – 1/2/3 (L/R/W).

  - **joint_id** (_int_) – 1 - 7（7 is gripper）

**3.8** ` get_encoder(id,joint_id)`

- **Function：** Obtain the specified joint potential value.

- **Parameters**

  - **id** - 1/2/3（L/R/W）.

  - **joint_id** - (int) 1 ~ 6

- **Return Value：**

  0 ~ 4096

**3.9** ` get_encoders(id)`

- **Function：** Get the six joints of the manipulator

- **Parameters**

  **id** – 1/2（L/R）.

- **Return Value：**

  list

**3.10** ` get_radians(id)`

- **Function：** Get the radians of all joints

- **Parameters**

  **id** – 1/2（L/R）

- **Return Value：**

  A list of float radians [radian1, ...]

- **Return Value：**
  list

**3.11** ` send_radians(id, radians, speed)`

- **Function：** Send the radians of all joints to robot arm
- **Parameters**

  - **id** – 1/2（L/R）.

  - **radians** – a list of radian values（List[float]），length 6

  - **speed** - (int)0 ~ 100

**3.12** ` is_in_position(id, data, mode)`

- **Function：** Detect whether in the position.

- **Parameters**

  - **id** – 0/1/2/3 (ALL/L/R/W).

  - **data** – A data list, angles or coords. If id is 1/2. data length is 6. If id is 0. data len 13. if id is 3. data len 1

  - **mode** - 1 - coords, 0 - angles

- **Return Value：**

  1 - True
  0 - False
  -1 - error

**3.13** ` is_moving(id)`

- **Function：** Detect if the robot is moving

- **Parameters**

  **id** – 0/1/2/3 (ALL/L/R/W).

- **Return Value：**

  0 - not moving
  1 - is moving
  -1 - error data

**3.14** ` set_color(id, r=0, g=0, b=0)`

- **Function：** Set the light color on the top of the robot arm.

- **Parameters**

  - **id** - 1/2（L/R）

  - **r** (_int_) – 0 ~ 255

  - **g** (_int_) – 0 ~ 255

  - **b** (_int_) – 0 ~ 255

**3.15** ` set_encoder（id，joint_id，encoder，speed)`

- **Function：** Set a single joint rotation to the specified potential value.

- **Parameters**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** - 1 - 6.

  - **encoder** – The value of the set encoder.

**3.16** ` set_encoders（id，encoder，speed)`

- **Function：** Set the six joints of the manipulator to execute synchronously to the specified position.

- **Parameters**

  - **id** – 1/2（L/R）.

  - **encoders** - A encoder list, length 6.

  - **speed** – speed 1 ~ 100

**3.17** ` get_angle（id，joint_id）`

- **Function：** Get the angle of a single joint

- **Parameters**

  - **id** (_int_) – 1/2/3 (L/R/W).

  - **joint_id** (_int_) – 1 - 7（7 is gripper）

**3.18** ` set_servo_calibration（id，servo_no`

- **Function：** The current position of the calibration joint actuator is the angle zero point,

  and the corresponding potential value is 2048.

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

**3.19** ` set_joint_current（id，joint_id，current）`

- **Function：** Set Collision Current

- **Parameters:**

  - **id** - 0/1/2 (ALL/L/R)

  - **joint_id** - 1 - 6

  - **current** – current value

**3.19** ` get_coords(id)`

- **Function：** Read a single coordinate parameter

- **Parameters**

  **id** – 1/2（L/R）

### 4 JOG Mode

**4.1** ` jog_absolute（id，joint_id，angle，speed）`

- **Function：** Absolute joint control

- **Parameters:**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** - int 1-6.

  - **angle** - int

  - **speed** - int (0 - 100)

**4.2** ` jog_angle（id，joint_id，direction，speed）`

- **Function：** Jog control joint

- **Parameters:**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** - int 1-6.

  - **direction** - 0 - decrease，1 - increase

  - **speed** - int (0 - 100)

**4.3** ` jog_coord(id, coord_id, direction, speed)`

- **Function：** Jog control coordinate

- **Parameters:**

  - **id** – 1/2/3 (L/R/W).

  - **coord_id** – int 1-6 (x/y/z/rx/ry/rz).

  - **direction** - 0 - decrease，1 - increase

  - **speed** - int (0 - 100)

**4.4** ` jog_inc_coord（axis，increment，speed）`

- **Function：** Double-arm coordinated coordinate stepping

- **Parameters:**

  - **axis** – 1 - 6 (x/y/z/rx/ry/rz)

  - **increment** -

  - **speed** - 1 - 100

**4.5** ` jog_increment（id，joint_id，increment，speed）`

- **Function：** step mode

- **Parameters:**

  - **id** – 1/2/3 (L/R/W).

  - **joint_id** - int 1-6.

  - **increment** -

  - **speed** - int (1 - 100)

**4.6** ` jog_stop(id)`

- **Function：** JOG stop

- **Parameters:**

  **id** – 1/2/3（L/R/W）

### 5 Servo COntrol

**5.1** ` focus_servo（id，servo_id）`

- **Function：** Power on designated servo

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **servo_id** - 1 - 6

**5.2** ` get_servo_currents(id)`

- **Function：** Get joint current

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

- **Return Value：**

  value mA

<!-- **5.3** ` get_servo_data（id，servo_no，data_id）`

- **Function：** Read the data parameter of the specified address of the steering gear.

- **Parameters：**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

  - **data_id** – Data address.

- **Return Value：**

  values (0 - 4096)
  0 - disable
  1 - enable
  -1 - error -->

**5.3** ` get_servo_status(id)`

- **Function：** Get joint status

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

- **Return Value：**

  [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error

**5.4** ` get_servo_temps(id)`

- **Function：** Get joint temperature

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

**5.5** ` get_servo_voltages(id)`

- **Function：** Get joint voltages

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

- **Return Value：**

  volts < 24 V

<!-- **5.7** ` set_servo_data（id，servo_no，data_id，value）`

- **Function：**Set the data parameters of the specified address of the steering gear

- **Parameters**

  - **id** – 1/2/3 (L/R/W)

  - **servo_no** – Serial number of articulated steering gear, 1 - 6.

  - **data_id** – Data address.

  - **value** – 0 - 4096 -->

### 6 Atom IO Control

**6.1** ` set_pin_mode(id, pin_no, pin_mode)`

- **Function：** Set the state mode of the specified pin in atom.

- **Parameters：**

  - **id** - 1/2（L/R）

  - **pin_no** (_int_) – pin number (1 - 5).

  - **pin_mode** (_int_) – 0 - input, 1 - output

**6.2** ` set_digital_output(id, pin_no, pin_signal)`

- **Function：** Set atom IO output level

- **Parameters：**

  - **id** - 1/2（L/R）

  - **pin_no** (_int_) - 1 - 5

  - **pin_signal** (_int_) – 0 / 1

**6.3** ` get_digital_input(id, pin_no)`

- **Function：** signal

- **Parameters：**

  - **id** - 1/2（L/R）

  - **pin_no** (_int_) - 1 - 5

**6.4** ` set_pwm_output(id, channel, frequency, **6.1pin_val)`

- **Function：** PWM control

- **Parameters：**

  - **id** - 1/2（L/R）

  - **channel** (_int_) – IO number (1 - 5).

  - **frequency** (_int_) – clock frequency (0/1: 0 - 1Mhz 1 - 10Mhz)

  - **pin_val** (_int_) – Duty cycle 0 ~ 100: 0 ~ 100%

### 7 Gripper Control

**7.1** ` get_gripper_value(id)`

- **Function：** Get the value of gripper.

- **Parameters**

  **id** – 1/2（L/R）

- **Return Value：**

  gripper value (int)

**7.2** ` is_gripper_moving(id)`

- **Function：** Judge whether the gripper is moving or not

- **Parameters**

  **id** – 1/2（L/R）

- **Return Value：**

  0 - not moving
  1 - is moving
  -1 - error data

**7.4** ` set_gripper_state(id, flag)`

- **Function：** Set gripper switch state

- **Parameters**

  - **id** - 1/2（L/R）

  - **flag** (_int_) - 0 - close，1 - open

**7.5** ` set_gripper_value(id, value, speed)`

- **Function：** Set gripper value

* **Parameters**

  - **id** – 1/2 (L/R)

  - **value** (_int_) – 0 ~ 100

  - **speed** (_int_) – 0 ~ 100

### 8 Socket Control

```python
from pymycobot import MyBuddySocket

mst = MyBuddySocket("192.168.0.1", 9000)
mst.connect("/dev/ttyACM0", "115200")

print(mst.get_angles(1))
```

### 9 Raspberry PI-GPIO

**9.1** ` set_gpio_input(pin)`

- **Function：** Set GPIO input value.

- **Parameters**

  **pin** - (int)pin number.

**9.2** ` set_gpio_mode(pin_no, mode)`

- **Function：** Init GPIO module, and set BCM mode.

- **Parameters**

  - **pin_no** – (int)pin number.

  - **mode** – 0 - input 1 - output

**9.3** ` set_gpio_output(pin, v)`

- **Function：** Set GPIO output value.

- **Parameters**

  - **pin** - (int)pin number.

  - **v** - (int) 0 / 1

**9.4** ` set_gpio_pwm（pin, baud, dc）`

- **Function：** Set GPIO PWM value.

- **Parameters**

  - **pin** – (int)pin number.

  - **baud** – (int) 10 - 1000000

  - **dc** – (int) 0 - 100

### 10 Coordinate Transformation

**10.1** ` set_tool_reference（id，coords）`

- **Function：** Set tool coordinate system

- **Parameters**

  - **id** - 0/1/2 (ALL/L/R)

  - **coords** – a list of coords value(List[float]), length 6. [x(mm), y, z, rx(angle), ry, rz]

**10.2** ` set_world_reference（id，coords）`

- **Function：** Set the world coordinate system

- **Parameters**

  - **id** - 0/1/2 (ALL/L/R)

  - **coords** – a list of coords value(List[float]), length 6 [x(mm), y, z, rx(angle), ry, rz]

**10.3** ` get_reference_frame(id)`

- **Function：** Get the base coordinate system

- **Parameters**

  **id** – 0/1/2 (ALL/L/R)

- **Return Value：**

  0 - base 1 - tool

**10.4** ` get_tool_reference(id)`

- **Function：** Get tool coordinate system

- **Parameters**

  **id** – 0/1/2 (ALL/L/R)

**10.5** ` get_world_reference(id)`

- **Function：** Get the world coordinate system

- **Parameters**

  **id** – 0/1/2 (ALL/L/R)

**10.6** ` set_reference_frame(id, rftype)`

- **Function：** Set the base coordinate system

- **Parameters**

  - **id** - 0/1/2 (ALL/L/R)

  - **rftype** - 0 - base 1 - tool.

**10.7** ` set_movement_type(id, move_type)`

- **Function：** Set movement type

- **Parameters**

  - **id** - 0/1/2 (ALL/L/R)

  - **move_type** - 1 - movel，0 - moveJ

**10.8** ` get_movement_type(id)`

- **Function：** Get movement type

- **Parameters**

  **id** – 0/1/2 (ALL/L/R)

- **Return Value：**

  1 - movel
  0 - moveJ

**10.9** ` set_end_type(id, end)`

- **Function：** Set end coordinate system

- **Parameters**

  - **id** - 0/1/2 (ALL/L/R)

  - **end** – 0 - flange, 1 - tool

**10.10** ` get_end_type(id)`

- **Function：** Get end coordinate system

- **Parameters**

  **id** – 0/1/2 (ALL/L/R)

- **Return Value：**

  0 - flange
  1 - tool

**10.11** ` write_base_coords(id, coords, speed)`

- **Function：** Base coordinate move

- **Parameters**

  - **id** - 1/2（L/R）

  - **coords** – coords: a list of coords value(List[float]), length 6, [x(mm), y, z, rx(angle), ry, rz]

  - **speed** - 1 - 100

**10.12** ` write_base_coord（id，axis, coord, speed)`

- **Function：** Base single coordinate movement

- **Parameters**

  - **id** - 1/2（L/R）

  - **axis** – 1 - 6 (x/y/z/rx/ry/rz)

  - **coord** - Coordinate value

  - **speed** - 1 - 100

**10.13** ` base_to_single_coords(base_coords, arm)`

- **Function：** Convert base coordinates to coordinates

- **Parameters：**

  - **coords** – a list of base coords value len 6

  - **arm** – 0 - left. 1 - right

- **Return Value：:**

  coords

**10.14** ` get_base_coord(id)`

- **Function：** Get the base coordinates of the single arm

- **Parameters：**

  **id** – 1/2（L/R）

**10.15** `get_base_coords(\*args: int)`

- **Function：** Convert coordinates to base coordinates. Pass in parameters or no parameters

- **Parameters：**

  - **coords** – a list of coords value(List[float]), length 6 [x(mm), y, z, rx(angle), ry, rz]

  - **arm** - 0 - L. 1 -

- **Return Value：:**

  Base coords

### 11 Speed Planning

**11.1** ` get_plan_acceleration(id=0)`

- **Function：** Get planning acceleration

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W)

- **Return Value：**

  [movel planning acceleration, movej planning acceleration].

**11.2** ` get_plan_speed(id=0)`

- **Function：** Get planning speed

- **Parameters：**

  **id** – 0/1/2/3 (ALL/L/R/W)

- **Return Value：**

  [movel planning speed, movej planning speed].

**11.3** ` set_acceleration(id, acc)`

- **Function：**Set acceleration during all moves

- **Parameters:**

  - **id** – 1/2/3 (L/R/W)

  - **acc** - 1 - 100

**11.4** ` get_acceleration(id)`

- **Function：**Read acceleration during all moves

- **Parameters：**

  **id** – 1/2/3 (L/R/W)

### 12 Collision Detection

**12.1** ` get_joint_current（id，joint_id）`

- **Function:** Get Collision Current

- **Parameters:**

  - **id** - 0/1/2 (ALL/L/R)

  - **joint_id** - 1 - 6

**12.2** ` collision_switch（state）`

- **Function:** Collision Detection Switch

- **Parameters:**

  **state** (_int_) – 0 - close 1 - open (Off by default)

**12.3** ` collision（left_angles，right_angles）`

- **Function:** Collision detection main program

- **Parameters:**

  - **left_angles** – left arm angle len 6.

  - **right_angles** – right arm angle len 6.

- **Returns**

  int

**12.4** ` is_collision_on()`

- **Parameters：** Get collision detection status
- **Return Value：：**

  0 - disbale
  1 - enable
