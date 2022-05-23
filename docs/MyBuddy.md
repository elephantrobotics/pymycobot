# pymycobot package

MyBuddy Python API

<details>
<summary>Catalogue:</summary>

- [get_robot_version](#get_robot_version)
- [power_on](#power_on)

</details>

### focus_servo(id, servo_id)

Power on designated servo

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_id** – 1 - 6

### get_acceleration(id)

Read acceleration during all moves

* **Parameters**

    **id** – 1/2/3 (L/R/W)

### get_angles(id)

Get the degree of all joints.

* **Parameters**

    **id** – 1/2/3 (L/R/W)

* **Returns**

    A float list of all degree.

* **Return type**

    list

### get_basic_input(pin_no)

Get basic input for M5 version.

* **Parameters**

    **pin_no** – (int) pin port number (0 - 20).

### get_coords(id)

* **Parameters**

    **id** – 1/2/3 (L/R/W).

### get_digital_input(id, pin_no)

singal value

* **Parameters**

  * **id** – 1/2 (L/R)

  * **pin_no** (_int_) – 1 - 5

### get_encoder(id, joint_id)

Obtain the specified joint potential value.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **joint_id** – (int) 1 ~ 6

* **Returns**

    0 ~ 4096

### get_encoders(id)

Get the six joints of the manipulator

* **Parameters**

    **id** – 1/2/3 (L/R/W).

* **Returns**

    The list of encoders

### get_end_type(id)

Get end coordinate system

* **Parameters**

    **id** – 0/1/2 (ALL/L/R)

* **Returns**

    0 - flange
    1 - tool

### get_gripper_value(id)

Get the value of gripper.

* **Parameters**

    **id** – 1/2 (L/R)

* **Returns**

    gripper value (int)

### get_joint_current(id, joint_id)

Get Collision Current

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **joint_id** – 1 - 6

### get_joint_max_angle(id, joint_id)

Gets the maximum movement angle of the specified joint

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint_id** – (int) 1 - 6

* **Returns**

    angle value(float)

### get_joint_min_angle(id, joint_id)

Gets the minimum movement angle of the specified joint

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint_id** – (int) 1 - 6

* **Returns**

    angle value(float)

### get_movement_type(id)

Get movement type

* **Parameters**

    **id** – 0/1/2 (ALL/L/R)

* **Returns**

    1 - movel
    0 - moveJ

### get_plan_acceleration()

Get planning acceleration

* **Returns**

    [movel planning acceleration, movej planning acceleration].

### get_plan_speed()

Get planning speed

* **Returns**

    [movel planning speed, movej planning speed].

### get_reference_frame(id)

Get the base coordinate system

* **Parameters**

    **id** – 0/1/2 (ALL/L/R)

* **Returns**

    0 - base 1 - tool.

### get_robot_id()

Detect this robot id

### get_robot_version()

Get cobot version

* **Returns**

    1
    mycobotPro: 101

* **Return type**

    mycobot

### get_servo_currents(id)

Get joint current

* **Parameters**

    **id** – 1/2/3 (L/R/W)

* **Returns**

    value mA

### get_servo_data(id, servo_no, data_id)

Read the data parameter of the specified address of the steering gear.

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_no** – Serial number of articulated steering gear, 1 - 6.

  * **data_id** – Data address.

* **Returns**

    0 - 4096
    0 - disable
    1 - enable
    -1 - error

* **Return type**

    values

### get_servo_status(id)

Get joint status

* **Parameters**

    **id** – 1/2/3 (L/R/W)

* **Returns**

    [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error

### get_servo_temps(id)

Get joint temperature

* **Parameters**

    **id** – 1/2/3 (L/R/W)

### get_servo_voltages(id)

Get joint voltages

* **Parameters**

    **id** – 1/2/3 (L/R/W)

* **Returns**

    volts < 24 V

### get_speed(id)

Get speed

* **Parameters**

    **id** – 1/2/3 (L/R/W).

* **Returns**

    (int)

* **Return type**

    speed

### get_system_version()

Get cobot version

* **Returns**

    1
    mycobotPro: 101

* **Return type**

    mycobot

### get_tool_reference(id)

Get tool coordinate system

* **Parameters**

    **id** – 0/1/2 (ALL/L/R)

### get_world_reference(id)

Get the world coordinate system

* **Parameters**

    **id** – 0/1/2 (ALL/L/R)

### is_all_servo_enable(id)

Determine whether the specified steering gear is connected

* **Parameters**

    **id** – 1/2/3 (L/R/W)

* **Returns**

    0 - disable
    1 - enable
    -1 - error

### is_controller_connected()

Wether connected with Atom.

### is_free_mode(id)

Check if it is free mode

* **Parameters**

    **id** – 0/1/2/3 (ALL/L/R/W)

* **Returns**

    0/1

### is_gripper_moving(id)

Judge whether the gripper is moving or not

* **Parameters**

    **id** – 1/2 (L/R)

* **Returns**

    not moving
    1 : is moving
    -1: error data

* **Return type**

    0

### is_in_position(id, mode, data)

Judge whether in the position.

* **Parameters**

  * **id** – ALL/L/R/W (0/1/2/3).

  * **mode** – 1 - coords, 0 - angles

  * **data** – A data list, angles or coords, length 6.

* **Returns**

    True
    0 : False
    -1: Error

* **Return type**

    1

### is_moving(id)

* **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

* **Returns**

    not moving
    1 : is moving
    -1 : error data

* **Return type**

    0

### is_paused(id)

Judge whether the manipulator pauses or not.

* **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

* **Returns**

    1 - paused
    0 - not paused
    -1 - error

### is_power_on()

Adjust robot arm status

* **Returns**

    power on
    0 : power off
    -1: error data

* **Return type**

    1

### is_servo_enable(id, servo_id)

Determine whether all steering gears are connected

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_id** – (int) 1 ~ 6

* **Returns**

    0 - disable
    1 - enable
    -1 - error

### jog_absolute(id, joint_id, angle, speed)

Absolute joint control

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **joint_id** – int 1-6.

  * **angle** – int

  * **speed** – int (0 - 100)

### jog_angle(id, joint_id, direction, speed)

Jog control angle.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **joint_id** – int 1-6.

  * **direction** – 0 - decrease, 1 - increase

  * **speed** – int (0 - 100)

### jog_coord(id, coord_id, direction, speed)

Jog control coord.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **coord_id** – int 1-6 (x/y/z/rx/ry/rz).

  * **direction** – 0 - decrease, 1 - increase

  * **speed** – int (0 - 100)

### jog_increment(id, joint_id, increment, speed)

step mode

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **joint_id** – int 1-6.

  * **increment** – # TODO 未注明

  * **speed** – int (1 - 100)

### jog_stop(id)

Stop jog moving
:param id: 1/2/3 (L/R/W).

### joint_brake(id, joint_id)

Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint_id** – 1 - 6

### pause(id)

Pause movement

* **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

### power_off()

Close communication with Atom.

### power_on()

Open communication with Atom.

### release_all_servos()

### release_servo(id, servo_id)

Power off designated servo

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_id** – 1 - 6.

### resume(id)

Recovery movement

* **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).

### send_angle(id, joint, angle, speed)

Send one degree of joint to robot arm.

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint** – 1 ~ 6

  * **angle** – int

  * **speed** – 1 ~ 100

### send_angles(id, degrees, speed, mode)

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **degrees** – [angle_list]

  * **speed** – 1 - 100

  * **mode** – 0 - with interpolation 1 - No interpolation 2 - reserved.

### send_coord(id, coord, data, speed)

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **coord** – 1 ~ 6 (x/y/z/rx/ry/rz)

  * **data** – int

  * **speed** – 0 ~ 100

### send_coords(id, coords, speed, mode)

Send all coords to robot arm.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **coords** – a list of coords value(List[float]), length 6, [x(mm), y, z, rx(angle), ry, rz]

  * **speed** – (int) 0 ~ 100

  * **mode** – (int) 0 - moveJ, 1 - moveL, 2 - moveC

### set_acceleration(id, acc)

Read acceleration during all moves

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **acc** – 1 - 100

### set_basic_mode(pin_no, pin_mode)

Set base IO, input or output mode

* **Parameters**

  * **pin_no** – 1 - 5

  * **pin_mode** – 0 - input 1 - output

### set_basic_output(pin_no, pin_signal)

Set basic output.

* **Parameters**

  * **pin_no** – pin port number (0 - 20).

  * **pin_signal** – 0 / 1 (0 - low, 1 - high)

### set_color(r=0, g=0, b=0)

Set the light color on the top of the robot arm.

* **Parameters**

  * **r** (_int_) – 0 ~ 255

  * **g** (_int_) – 0 ~ 255

  * **b** (_int_) – 0 ~ 255

### set_digital_output(id, pin_no, pin_signal)

* **Parameters**

  * **id** – 1/2 (L/R)

  * **pin_no** (_int_) – 1 - 5

  * **pin_signal** (_int_) – 0 / 1

### set_encoder(id, joint_id, encoder, speed)

Set a single joint rotation to the specified potential value.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **joint_id** – 1 - 6.

  * **encoder** – The value of the set encoder.

### set_encoders(id, encoders, speed, mode)

Set the six joints of the manipulator to execute synchronously to the specified position.

* **Parameters**

  * **id** – 1/2/3 (L/R/W).

  * **encoders** – A encoder list, length 6.

  * **speed** – speed 1 ~ 100

  * **mode** – 0 - with interpolation 1 - No interpolation 2 - reserved.

### set_end_type(id, end)

Set end coordinate system

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **end** – 0 - flange, 1 - tool

### set_free_mode(id)

set free mode

* **Parameters**

    **id** – 0/1/2/3 (ALL/L/R/W)

### set_gripper_calibration(id)

Set the current position to zero, set current position value is 2048.
:param id: 1/2 (L/R)

### set_gripper_state(id, flag)

Set gripper switch state

* **Parameters**

  * **id** – 1/2 (L/R)

  * **flag** (_int_) – 0 - close, 1 - open

### set_gripper_value(id, value)

Set gripper value

* **Parameters**

  * **id** – 1/2 (L/R)

  * **value** (_int_) – 0 ~ 100

### set_joint_current(id, joint_id, current)

Set Collision Current

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **joint_id** – 1 - 6

  * **current** – current value

### set_joint_max(id, joint_id, angle)

Set the joint maximum angle

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint_id** – int 1-6.

  * **angle** – 0 ~ 180

### set_joint_min(id, joint_id, angle)

Set the joint minimum angle

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **joint_id** – int 1-6.

  * **angle** – 0 ~ 180

### set_movement_type(id, move_type)

Set movement type

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **move_type** – 1 - movel, 0 - moveJ

### set_pin_mode(id, pin_no, pin_mode)

Set the state mode of the specified pin in atom.

* **Parameters**

  * **id** – 1/2 (L/R)

  * **pin_no** (_int_) – pin number (1 - 5).

  * **pin_mode** (_int_) – 0 - input, 1 - output

### set_plan_acceleration(acceleration, is_linear)

Set planning acceleration

* **Parameters**

  * **acceleration** (_int_) – (0 ~ 100).

  * **is_linear** – 0 -> joint 1 -> straight line

### set_plan_speed(speed, is_linear)

Set planning speed

* **Parameters**

  * **speed** (_int_) – (0 ~ 100).

  * **is_linear** – 0 -> joint 1 -> straight line

### set_pwm_output(id, channel, frequency, pin_val)

PWM control

* **Parameters**

  * **id** – 1/2 (L/R)

  * **channel** (_int_) – IO number (1 - 5).

  * **frequency** (_int_) – clock frequency (0/1: 0 - 1Mhz 1 - 10Mhz)

  * **pin_val** (_int_) – Duty cycle 0 ~ 100: 0 ~ 100%

### set_reference_frame(id, rftype)

Set the base coordinate system

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **rftype** – 0 - base 1 - tool.

### set_robot_id(id)

Set this robot id

### set_servo_calibration(id, servo_no)

The current position of the calibration joint actuator is the angle zero point,

    and the corresponding potential value is 2048.

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_no** – Serial number of articulated steering gear, 1 - 6.

### set_servo_data(id, servo_no, data_id, value)

Set the data parameters of the specified address of the steering gear

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **servo_no** – Serial number of articulated steering gear, 1 - 6.

  * **data_id** – Data address.

  * **value** – 0 - 4096

### set_speed(id, speed)

Set speed value

* **Parameters**

  * **id** – 1/2/3 (L/R/W)

  * **speed** (_int_) – 0 - 100

### set_tool_reference(id, coords)

Set tool coordinate system

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **coords** – a list of coords value(List[float]), length 6.
    for mycobot :[x(mm), y, z, rx(angle), ry, rz]

### set_world_reference(id, coords)

Set the world coordinate system

* **Parameters**

  * **id** – 0/1/2 (ALL/L/R)

  * **coords** – a list of coords value(List[float]), length 6 [x(mm), y, z, rx(angle), ry, rz]

### stop(id)

Stop moving

* **Parameters**

    **id** – ALL/L/R/W (0/1/2/3).
