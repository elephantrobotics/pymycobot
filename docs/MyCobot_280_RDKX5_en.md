# MyCobot 280 X5 PI

[toc]

## Python API usage instructions

API (Application Programming Interface), also known as Application Programming Interface functions, are predefined functions. When using the following function interfaces, please import our API library at the beginning by entering the following code, otherwise it will not run successfully:

```python
# Example
from pymycobot import MyCobot280RDKX5

mc = MyCobot280RDKX5('/dev/ttyS1')

# Gets the current angle of all joints
angles = mc.get_angles()
print(angles)

# Set 1 joint to move to 40 and speed to 20
mc.send_angle(1, 40, 20)
```

### 1. System Status

#### `get_modify_version()`
- **function:** get modify version
#### `get_system_version()`
- **function:** get system version

### 2. Overall Status

#### `clear_queue()`
- **function:** Clear the command queue
#### `async_or_sync()`
- **function:** Set the command execution mode
- **Return value:**
  - **`0`: Asynchronous mode**
  - **`1`: Synchronous mode**
#### `get_error_information()`
- **function:** Obtaining robot error information
- **Return value:**
  - **`0`: No error message.**
  - **`1 ~ 6`: The corresponding joint exceeds the limit position.**
  - **`16 ~ 19`: Collision protection.**
  - **`32`: Kinematics inverse solution has no solution.**
  - **`33 ~ 34`: Linear motion has no adjacent solution.**
#### `clear_error_information()`
- **function:** Clear robot error message
#### `power_on()`
- **function:** Open communication with Atom.
#### `power_off()`
- **function:** Close communication with Atom.
#### `is_power_on()`
- **function:** Adjust robot arm status
- **Return value:**
  - **`1` - power on**
  - **`0` - power off**
  - **`-1` - error data**
#### `release_all_servos(data)`
- **function:** Relax all joints
- **Parameters:**
  - **data: `1` - Undamping (The default is damping)**
#### `read_next_error()`
- **function:** Robot Error Detection
- **Return value:**
  - **list len 6.**
  - **`0` : No abnormality**
  - **`1` : Communication disconnected**
  - **`2` : Unstable communication**
  - **`3` : Servo abnormality**
#### `set_fresh_mode(mode)`
- **function:** Set command refresh mode
- **Parameters:**
  - **mode: int.**
  - **`1` - Always execute the latest command first.**
  - **`0` - Execute instructions sequentially in the form of a queue.**
#### `get_fresh_mode()`
- **function:** Query sports mode
#### `set_vision_mode(flag)`
- **function:** Set the visual tracking mode to limit the posture flipping of send_coords in refresh mode.
  - **(Only for visual tracking function)**
- **Parameters:**
  - **flag: 0/1; `0` - close; `1` - open**

### 3. Motion Control Interface

#### `get_angles()`
- **function:** Get the angle of all joints.
- **Return value:**
  - **list: A float list of all angle.**
#### `send_angle(id, degree, speed)`
- **function:** Send one angle of joint to robot arm.
- **Parameters:**
  - **id : Joint id(genre.Angle) int 1-6.**
  - **angle : angle value(float).**
  - **speed : (int) 1 ~ 100**
#### `send_angles(angles, speed)`
- **function:** Send the angles of all joints to robot arm.
- **Parameters:**
  - **angles: a list of angle values(List[float]). len 6.**
  - **speed : (int) 1 ~ 100**
#### `get_coords()`
- **function:** Get the coords from robot arm, coordinate system based on base.
- **Return value:**
  - **list : A float list of coord .[x, y, z, rx, ry, rz]**
#### `send_coord(id, coord, speed)`
- **function:** Send one coord to robot arm.
- **Parameters:**
  - **id(int) : coord id(genre.Coord) int 1-6.**
  - **coord(float) : coord value, mm**
  - **speed(int) : 1 ~ 100**
#### `send_coords(coords, speed, mode)`
- **function:** Send all coords to robot arm.
- **Parameters:**
  - **coords: a list of coords value(List[float]).[x(mm), y, z, rx(angle), ry, rz]**
  - **speed : (int) 1 ~ 100**
  - **mode : (int) 0 - angluar, 1 - linear**
#### `pause()`
- **function:** Pause movement
#### `is_paused()`
- **function:** Judge whether the manipulator pauses or not.
- **Return value:**
  - **`1` - paused**
  - **`0` - not paused**
  - **`-1` - error**
#### `resume()`
- **function:** Recovery movement
#### `stop()`
- **function:** Stop moving
#### `is_in_position(data, id = 0)`
- **function:** Judge whether in the position.
- **Parameters:**
  - **data: A data list, angles or coords.len 6.**
  - **id: 1 - coords, 0 - angles**
- **Return value:**
  - **`1` - True**
  - **`0` - False**
  - **`-1` - Error**
#### `is_moving()`
- **function:** Detect if the robot is moving
- **Return value:**
  - **`0` - not moving**
  - **`1` - is moving**
  - **`-1` - error data**
#### `write_angles_time_control(angles, step_time)`
- **function:** Write the angle of a joint in time control mode
- **Parameters:**
  - **angles: Angle value**
  - **step_time: Time unit: 30ms, range(1 ~ 255)**

### 4. JOG Mode And Operation

#### `jog_angle(joint_id, direction, speed)`
- **function:** Jog control angle.
- **Parameters:**
  - **joint_id: int 1-6.**
  - **direction: `0` - decrease, `1` - increase**
  - **speed: int (0 - 100)**
#### `jog_rpy(end_direction, direction, speed)`
- **function:** Rotate the end around a fixed axis in the base coordinate system
- **Parameters:**
  - **end_direction (int):  Roll, Pitch, Yaw (1-3)**
  - **direction (int): `1` - forward rotation, `0` - reverse rotation**
  - **speed (int): 1 ~ 100**
#### `jog_coord(coord_id, direction, speed)`
- **function:** Jog control coord.
- **Parameters:**
  - **coord_id: int 1-6**
  - **direction: `0` - decrease, `1` - increase**
  - **speed: int (1 - 100)**
#### `jog_increment_angle(joint_id, increment, speed)`
- **function:** angle step mode
- **Parameters:**
  - **joint_id: int 1-6.**
  - **increment: Angle increment value**
  - **speed: int (0 - 100)**
#### `jog_increment_coord(axis, increment, speed)`
- **function:** coord step mode
- **Parameters:**
  - **axis: axis id 1 - 6.**
  - **increment: Coord increment value**
  - **speed: int (1 - 100)**
#### `set_HTS_gripper_torque(torque)`
- **function:** Set new adaptive gripper torque
- **Parameters:**
  - **torque (int): 150 ~ 980**
- **Return value:**
  - **0: Set failed**
  - **1: Set successful**
#### `get_HTS_gripper_torque()`
- **function:** Get gripper torque
- **Return value:**
  - **int: 150 ~ 980**
#### `get_gripper_protect_current()`
- **function:** Get the gripper protection current
- **Return value:**
  - **int: 1 ~ 500**
#### `init_gripper()`
- **function:** Initialize gripper
- **Return value:**
  - **int: 0 or 1 (1 - success)**
#### `set_gripper_protect_current(current)`
- **function:** Set the gripper protection current
- **Parameters:**
  - **current (int): 1 ~ 500**
#### `set_encoder(joint_id, encoder, speed)`
- **function:** Set a single joint rotation to the specified potential value.
- **Parameters:**
  - **joint_id: int  1 - 6**
  - **for gripper: Joint id 7**
  - **encoder: The value of the set encoder.**
  - **speed : 1 - 100**
#### `get_encoder(joint_id)`
- **function:** Obtain the specified joint potential value.
- **Parameters:**
  - **joint_id: (int) 1 - 6**
  - **for gripper: Joint id 7**
#### `set_encoders(encoders, sp)`
- **function:** Set the six joints of the manipulator to execute synchronously to the specified position.
- **Parameters:**
  - **encoders: A encoder list. len 6.**
  - **sp: speed 1 ~ 100**
#### `get_encoders()`
- **function:** Get the six joints of the manipulator
- **Return value:**
  - **The list of encoders**
#### `set_encoders_drag(encoders, speeds)`
- **function:** Send all encoders and speeds
- **Parameters:**
  - **encoders: encoders list.**
  - **speeds: Obtained by the get_servo_speeds() method**
#### `get_joint_min_angle(joint_id)`
- **function:** Gets the minimum movement angle of the specified joint
- **Parameters:**
  - **joint_id: 1 - 6**
- **Return value:**
  - **angle value(float)**
#### `get_joint_max_angle(joint_id)`
- **function:** Gets the maximum movement angle of the specified joint
- **Parameters:**
  - **joint_id: 1 - 6**
- **Return value:**
  - **angle value(float)**
#### `set_joint_min(id, angle)`
- **function:** Set the joint minimum angle
- **Parameters:**
  - **id: int.**
  - **Joint id 1 - 6**
  - **for gripper: Joint id 7**
  - **angle: 0 ~ 180**
#### `set_joint_max(id, angle)`
- **function:** Set the joint maximum angle
- **Parameters:**
  - **id: int.**
  - **Joint id 1 - 6**
  - **for gripper: Joint id 7**
  - **angle: 0 ~ 180**

### 5. Servo Control

#### `is_servo_enable(servo_id)`
- **function:** To detect the connection state of a single joint
- **Parameters:**
  - **servo_id: 1 - 6**
- **Return value:**
  - **`0` - disable**
  - **`1` - enable**
  - **`-1` - error**
#### `is_all_servo_enable()`
- **function:** Detect the connection status of all joints
- **Return value:**
  - **`0` - disable**
  - **`1` - enable**
  - **`-1` - error**
#### `set_servo_data(servo_id, data_id, value, mode)`
- **function:** Set the data parameters of the specified address of the steering gear
- **Parameters:**
  - **servo_id: Serial number of articulated steering gear. 1 - 7**
  - **data_id: Data address.**
  - **value: 0 - 4096**
  - **mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.**
#### `get_servo_data(servo_id, data_id, mode)`
- **function:** Read the data parameter of the specified address of the steering gear.
- **Parameters:**
  - **servo_id: Serial number of articulated steering gear.1 - 7**
  - **data_id: Data address.**
  - **mode: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.**
- **Return value:**
  - **values 0 - 4096**
#### `set_servo_calibration(servo_id)`
- **function:** The current position of the calibration joint actuator is the angle zero point,
  - **and the corresponding potential value is 2048.**
- **Parameters:**
  - **servo_id: Serial number of articulated steering gear. 1 - 6**
#### `joint_brake(joint_id)`
- **function:** Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed
- **Parameters:**
  - **joint_id:  1 - 6**
#### `release_servo(servo_id, mode)`
- **function:** Power off designated servo
- **Parameters:**
  - **servo_id: int 1 - 6**
  - **mode: Default damping, set to 1, cancel damping**
#### `focus_servo(servo_id)`
- **function:** Power on designated servo
- **Parameters:**
  - **servo_id: int 1 - 6**

### 6. Gripper Control

#### `get_gripper_value(gripper_type)`
- **function:** Get the value of gripper.
- **Parameters:**
  - **gripper_type (int): default 1**
  - **`1`: Adaptive gripper**
  - **`3`: Parallel gripper**
  - **`4`: Flexible gripper**
- **Return value:**
  - **gripper value (int)**
#### `set_gripper_state(flag, speed, _type_1, is_torque)`
- **function:** Set gripper switch state
- **Parameters:**
  - **flag  (int): 0 - open, 1 - close, 254 - release**
  - **speed (int): 1 ~ 100**
  - **_type_1 (int): default 1**
  - **`1` : Adaptive gripper. default to adaptive gripper**
  - **`2` : 5 finger dexterous hand**
  - **`3` : Parallel gripper, this parameter can be omitted**
  - **`4` : Flexible gripper**
  - **is_torque (int): When there is no type parameter, this parameter can be omitted.**
  - **`1`: Force control**
  - **`0`: Non-force control**
#### `set_gripper_value(gripper_value, speed, gripper_type, is_torque)`
- **function:** Set gripper value
- **Parameters:**
  - **gripper_value (int): 0 ~ 100**
  - **speed (int): 1 ~ 100**
  - **gripper_type (int): default 1**
  - **`1`: Adaptive gripper**
  - **`3`: Parallel gripper, this parameter can be omitted**
  - **`4`: Flexible gripper**
  - **is_torque (int): When there is no type parameter, this parameter can be omitted.**
  - **`1`: Force control**
  - **`0`: Non-force control**
#### `set_gripper_calibration()`
- **function:** Set the current position to zero, set current position value is `2048`.
#### `is_gripper_moving()`
- **function:** Judge whether the gripper is moving or not
- **Return value:**
  - **`0` - not moving**
  - **`1` - is moving**
  - **`-1` - error data**

### 7. End ATOM Function

#### `get_tool_system_version()`
- **function:** Read the terminal primary and minor version numbers
#### `get_tool_modify_version()`
- **function:** Read the terminal modified version number
#### `is_tool_connected()`
- **function:** Check the end connection status
#### `set_color(r = 0, g = 0, b = 0)`
- **function:** Set the light color on the top of the robot arm.
- **Parameters:**
  - **r (int): 0 ~ 255**
  - **g (int): 0 ~ 255**
  - **b (int): 0 ~ 255**
#### `is_tool_button_click()`
- **function:** Check whether the button on the end is pressed
#### `set_digital_output(pin_no, pin_signal)`
- **function:** Set the terminal atom io status
- **Parameters:**
  - **pin_no     (int):**
  - **pin_signal (int): 0 / 1**
#### `get_digital_input(pin_no)`
- **function:** singal value

### 8. Kinematic Algorithm Interface

#### `solve_inv_kinematics(target_coords, current_angles)`
- **function:** Convert target coordinates to angles
- **Parameters:**
  - **target_coords: A float list of all coordinates.**
  - **current_angles : A float list of all angle.**
- **Return value:**
  - **list: A float list of all angle.**

### 9. Coordinate System Interface

#### `set_tool_reference(coords)`
- **function:** Set tool coordinate system
- **Parameters:**
  - **coords: a list of coords value(List[float])**
  - **[x(mm), y, z, rx(angle), ry, rz]**
#### `get_tool_reference()`
- **function:** Get tool coordinate system
#### `set_world_reference(coords)`
- **function:** Set the world coordinate system
- **Parameters:**
  - **coords: a list of coords value(List[float])**
  - **[x(mm), y, z, rx(angle), ry, rz]**
#### `get_world_reference()`
- **function:** Get the world coordinate system
#### `set_reference_frame(rftype)`
- **function:** Set the base coordinate system
- **Parameters:**
  - **rftype: 0 - base 1 - tool.**
#### `get_reference_frame()`
- **function:** Get the base coordinate system
- **Return value:**
  - **`0` - base `1` - tool.**
#### `set_movement_type(move_type)`
- **function:** Set movement type
- **Parameters:**
  - **move_type: `1` - movel, `0` - moveJ**
#### `get_movement_type()`
- **function:** Get movement type
- **Return value:**
  - **`1` - movel, `0` - moveJ**
#### `set_end_type(end)`
- **function:** Set end coordinate system
- **Parameters:**
  - **end: int**
  - **`0` - flange, `1` - tool**
#### `get_end_type()`
- **function:** Get end coordinate system
- **Return value:**
  - **`0` - flange, `1` - tool**

### 10. 9G Servo machine backgammon

#### `move_round()`
- **function:** Drive the 9g steering gear clockwise for one revolution
#### `set_four_pieces_zero()`
- **function:** Set the zero position of the four-piece motor
- **Return value:**
  - **int: `0` or `1` (1 - success)**

### 11. Stdio Interface

#### `get_angles_coords()`
- **function:** Get joint angles and coordinates
#### `get_quick_move_message()`
- **function:** Get the quick move message
#### `get_servo_speeds()`
- **function:** Get joint speed
- **Return value:**
  - **A list unit step/s**

### 12. Servo State Value Interface

#### `get_servo_currents()`
- **function:** Get all joint current
- **Return value:**
  - **A list unit mA**
#### `get_servo_voltages()`
- **function:** Get joint voltages
- **Return value:**
  - **A list volts < 24 V**
#### `get_servo_status()`
- **function:** Get joint status
- **Return value:**
  - **[voltage, sensor, temperature, current, angle, overload], a value of 0 means no error, a value of 1 indicates an error**
#### `get_servo_temps()`
- **function:** Get joint temperature
- **Return value:**
  - **A list unit ��**

### 13. Drag Track Interface

#### `drag_start_record()`
- **function:** Start track recording
- **Return value:**
  - **Recording queue length**
#### `drag_end_record()`
- **function:** End track recording
- **Return value:**
  - **Recording queue length**
#### `drag_get_record_data()`
- **function:** Get the recorded track
- **Return value:**
  - **List of potential values (encoder values) and operating speeds of each joint**
  - **eg: [J1_encoder, J1_run_speed,J2_encoder, J2_run_speed,J3_encoder, J3_run_speed,J4_encoder, J4_run_speed,J5_**
  - **encoder, J5_run_speed,J6_encoder, J6_run_speed]**
#### `drag_get_record_len()`
- **function:** Get the total number of recorded points
- **Return value:**
  - **Recording queue length**
#### `drag_clear_record_data()`
- **function:** Clear recording track
- **Return value:**
  - **Recording queue length 0**