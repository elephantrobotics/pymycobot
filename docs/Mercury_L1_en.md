# Mercury L1 Python API
[toc]

## Pre-use Preparation

Before using the Python API, please ensure that the following hardware and environmental prerequisites are met:

- **Hardware Equipment**
  - Mercury L1 Robotic Arm
  - Ethernet Cable (for connecting the robotic arm to the computer)
  - Power Adapter
  - Emergency Stop Switch (to ensure safe operation)

- **Software and Environment**
  - Python version 3.9 or higher installed
  - `pymycobot` library installed (install via the terminal command `pip install pymycobot`)
  - Ensure that the Mercury L1 is properly powered on and in standby mode
  - **Note:** The L1 server software starts automatically upon device power-up; no manual intervention is required.

- **Network Configuration**
  - Mercury L1 Default IP Address: `192.168.1.232`
  - Default Port Number: `6501`
  - **Note:** The PC's network interface IP address must be configured to be within the **same network segment** (e.g., `192.168.1.xxx`, where `xxx` is any number between 2 and 254, ensuring it does not conflict with the robotic arm's IP). 
  - Example:
    - Robotic Arm IP: `192.168.1.232`
    - PC IP: `192.168.1.100`
    - Subnet Mask: `255.255.255.0`
    - DNS Server: `114.114.114.114`

  - **Verification:** Once the network configuration is complete, execute the following command in the PC terminal; if data packets are successfully returned, the network connection is functioning correctly:  
  
    ```bash
    ping 192.168.1.232
    ```

---

## Introduction to Using the Arm API

An API (Application Programming Interface)—also referred to as an application programming interface function—consists of a set of pre-defined functions. When utilizing the following function interfaces, please ensure that you import our API library at the very beginning; this is accomplished by entering the code shown below. Failure to do so will prevent the program from running successfully:

**Note:** Before use, please ensure that the Mercury L1 server has been started and that the PC and the robotic arm are located within the same network segment.

```python
# Example
from pymycobot import MercuryL1Client

# The default IP address is "192.168.1.232", and the default port number is 6501.
mc = MercuryL1Client('192.168.1.232', 6501)

if mc.is_power_on() != 1:
mc.power_on(0)

print(mc.get_angles())
```

### 1. System Status

#### `get_system_version()`

- **Function:** Reads the machine's main controller version.
- **Return Value:** Main controller version number.

<!-- #### `get_modified_version()`

- **Function:** Reads the modification version number (for internal use only).
- **Return Value:** Modification version number. -->

#### `get_robot_type()`

- **Function:** Detects the machine model.

- **Return Value:** Definition rule: The actual machine model identifier. For example, the Mercury L1 model is `6501`.

#### `get_atom_version(arm_id)`

- **Function:** Reads the end-effector version number.
- **Parameters:** (`int`) `arm_id` — Arm ID:
  - `1`: Left arm
  - `2`: Right arm
- **Return Value:** Version number (`float`).

#### `get_tool_modify_version(arm_id)`

- **Function:** Reads the end-effector modification version number.
- **Parameters:** (`int`) `arm_id` — Arm ID:
  - `1`: Left arm
  - `2`: Right arm
- **Return Value:** Modification version number.

### 2. Robot Overall Operational Status

#### `power_on(arm_id)`

- **Function:** Starts the robot; powers it on.
- **Parameters:** (`int`) `arm_id` — Arm ID
  - `0`: Left arm and right arm
  - `1`: Left arm
  - `2`: Right arm
- **Return Value:**
  - `1` - Power-on successful.
  - `2` - Power-on failed.
  - `0` - Not powered on.

#### `power_off()`

- **Function:** Shuts down the robot; powers it off.

- **Return Value:**
  - `1` - Command successfully received.

#### `is_power_on()`

- **Function:** Checks whether the robotic arm is powered on.

- **Return Value:**
  - `1`: Powered on successfully.
  - `0`: Not powered on.
  - `2`: Power-on failed.

#### `is_init_calibration()`

- **Function:** Checks whether the robot's zero position has been set.

- **Return Value:** `bool` — Returns `True` if the robot has completed zero-position calibration initialization; otherwise, returns `False`.

#### `get_fresh_mode()`

- **Function:** Queries the motion mode.

- **Return Value:**
  - `0`: Interpolation Mode
  - `1`: Refresh Mode

#### `set_fresh_mode()`

- **Function:** Sets the refresh mode.

- **Parameters:**
  - `1`: Refresh Mode — Always executes the latest command first. 
  - `0`: Interpolation Mode — Executes commands sequentially in a queue.

#### `get_free_move_mode()`

- **Function:** Retrieves the free-move mode status.

- **Return Value:**
- `0`: Free-move mode is disabled.
- `1`: Free-move mode is enabled.

#### `set_free_move_mode(mode)`

- **Function:** Sets the free-move mode. (Note: Joints can only be relaxed by holding down the end-effector button when free-move mode is enabled.)

- **Parameters:**
- `1`: Enable free-move mode. 
- `0`: Disable free-move mode.

### 3. Robot Anomaly Detection

#### `get_robot_status()`

- **Function:** Retrieves the error and safety status for the left and right robot arms.
- **Return Value:** A `list` in the format `[[Left Arm Status], [Right Arm Status]]`, where `0` indicates a normal state. For example: `[[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]`. Others — Robot Anomalies:
  - **Left Arm**
    - `[Joint collision detected?, Currently in motion?, J1 limit exceeded?, J2 limit exceeded?, J3 limit exceeded?, J4 limit exceeded?, J5 limit exceeded?, J6 limit exceeded?, J7 limit exceeded?, J8 limit exceeded?, J1 motor hardware error?, J2 motor hardware error?, J3 motor hardware error?, J4 motor hardware error?, J5 motor hardware error?, J6 motor hardware error?, J7 motor hardware error?, J8 motor hardware error?, J1 software communication error?, J2 software communication error?, J3 software communication error?, J4 software communication error?, J5 software communication error?, J6 software communication error?, J7 software communication error?, J8 software communication error?]`
  - **Right Arm**
    - `[Joint collision detected, Currently in motion, J1 limit exceeded, J2 limit exceeded, J3 limit exceeded, J4 limit exceeded, J5 limit exceeded, J6 limit exceeded, J7 limit exceeded, J8 limit exceeded, J9 limit exceeded, J1 motor hardware error, J2 motor hardware error, J3 motor hardware error, J4 motor hardware error, J5 motor hardware error, J6 motor hardware error, J7 motor hardware error, J8 motor hardware error, J9 motor hardware error, J1 software communication error, J2 software communication error, J3 software communication error, J4 software communication error, J5 software communication error, J6 software communication error, J7 software communication error, J8 software communication error, J9 software communication error]`

#### `servo_restore(arm_id, joint_id)`

- **Function**: Clears joint exceptions.
- **Parameters**:
  - `arm_id`: `int`. Arm ID.
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: `int`. Joint ID (1–7); use `254` to restore all joints.

#### `get_comm_error_counts(arm_id, joint_id)`

- **Function**: Reads the communication error counts.
- **Parameters**:
  - `arm_id`: `int` — Arm ID
    - `0`: Left arm and right arm
    - `1`: Left arm
    - `2`: Right arm
  - `joint_id`: `int` — Joint ID (1 - 9)
- **Return Value**: `list` — A list of length 4 (e.g., `[0, 0, 0, 0]`), representing the following:
  - `[0]`: Joint transmission error count
  - `[1]`: Joint reception error count
  - `[2]`: End-effector transmission error count
  - `[3]`: End-effector reception error count

#### `get_error_information()`

- **Function**: Reads the robot's error information.
- **Return Value**: `list[int]` — `[Left Arm Status, Right Arm Status]`
  - `0`: No error information
  - `1~6`: The corresponding joint has exceeded its limit position.
  - `32~36`: Coordinate motion anomaly. 
  - `32`: No solution found for the coordinates; please check if the arm extension is approaching its limit.
  - `33`: No adjacent solution found for linear motion. - `34`: Velocity blending error
  - `35`: No adjacent solution found for null-space motion
  - `36`: No solution found at singular position; please use joint control to move away from the singularity
  - `81~86`: Joints J1 ~ J6 have triggered a collision; please use the `resume` interface to recover

#### `clear_error_information(arm_id)`

- **Function:** Clears the robot's error information
- **Parameters:** (`int`) `arm_id` — Arm ID
  - `0`: Left arm and right arm
  - `1`: Left arm
  - `2`: Right arm

#### `over_limit_return_zero()`

- **Function:** Command to return robot joints to the zero position following an over-limit error

#### `get_motors_run_err()`

- **Function:** Reads motor error information during robot motion
- **Return Value:** `list` — A list of length 6; if all elements are 0, it indicates normal operation

### 4. Robot Motion Control

#### `set_control_mode(mode)`

- **Function:** Sets the robot's motion mode.
- **Parameters:**
  - `mode`: `int`. 0 ~ 1 (inclusive); default is 0.
    - `0`: Position Mode
    - `1`: Torque Mode

#### `get_control_mode()`

- **Function:** Retrieves the robot's current motion mode.
- **Return Value:**
  - `0`: Position Mode
  - `1`: Torque Mode

#### `get_angles()`

- **Function:** Retrieves the angles of all joints.
- **Return Value:** `list`. A list of floating-point values ​​representing the angles of both the left and right arms, formatted as `[[left arm angles], [right arm angles]]`.

#### `get_angle(joint_id)`

- **Function:** Retrieves the angle of a single specific joint.
- **Parameters:**
- `joint_id`: `int`. The joint ID; range: 1 ~ 9.
- **Return Value:** `list[float]`. The angle for the specified joint across both arms, formatted as `[left angle, right angle]`.

#### `send_angle(arm_id, joint_id, speed, left_angle=None, right_angle=None, _async=False)`

- **Function:** Sends a target angle command to a specific joint on the robotic arm (when `arm_id` is set to 1 or 2, the value for the other arm is ignored).
- **Parameters:**
  - `arm_id`: (`int`) The arm ID.
    - `0`: Left and Right Arms. Requires both the `left_angle` and `right_angle` parameters to be provided. 
    - `1`: Left Arm. Only the `left_angle` parameter is required. 
    - `2`: Right Arm. Only the `right_angle` parameter is required. 
  - `joint_id`: Joint ID (int), range: 1–9
  - `speed`: (`int`) 1 ~ 100
  - `left_angle`: Angle value (`float`)

| Joint ID | Range |
| ---- | ---- |
| 1 | -181 ~ 135 |
| 2 | -46 ~ 96 |
| 3 | -155 ~ 155 |
| 4 | -135 ~ 18 |
| 5 | -155 ~ 155 |
| 6 | -115 ~ 115 |
| 7 | -155 ~ 155 |
| 8 (Waist) | 0 ~ 40 |

  - `right_angle`: Angle value (`float`)

| Joint ID | Range |
| ---- | ---- |
| 1 | -181 ~ 135 |
| 2 | -46 ~ 96 |
| 3 | -155 ~ 155 |
| 4 | -135 ~ 18 |
| 5 | -155 ~ 155 |
| 6 | -115 ~ 115 |
| 7 | -155 ~ 155 |
| 8 (Neck) | -50 ~ 50 |
| 9 (Head) | -82 ~ 82 |

  - `_async`: Motion closed-loop control switch; default: Enabled (False); Disabled (True).

#### `send_angles(arm_id, speed, left_angles=None, right_angles=None, _async=False)`

- **Function:** Sends angle commands to all joints of the robotic arm (when `arm_id` mode is 1 or 2, the values ​​for the other arm can be arbitrary).
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms; requires simultaneous input of both `left_angles` and `right_angles` parameters. 
    - `1`: Left Arm; requires input of only the `left_angles` parameter. 
    - `2`: Right Arm; requires input of only the `right_angles` parameter. 
  - `speed`: (`int`) 1 ~ 100
  - `left_angles`: List of angles in degrees (`List[float]`), length 8.
  - `right_angles`: List of angles in degrees (`List[float]`), length 9.
  - `_async`: Motion closed-loop switch; default is enabled (False); disabled (True).

#### `get_coords()`

- **Function:** Retrieves the robotic arm's coordinates relative to the base coordinate system.
- **Return Value:** A list of floating-point coordinates for the left and right arms: `[[x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz]]`.

#### `send_coord(arm_id, coord_id, speed, left_coord=None, right_coord=None, _async=False)`

- **Function:** Sends a coordinate command to the robotic arm (when `arm_id` mode is 1 or 2, the values ​​for the other arm can be arbitrary).
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms; requires simultaneous input of both `left_coord` and `right_coord` parameters. 
    - `1`: Left Arm; requires input of only the `left_coord` parameter. 
    - `2`: Right Arm; requires input of only the `right_coord` parameter. - `coord_id`: Sends a specific coordinate value to the robotic arm; IDs 1-6 correspond to [x, y, z, rx, ry, rz].
  - `speed` (`int`): 1 ~ 100
  - `left_coord`: Coordinate value (`float`)

| Coordinate ID | Range |
| ---- | ---- |
| x | -1000 ~ 1000 |
| y | -474 ~ 1000 |
| z | -1000 ~ 1000 |
| rx | -180 ~ 180 |
| ry | -180 ~ 180 |
| rz | -180 ~ 180 |

  - `right_coord`: Coordinate value (`float`); the range is the same as `left_coord`.
  - `_async`: Motion closed-loop switch; default is enabled (False); disabled (True).

#### `send_coords(arm_id, speed, left_coords=None, right_coords=None, _async=False)`

- **Function:** Sends a complete set of coordinates and pose data, moving the robotic arm's end-effector from the origin to a specified target point (when `arm_id` mode is 1 or 2, the values ​​for the other arm can be arbitrary).
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms; requires input for both `left_coords` and `right_coords` parameters.
    - `1`: Left Arm; requires input only for the `left_coords` parameter.
    - `2`: Right Arm; requires input only for the `right_coords` parameter. 
  - `speed` (`int`): 1 ~ 100
  - `left_coords`: Coordinate list; values ​​`[x, y, z, rx, ry, rz]`, length 6.
  - `right_coords`: Coordinate list; values ​​`[x, y, z, rx, ry, rz]`, length 6.
  - `_async`: Motion closed-loop switch; default is enabled (False); disabled (True). 

#### `pause(arm_id, deceleration=0)`

- **Function:** Controls the command core to pause and halt all motion commands.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `deceleration`: Whether to decelerate before stopping; defaults to 0. Both 0 and 1 indicate a gradual pause.
- **Return Value:**
  - `1` - Stopped
  - `0` - Not stopped
  - `-1` - Error

#### `is_paused()`

- **Function:** Checks whether the program has paused its motion commands.
- **Return Value:**
  - `1` - Paused
  - `0` - Not paused
  - `-1` - Error

#### `resume(arm_id)`

- **Function:** Resumes robot motion and completes any pending commands.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm

#### `stop(arm_id, deceleration=0)`

- **Function:** Halts robot motion.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `deceleration`: Whether to decelerate before stopping; defaults to 0. Both 0 and 1 indicate a gradual stop.
- **Return Value:**
  - `1` - Stopped
  - `0` - Not stopped
  - `-1` - Error

#### `is_in_position(data, flag)`

- **Function:** Determines whether the robot has reached a specific target position.
- **Parameters:**
  - `data`: A set of data points, which can be either joint angles or Cartesian coordinates. Assume the valid length range for input angles is 6, and the valid length range for input coordinate values ​​is 6.
  - `flag` Data Type (Value range: 0 or 1)
    - `0`: List of angle values
    - `1`: List of coordinate values
- **Return Value:**
  - `1` - true
  - `0` - false
  - `-1` - error

#### `is_moving()`

- **Function:** Detects whether the robot is currently in motion.
- **Return Value:**
  - `1`: Moving
  - `0`: Stopped
  - `-1`: Error

### 5. JOG Mode and Operations

#### `jog_angle(arm_id, joint_id, direction, speed)`

- **Function:** JOG control for joint angles; the specified joint moves continuously.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: The ID of the robotic arm joint; range: 1 to 9.
  - `direction` (`int`): Controls the direction of joint movement; input `0` for movement in the negative direction, input `1` for movement in the positive direction.
  - `speed`: 1 to 100.

#### `jog_coord(arm_id, coord_id, direction, speed)`

- **Function:** JOG control for coordinates; the specified coordinate moves continuously.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `coord_id`: (`int`) The coordinate axis of the robotic arm; range: 1 to 6.
  - `direction` (`int`): Controls the direction of movement along the axis; input `0` for movement in the negative direction, input `1` for movement in the positive direction.
  - `speed`: 1 to 100.

#### `jog_increment_angle(arm_id, joint_id, increment, speed)`

- **Function:** Incremental control for a single joint angle.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: 1 to 9.
  - `increment`: The incremental movement relative to the current joint angle position.
  - `speed`: 1 to 100.

#### `jog_increment_coord(arm_id, coord_id, increment, speed)`

- **Function:** Incremental control for a single coordinate axis.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `coord_id`: Coordinate axis 1 - 6.
  - `increment`: Incremental movement relative to the current position coordinates.
  - `speed`: 1 ~ 100

### 6. Speed/Acceleration Parameters

#### `get_max_speed(mode)`

- **Function:** Get the maximum movement speed.
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular speed
    - `1`: Cartesian speed
- **Return Value:** Angular speed range: 1–150°/s; Cartesian speed range: 1–200 mm/s.

#### `set_max_speed(arm_id, mode, max_speed)`

- **Function:** Set the maximum movement speed.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left Arm and Right Arm
    - `1`: Left Arm
    - `2`: Right Arm
  - `mode` : `int`
    - `0`: Angular speed
    - `1`: Cartesian speed
  - `max_speed`: Angular speed range: 1–150°/s; Cartesian speed range: 1–200 mm/s.

#### `get_max_acc(mode)`

- **Function:** Get the maximum movement acceleration.
- **Parameters:**
  - `mode` : `int`
    - `0`: Angular acceleration
    - `1`: Cartesian acceleration
- **Return Value:** Angular acceleration range: 1–150°/s; Cartesian acceleration range: 1–400 mm/s.

#### `set_max_acc(arm_id, mode, max_acc)`

- **Function:** Set the maximum movement acceleration.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left Arm and Right Arm
    - `1`: Left Arm
    - `2`: Right Arm
  - `mode` : `int`
    - `0`: Angular acceleration
    - `1`: Cartesian acceleration
  - `max_acc`: Angular acceleration range: 1–150°/s; Cartesian acceleration range: 1–400 mm/s.

### 7. Software Joint Limits

#### `get_joint_min_angle(joint_id)`

- **Function:** Retrieves the minimum movement angle for a specified joint.
- **Parameters:**
  - `joint_id`: Input Joint ID (Range: 1–9)
- **Return Value:** `float` (Angle value)

#### `get_joint_max_angle(joint_id)`

- **Function:** Retrieves the maximum movement angle for a specified joint.
- **Parameters:**
  - `joint_id`: Input Joint ID (Range: 1–9)
- **Return Value:** `float` (Angle value)

#### `set_joint_min_angle(arm_id, joint_id, angle)`

- **Function:** Sets the minimum angle limit for a joint.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: Input Joint ID (Range: 1–6)
  - `angle`: Refer to the limit information for the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface; the value must not be less than the minimum limit.

#### `set_joint_max_angle(arm_id, joint_id, angle)`

- **Function:** Sets the maximum angle limit for a joint.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: Input Joint ID (Range: 1–6)
  - `angle`: Refer to the limit information for the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface; the value must not be greater than the maximum limit.

### 8. Joint Motor Auxiliary Control

#### `get_servo_encoders()`

- **Function:** Reads the encoder values ​​for all joints.
- **Return Value:** A list of length 6.

#### `set_servo_calibration(arm_id, servo_id)`

- **Function:** Calibrates the current position of a joint actuator as the zero-degree reference point.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `servo_id`: 1 - 9

#### `set_break(joint_id, value)`

- **Function:** Sets the joint brake status.
- **Parameters:**
- `joint_id`: (`int`) Joint ID (1 - 6)
- `value`: (`int`) 0 - Disable; 1 - Enable
- **Return Value:** 0: Failure; 1: Success

#### `set_motor_enabled(arm_id, joint_id, state)`

- **Function:** Sets the robot's motor torque state (enables/disables joint control).
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id`: (`int`) Joint ID (1 - 9); 254 for all joints
  - `state`: (`int`) 0 - Disable; 1 - Enable

### 9. Drag Teaching

#### `drag_teach_save()`

- **Function:** Starts recording drag-teaching points.
  - Note: To ensure optimal motion performance, please do not exceed a recording duration of 120 seconds.

#### `drag_teach_pause()`

- **Function:** Pauses the drag-teaching recording.

#### `drag_teach_execute()`

- **Function:** Executes the recorded drag-teaching path (runs only once).

#### `drag_teach_clean()`

- **Function:** Clears the recorded teaching points.

#### `set_pro_gripper_offset(offset=2)`

- **Function:** Sets the offset for the Pro force-controlled gripper. When executing a drag-to-teach trajectory that involves the gripper, the actual gripper angle executed will be reduced by the specified `offset` (default is 2).
- **Parameters:** `int`
  - `offset`: Range -5 to 5; default is 2.

#### `get_pro_gripper_offset()`

- **Function:** Retrieves the offset for the Pro force-controlled gripper. When executing a drag-to-teach trajectory that involves the gripper, the actual gripper angle executed will be reduced by the specified `offset` (default is 2).
- **Return Value:** `int`, range -5 to 5.

### 10. Dynamics

#### `get_collision_mode()`

- **Function:** Queries the collision detection mode.
- **Return Value:**
  - `0`: Disabled
  - `1`: Enabled

#### `set_collision_mode(mode)`

- **Function:** Sets the joint collision detection mode.
- **Parameters:** `int`
  - `mode`:
    - `0`: Disabled
    - `1`: Enabled

#### `set_collision_threshold(joint_id, threshold_value=100)`

- **Function:** Sets the collision detection threshold for a specific joint.
- **Parameters:**
  - `joint_id`: `int` — Joint ID, range 1 to 6.
  - `threshold_value`: `int` — Collision threshold; range 50 to 250. Default value is 100. A lower value makes it easier to trigger a collision detection event.

#### `get_collision_threshold()`

- **Function:** Retrieves the collision thresholds for all joints.
- **Return Value:** A list containing the collision thresholds for all joints.

#### `set_torque_comp(joint_id, damping, comp_value=0)`

- **Function:** Sets the torque compensation coefficient.
- **Parameters:**
  - `joint_id`: `int` — Joint ID, range 1 to 6.
  - `damping`: `int` — Range 0 to 1. 1: Enabled; 0: Disabled.
  - `comp_value`: Compensation value; range: 0–250; default: 0. The lower the value, the more resistance is felt when manually dragging the joint.

#### `get_torque_comp()`

- **Function**: Retrieves the torque compensation coefficients.
- **Return Value**: A list containing the torque compensation coefficients for all joints.

### 11. Circular Motion

#### `write_move_c(transpoint, endpoint, speed)`

- **Function**: Executes circular trajectory motion (specifying a waypoint).
- **Parameters**:
  - `transpoint (list)`: The waypoint coordinates for the circular arc.
  - `endpoint (list)`: The endpoint coordinates for the circular arc.
  - `speed (int)`: 1 ~ 100.

### 12. Runtime Auxiliary Information

#### `get_zero_pos()`

- **Function**: Reads the zero-position encoder values.
- **Return Value**: A `list` containing the zero-position encoder values ​​for all 6 joints.

#### `get_servo_speeds()`

- **Function**: Retrieves the movement speeds for all joints.
- **Return Value**: A nested list in the format `[[Left Arm Speeds], [Right Arm Speeds]]`.

#### `get_servo_currents()`

- **Function**: Retrieves the current draw for each joint.
- **Return Value**: A nested list (values ​​range from 0 to 5000 mA) in the format `[[Left Arm Currents], [Right Arm Currents]]`.

#### `get_servo_status()`

- **Function**: Retrieves the operational status of all joints.
- **Return Value**: A nested list; a value of 0 indicates that no errors are present. Format: `[[Left Arm Status], [Right Arm Status]]`.

### 13. End-effector IO Control

#### `set_digital_output(pin_no, pin_signal)`

- **Function:** Sets the state of an end-effector IO pin.
- **Parameters:**
  - `pin_no` (int): Pin number; range: 1 to 2.
  - `pin_signal` (int): 0 / 1; 0 = Low level, 1 = High level.
- **Return Value:**
  - `1`: Success.

#### `get_digital_input(pin_no)`

- **Function:** Retrieves the state of an end-effector IO pin.
- **Parameters:** `pin_no` (int); range: 1 to 2.
- **Return Value:** `int` (0 / 1); 0 = Low level, 1 = High level.

#### `get_digital_inputs()`

- **Function:** Reads the states of all end-effector pins, including: IN1, IN2, Button 1 (right side), and Button 2 (Button 2 is located closer to the Emergency Stop button, on the left side).
- **Return Value:** `list[int]` (0 / 1); 0 = Low level, 1 = High level. e.g., `[0, 0, 1, 0]` indicates that Button 1 is pressed. ### 14. End-effector Light Board Functions

<!-- #### `is_btn_clicked()`

- **Function**: Get the status of the button at the end of the robot arm
- **Return Value**:
- 0: Not clicked
- 1: Clicked -->

### 15. Base IO Control

#### `set_base_io_output(pin_no, pin_signal)`

- **Function**: Sets the output state of the base IO pins.
- **Parameters**:
  - `pin_no` (`int`): Pin number, range: 1 to 12.
  - `pin_signal` (`int`): 0 - Low level; 1 - High level.

#### `get_base_io_output(pin_no)`

- **Function**: Gets the input state of the base IO pins.
- **Parameters**:
  - `pin_no` (`int`): Pin number, range: 1 to 12.
- **Return Value**: 0 - Low level; 1 - High level.

### 16. End-effector RS485 Communication Settings

#### `tool_serial_write_data(arm_id, command)`

- **Function**: Sends data via the end-effector's RS485 interface; data length ranges from 1 to 45 bytes.
- **Parameters**:
  - `arm_id` (`int`): Arm ID.
    - `1`: Left arm.
    - `2`: Right arm.
  - `command` (`list`): Data command in Modbus format.
- **Return Value**: A list containing the Modbus response data.

#### `flash_tool_firmware(arm_id, main_version, modified_version=0)`

- **Function**: Flashes the end-effector firmware.
- **Parameters**:
  - `arm_id` (`int`): Arm ID.
    - `1`: Left arm.
    - `2`: Right arm.
  - `main_version` (`str`): Major and minor version number (e.g., `'1.1'`).
  - `modified_version` (`int`): Revision version number, range: 0 to 255 (default is 0).

#### `set_tool_serial_baud_rate(arm_id,` baud_rate=115200)`

- **Function:** Sets the end-effector RS485 baud rate; default is 115200.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `baud_rate` (`int`): Standard baud rate; supports only 115200 and 1000000.
- **Return Value:** 1

#### `set_tool_serial_timeout(arm_id, timeout=10000)`

- **Function:** Sets the end-effector RS485 timeout duration; default is 10 seconds.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
- `timeout (int)`: Timeout duration (in milliseconds); range: 1 to 10000.
- **Return Value:** 1

#### `get_tool_config(arm_id)`

- **Function:** Retrieves the end-effector RS485 baud rate and timeout duration.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
- **Return Value:** (`list`) A list containing the baud rate and timeout duration; e.g., [baud_rate, timeout].

### 17. Tool Coordinate System Operations

#### `set_tool_reference(coords)`

- **Function:** Sets the tool coordinate system.
- **Parameters:**
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

| Coordinate ID | Range |
| ---- | ---- |
| x | -1000 ~ 1000 |
| y | -1000 ~ 1000 |
| z | -1000 ~ 1000 |
| rx | -180 ~ 180 |
| ry | -180 ~ 180 |
| rz | -180 ~ 180 |

#### `get_tool_reference(coords)`

- **Function:** Gets the tool coordinate system.
- **Return Value:** (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **Function:** Sets the world coordinate system.
- **Parameters:**
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

| Coordinate ID | Range |
| ---- | ---- |
| x | -1000 ~ 1000 |
| y | -1000 ~ 1000 |
| z | -1000 ~ 1000 |
| rx | -180 ~ 180 |
| ry | -180 ~ 180 |
| rz | -180 ~ 180 |

#### `get_world_reference()`

- **Function:** Gets the world coordinate system.
- **Return Value:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **Function:** Sets the base coordinate system.
- **Parameters:**
  - `rftype`: 0 - Base Coordinates (Default) | 1 - World Coordinates.

#### `get_reference_frame()`

- **Function:** Gets the base coordinate system.
- **Return Value:** (`list`) [x, y, z, rx, ry, rz].

#### `set_movement_type(move_type)`

- **Function:** Sets the movement type.
- **Parameters:**
  - `move_type`: 1 - moveL, 0 - moveJ.

#### `get_movement_type()`

- **Function:** Gets the movement type.
- **Return Value:**
  - `1` - moveL
  - `0` - moveJ

#### `set_end_type(end)`

- **Function:** Sets the end-effector coordinate system.
- **Parameters:**
  - `end (int)`: `0` - Flange (Default), `1` - Tool.

#### `get_end_type()`

- **Function:** Gets the end-effector coordinate system.
- **Return Value:**
  - `0` - Flange (Default)
  - `1` - Tool

### 18. Algorithm Parameters

<!-- #### `get_vr_mode()`

- **Function:** Gets the VR mode status.
- **Return Value:**
- `0`: Off
- `1`: On

#### `set_vr_mode(move)`

- **Function:** Sets the VR mode.
- **Parameters:**
- `move`: 1 - On, 0 - Off. -->

#### `get_model_direction()`

- **Function:** Gets the joint model direction.
- **Return Value:** Model directions for joints 1 through 6.
  - `1` - Same direction as the motor.
  - `0` - Opposite direction to the motor.

#### `set_model_direction(arm_id, joint_id, direction)`

- **Function:** Sets the joint model direction.
- **Parameters:**
  - `arm_id`: (`int`) Arm ID
    - `0`: Left and Right Arms
    - `1`: Left Arm
    - `2`: Right Arm
  - `joint_id (int)`: 1 ~ 9
  - `direction (int)`: `1` - Same direction as the motor. `0` - Opposite direction to the motor.

#### `get_filter_len(rank)`

- **Function:** Gets the filter parameters.
- **Parameters:**
  - `rank`: `int`
    - `1`: Drag-to-teach sampling filter
    - `2`: Drag-to-teach execution filter
    - `3`: Joint velocity blending filter
    - `4`: Cartesian velocity blending filter
    - `5`: Drag-to-teach sampling period
- **Return Value:** `int` (1 ~ 255)

#### `set_filter_len(rank, value)`

- **Function:** Sets filter parameters.
- **Parameters:**
  - `rank (int)`: 1 ~ 5
    - `1`: Drag-to-teach sampling filter
    - `2`: Drag-to-teach execution filter
    - `3`: Joint velocity blending filter
    - `4`: Cartesian velocity blending filter
    - `5`: Drag-to-teach sampling period
  - `value (int)`: 1 ~ 255

#### `get_fusion_parameters(rank_mode)`

- **Function:** Retrieves velocity blending planning parameters.
- **Parameters:**
  - `rank_mode`: 1 ~ 4
    - `1`: Blend joint velocity
    - `2`: Blend joint acceleration
    - `3`: Blend Cartesian velocity
    - `4`: Blend Cartesian acceleration
- **Return Value:** `int` (0 ~ 1000)

#### `set_fusion_parameters(rank_mode, value)`

- **Function:** Sets velocity blending planning parameters.
- **Parameters:**
  - `rank_mode (int)`: 1 ~ 4
  - `value (int)`: 0 ~ 1000

### 19. Kinematics Algorithm Interface

#### `solve_inv_kinematics(target_coords, current_angles)`

- **Function:** Converts Cartesian coordinates into joint angles.
- **Parameters:**
  - `target_coords`: `list` — A list of floating-point values ​​representing all target coordinates. 
  - `current_angles`: `list` — A list of floating-point values ​​representing all current joint angles of the robotic arm.
- **Return Value:** `list` — A list of floating-point values ​​representing the calculated joint angles.

### 20. Pro Force-Controlled Gripper

#### `get_pro_gripper_firmware_version(arm_id, gripper_id=14)`

- **Function**: Reads the major and minor firmware version numbers of the Pro force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, valid range is 1 ~ 254.

- **Return Value**: (`float`) Version number (format: x.x)

#### `get_pro_gripper_firmware_modified_version(arm_id, gripper_id=14)`

- **Function**: Reads the modified firmware version number of the Pro force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, valid range is 1 ~ 254.

- **Return Value**: (`int`) Modified version number

#### `set_pro_gripper_id(arm_id, target_id, gripper_id=14)`

- **Function**: Sets the ID of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `target_id` (`int`): Range: 1 ~ 254.
  - `gripper_id` (`int`): Gripper ID; default is 14, valid range is 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_id(arm_id, gripper_id=14)`

- **Function**: Reads the ID of the force-controlled gripper. - **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**: `int` (Range: 1 ~ 254).

#### `set_pro_gripper_angle(arm_id, gripper_angle, gripper_id=14)`

- **Function**: Sets the angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_angle` (`int`): Gripper angle; range is 0 ~ 100.
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_angle(arm_id, gripper_id=14)`

- **Function**: Reads the angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**: `int` (Range: 0 ~ 100).

#### `set_pro_gripper_open(arm_id, gripper_id=14)`

- **Function**: Opens the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254. - **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_close(arm_id, gripper_id=14)`

- **Function**: Closes the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
- `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_calibration(arm_id, gripper_id=14)`

- **Function**: Sets the zero position for the force-controlled gripper. (Calibration is required before first use.)
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_status(arm_id, gripper_id=14)`

- **Function**: Reads the gripping status of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254.
- **Return Value**:
  - `0` - Moving. 
  - `1` - Motion stopped; no object detected. 
  - `2` - Motion stopped; object detected. 
  - `3` - Object dropped (after detection). 

#### `set_pro_gripper_enabled(arm_id, state, gripper_id=14)`

- **Function**: Sets the enabled state of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `state` (`bool`): 0 or 1; 0 - Disable, 1 - Enable.
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_torque(arm_id, torque_value, gripper_id=14)`

- **Function**: Sets the torque of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `torque_value` (`int`): Torque value; range is 0 ~ 100.
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_torque(arm_id, gripper_id=14)`

- **Function**: Reads the torque of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; default is 14, range is 1 ~ 254.
- **Return Value**: (`int`) 0 ~ 100

#### `set_pro_gripper_speed(arm_id, speed, gripper_id=14)`

- **Function**: Sets the speed of the force-controlled gripper. - **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `speed` (`int`): Gripper movement speed; range: 1 to 100.
  - `gripper_id` (`int`): Gripper ID; default: 14; range: 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_speed(arm_id, gripper_id=14)`

- **Function**: Reads the speed of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254.
  - **Return Value**: The default movement speed of the gripper, range: 1 to 100.

#### `set_pro_gripper_abs_angle(arm_id, gripper_angle, gripper_id=14)`

- **Function**: Sets the absolute angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_angle` (`int`): Gripper angle, range: 0 to 100.
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_io_open_angle(arm_id, gripper_angle, gripper_id=14)`

- **Function**: Sets the IO-controlled opening angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `gripper_angle` (`int`): Gripper angle, range: 0 to 100.
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 to 254. - **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_io_open_angle(arm_id, gripper_id=14)`

- **Function**: Reads the IO opening angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 ~ 254.
- **Return Value**: `int` 0 ~ 100

#### `set_pro_gripper_io_close_angle(arm_id, gripper_angle, gripper_id=14)`

- **Function**: Sets the IO closing angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_angle` (`int`): Gripper angle; range: 0 ~ 100.
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_io_close_angle(arm_id, gripper_id=14)`

- **Function**: Reads the IO closing angle of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id` (`int`): Gripper ID; defaults to 14, range: 1 ~ 254. - **Return Value**: `int` 0 ~ 100

#### `set_pro_gripper_mini_pressure(arm_id, pressure_value, gripper_id=14)`

- **Function**: Sets the minimum activation force for the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `pressure_value` (`int`): Activation force value; range: 0 ~ 254.
  - `gripper_id` (`int`): Gripper ID; default: 14; range: 1 ~ 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_mini_pressure(arm_id, gripper_id=14)`

- **Function**: Reads the minimum activation force of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id` (`int`): Gripper ID; default: 14; range: 1 ~ 254.
- **Return Value**: (`int`) Activation force value; range: 0 ~ 254.

#### `set_pro_gripper_protection_current(arm_id, current_value, gripper_id=14)`

- **Function**: Sets the gripping current for the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `current_value` (`int`): Gripping current value; range: 100 ~ 300.
  - `gripper_id` (`int`): Gripper ID; default: 14; range: 1 ~ 254. - **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_protection_current(arm_id, gripper_id=14)`

- **Function**: Reads the gripping current of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id`: (`int`) Gripper ID; default is 14, valid range is 1 to 254.
- **Return Value**: (`int`) Gripping current value, range: 100 to 300.

#### `set_pro_gripper_baud(arm_id, baud_rate=0, gripper_id=14)`

- **Function**: Sets the baud rate for the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `baud_rate`: (`int`) Baud rate index, range: 0 to 1; default is 0 (115200).
    - `0` - 115200
    - `1` - 1000000
  - `gripper_id`: (`int`) Gripper ID; default is 14, valid range is 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `get_pro_gripper_baud(arm_id, gripper_id=14)`

- **Function**: Reads the baud rate of the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id`: (`int`) Gripper ID; default is 14, valid range is 1 to 254. - **Return Value**: (`int`) Baud rate index; default is 0 (115200).
  - `0` - 115200
  - `1` - 1000000

#### `set_pro_gripper_modbus(arm_id, state, custom_mode=False, gripper_id=14)`

- **Function**: Sets the Modbus communication mode for the force-controlled gripper.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID.
    - `1`: Left Arm
    - `2`: Right Arm
  - `state`: (`int`) Range: 0 to 1.
    - `0`: Disable Modbus communication mode; enable custom communication mode. 
    - `1`: Enable Modbus communication mode; disable custom communication mode.
  - `custom_mode`: (`bool`) Flag for custom communication mode; default is `False` (indicating the current mode is Modbus). If the system is currently in custom communication mode and you wish to switch *to* Modbus communication mode, you must set `custom_mode` to `True`. Example: `set_pro_gripper_modbus(1, True)`.
  - `gripper_id`: (`int`) Gripper ID; default is 14, range is 1 to 254.
- **Return Value**:
  - 0 - Failure
  - 1 - Success

#### `set_pro_gripper_init(arm_id, gripper_id=14)`

- **Function**: Initializes the gripper, restoring it to Modbus mode with a **baud rate of 115200**.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID.
    - `1`: Left Arm
    - `2`: Right Arm
  - `gripper_id`: (`int`) Gripper ID; default is 14, range is 1 to 254.
- **Return Value**: (`bool`)
  - `True` - Success
  - `False` - Failure

### 21. Aoyi Five-Finger Dexterous Hand

#### `get_five_fingers_angles(arm_id, hand_id=2)`

- **Function**: Reads the joint angles of all five fingers on the dexterous hand.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `hand_id`: (`int`) Five-finger device ID; defaults to 2, valid range is 0 ~ 254.
- **Return Value**: (`list[float]`) Angles for all five finger joints. For example, `[33.54, 173.83, 171.68, 172.1, 174.71, 1.0]` corresponds to `[Thumb Flexion, Index Finger, Middle Finger, Ring Finger, Little Finger, Thumb Rotation]`, respectively.

#### `get_five_fingers_angles(arm_id, finger_id, hand_id=2)`

- **Function**: Reads the angle of a single joint on the five-finger hand.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `finger_id`: (`int`) Range: 1 ~ 6.
    - `1`: Thumb Flexion
    - `2`: Index Finger
    - `3`: Middle Finger
    - `4`: Ring Finger
    - `5`: Little Finger
    - `6`: Thumb Rotation

- `hand_id`: (`int`) Five-finger device ID; defaults to 2, valid range is 0 ~ 254.

- **Return Value**: (`float`) The angle value. #### `set_five_fingers_angles(arm_id, fingers_angles, hand_id=2)`

- **Function**: Sets the joint angles for all five fingers.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `fingers_angles (list)`: A list of length 6; J1-J6 correspond to `[Thumb Flexion, Index Finger, Middle Finger, Ring Finger, Little Finger, Thumb Rotation]`.
    - `J1`: 2.26° ~ 36.76°
    - `J2`: 100.22° ~ 178.37°
    - `J3`: 97.81° ~ 176.06°
    - `J4`: 101.38° ~ 176.54°
    - `J5`: 98.84° ~ 174.86°
    - `J6`: 0° ~ 90°
  - `hand_id`: (`int`) Five-finger device ID; defaults to 2, valid range is 0 ~ 254.

#### `set_five_fingers_angle(arm_id, finger_id, finger_angle, hand_id=2)`

- **Function**: Sets the joint angle for a single finger.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `finger_id` (`int`): Range 1 ~ 6.
    - `1`: Thumb Flexion
    - `2`: Index Finger
    - `3`: Middle Finger
    - `4`: Ring Finger
    - `5`: Little Finger
    - `6`: Thumb Rotation
  - `finger_angle (int or float)`: The angle value. - `J1`: 2.26° ~ 36.76°
    - `J2`: 100.22° ~ 178.37°
    - `J3`: 97.81° ~ 176.06°
    - `J4`: 101.38° ~ 176.54°
    - `J5`: 98.84° ~ 174.86°
    - `J6`: 0° ~ 90°
  - `hand_id`: (`int`) Five-finger device ID; defaults to 2, range: 0 ~ 254.

#### `get_five_fingers_version(arm_id, hand_id=2)`

- **Function**: Reads the major and minor firmware version numbers of the five-finger dexterous hand.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `hand_id`: (`int`) Five-finger device ID; defaults to 2, range: 0 ~ 254.
- **Return Value**: (`float`) The major and minor firmware version number (e.g., `3.1`).

#### `get_five_fingers_hand_id(arm_id, hand_id=2)`

- **Function**: Reads the device ID of the five-finger dexterous hand.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `hand_id`: (`int`) Five-finger device ID; defaults to 2, range: 0 ~ 254.

- **Return Value**: (`int`) The five-finger device ID value. 

#### `set_five_fingers_hand_id(arm_id, target_hand_id, hand_id=2)`

- **Function**: Sets the device ID for the five-finger dexterous hand.
- **Parameters**:
  - `arm_id`: (`int`) Arm ID
    - `1`: Left arm
    - `2`: Right arm
  - `target_hand_id`: (`int`) The new five-finger device ID to be set; valid range: 0 to 254.
  - `hand_id`: (`int`) The current five-finger device ID (default: 2); valid range: 0 to 254.


----