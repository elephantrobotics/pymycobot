# API Method Details

[toc]

**When using the following function interfaces, please import our API library at the beginning; otherwise, the code will not run successfully. Enter the following code:**

```python 
from pymycobot import UltraArmP1
```

**Note:** If our API library is not installed, please refer to the [README.md](../README.md) document for installation instructions.

## 1 `set_reboot()`

- **Function:** Sets the robotic arm development board to restart.

- **Return Value:** None

## 2 `set_joint_release()`

- **Function:** Releases the joint

- **Return Value:** None

## 3 `set_joint_enable()`

- **Function:** Locks the joint

- **Return Value:** None

## 4 `get_system_version()`

- **Function:** Reads the firmware major version number

- **Return Value:** `float`, the correction version number

## 5 `get_modify_version()`

- **Function:** Reads the firmware correction version number

- **Return Value:** `int`, the correction version number

## 6 `get_angles_info()`

- **Function:** Gets the current angle of the robotic arm.

- **Return Value:** `list` is a list of floating-point values ​​representing the angles of all joints. [J1, J2, J3, J4]

## 7 `set_angle(id, angle, speed, _async=True)`

- **Function:** Sends the specified single joint to the specified angle.

- **Parameter Description:**

  - `id`: Represents the joint of the robotic arm, represented by numbers 1-4.

  - `degree`: Represents the joint angle

    <table>

    <tr>

    <th>Joint Id</th>

    <th>Range</th>

    </tr>

    <tr>

    <td text-align: center>1</td>

    <td>-160 ~ 160</td>

    </tr>

    <tr>

    <td>2</td>

    <td>-20 ~ 80</td>

    </tr>

    <tr>

    <td>3</td>

    <td>90 ~ 200</td>

    </tr>

    <tr>
    <td>4</td>

    <td>-180 ~ 180</td>

    </tr>


    </table>


  - `speed`: Represents the speed of the robotic arm's movement, ranging from 1 to 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Returns "ok" in closed-loop mode, 1 in open-loop mode.

## 8 `set_angles(angles, speed, _async=True)`

- **Function:** Sends all angles to all joints of the robotic arm.

- **Parameter Description:**

  - `degrees`: (List[float]) Contains the angles of all joints. A four-axis robot has four joints, so the length is 4, represented as [20,20,90, 20].

  - `speed`: Represents the speed of the robotic arm, ranging from 1 to 5700.

  - `_async`: Provides feedback on movement position, enabled by default.

- **Return Value:** Returns "ok" in closed-loop mode, 1 in open-loop mode.

## 9 `get_coords_info()`

- **Function:** Gets the current coordinates of the robotic arm.

- **Return Value:** A list of coordinates, with a length of 4, in the format [x, y, z, rx].

## 10 `set_coords_max_speed(coords, _async=True)`

- **Function:** Sends coordinate motion at maximum speed.

- **Parameter Description:**

  - `coords`: A list of coordinates of length 4 or 3, [X, Y, Z, RX] or [X, Y, Z].

    <table>

      <tr>

      <th>Coordinate Id</th>

      <th>Range</th>

      </tr>

      <tr>

      <td text-align: center>X</td>

      <td>-301.7 ~ 301.7</td>

      </tr>

      <tr>

      <td>Y</td>

      <td>-362.7 ~ 362.7</td>

      </tr> <tr>

      <td>Z</td>

      <td>-157 ~ 91</td>

      </tr>

      <tr>

      <td>Rx</td>

      <td>-180 ~ 180</td>

      </tr>

    </table>


  - `_async`: Motion positioning feedback, enabled by default.

- **Return value:** Closed-loop returns "ok", open-loop returns 1

## 11 `set_coords(coords, speed, _async=True)`

- **Function:** Sends global coordinates, allowing the robotic arm head to move from its original point to a specified point.

- **Parameter Description:**

  - `coords`: A list of coordinates of length 4 or 3, [X, Y, Z, RX] or [X, Y, Z]

    <table>

      <tr>

      <th>Coordinate Id</th>

      <th>Range</th>

      </tr>

      <tr>

      <td text-align: center>X</td>

      <td>-301.7 ~ 301.7</td>

      </tr>

      <tr>

      <td>Y</td>

      <td>-362.7 ~ 362.7</td>

      </tr>

      <tr>
      <td>Z</td>

      <td>-157 ~ 91</td>

      </tr>

      <tr>

      <td>Rx</td>

      <td>-180 ~ 180</td>

      </tr>


    </table>


  - `speed`: Represents the speed of the robotic arm's movement, ranging from 1 to 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return value:** Closed-loop returns "ok", open-loop returns 1.

## 12 `stop()`

- **Function:** Stops the robotic arm's movement.

- **Return value:** 1

## 13 `set_jog_angle(joint_id, direction, speed, _async=True)`

- **Function:** Sets the JOG angle.

- **Parameter description:**

  - `joint_id`: Represents the joint of the robotic arm, ranging from 1 to 4.

  - `direction`: Primarily controls the direction of the robotic arm's movement, 0 - positive movement, 1 - negative movement.

  - `speed`: Speed ​​from 1 to 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

## 14 `set_jog_coord(axis_id, direction, speed, _async=True)`

- **Function:** Sets the JOG coordinate motion.

- **Parameter Description:**

  - `axis_id`: Represents the joint coordinates of the robotic arm, ranging from 1 to 4.

  - `direction`: Primarily controls the direction of the robotic arm's movement, 0 - positive movement, 1 - negative movement.

  - `speed`: Speed ​​1 to 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

## 15 `jog_increment_angle(joint_id, increment, speed, _async=True)`

- **Function:** Sets the angle step motion

- **Parameter Description:**

  - `joint_id`: Represents the joint of the robotic arm, range 1 ~ 4

  - `increment`: Angle increment value.

  - `speed`: Speed ​​1 ~ 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

## 16 `jog_increment_coord(coord_id, increment, speed, _async=True)`

- **Function:** Sets the coordinate stepping motion.

- **Parameter Description:**

  - `axis_id`: Represents the joint coordinate of the robotic arm, ranging from 1 to 4.

  - `increment`: The coordinate increment value.

  - `speed`: Speed, ranging from 1 to 5700.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

## 17 `get_error_information()`

- **Function:** Reads error information

- **Return Value:** Error information

## 18 `set_pwm(p_value)`

- **Function:** Sets PWM control

- **Parameter Description:** `p_value` Duty cycle, range: 0-5

- **Return Value:** 1

## 19 `set_zero_calibration()`

- **Function:** Sets zero-point calibration

- **Return Value:** 1

## 19 `get_zero_calibration_state()`

- **Function:** Reads zero-point calibration status

- **Return Value:** `list` [1, 1, 1, 1]

## 20 `get_run_status()`

- **Function:** Read Running Status

- **Return Value:** Running Status

## 21 `open_laser()`

- **Function:** Turns on the laser

- **Return Value:** 1

## 22 `close_laser()`

- **Function:** Turns off the laser

- **Return Value:** 1

## 23 `set_gripper_angle(gripper_angle, gripper_speed)`

- **Function:** Sets the gripper's movement angle

- **Parameter Description:**

  - `gripper_angle`: `int`, 1 ~ 100.

  - `gripper_speed:` 1 ~ 100

- **Return value:** 1

## 24 `get_gripper_angle()`

- **Function:** Reads the gripper angle

- **Return value:** Gripper angle, 1 ~ 100

## 25 `set_gripper_parameter(addr, mode, parameter_value)`

- **Function:** Sets the gripper parameter

- **Parameter description:**

  - `addr`: `int`, 1 ~ 69

  - `mode:` (`int`): 1 ~ 2

  - `parameter_value` (`int`):
    - `Mode 1:` 0 ~ 255

    - `Mode 2:` Greater than 255
- **Return value:** 1

## 26 `get_gripper_parameter(addr, mode)`

- **Function:** Reads the gripper parameter
- **Parameter description:**
  - `addr`: `int`, 1 ~ 69

  - `mode:` (`int`): 1 ~ 2
- **Return value:** (int) Gripper parameter
    - `Mode 1:` 0 ~ 255

    - `Mode 2:` Greater than 255

## 27 `set_gripper_enable_status(state):`

- **Function:** Sets the gripper parameters

- **Parameter Description:**

  - `state`: `int`

    - `0`: Disabled

    - `1`: Enabled

- **Return Value:** 1

## 28 `set_gripper_zero()`

- **Function:** Sets the gripper to zero position

- **Return Value:** 1

## 29 `set_pump_state(pump_state)`

- **Function:** Sets the pump status

- **Parameter Description:**

  - `pump_state`: `int`

    - `0`: On

    - `1`: Off

    - `2`: Off

- **Return Value:** 1

## 30 `set_basic_io_output(pin_no, pin_signal)`

- **Function:** Sets the base IO pin output status

- **Parameter Description:**

  - `pin_no`: `int` 1 ~ 10

  - `pin_signal`: `int`

    - `0`: Low level

    - `1`: High level

- **Return value:** 1

## 31 `set_digital_io_output(pin_no, pin_signal)`

- **Function:** Sets the output state of the final I/O pin

- **Parameter description:**

  - `pin_no`: `int` 1 ~ 4

  - `pin_signal`: `int`

    - `0`: Low level

    - `1`: High level

- **Return value:** 1

## 32 `set_outer_shaft(shaft_state, speed)`

- **Function:** Sets the external axis

- **Parameter description:**

  - `shaft_state`: `int`

    - `0`: Off

    `1`: On

  - `speed`: `int` 1 ~ 5700

- **Return value:** 1

## 33 `set_i2c_data(data_state, data_addr, data_len, data_value)`

- **Function:** Sets I2C data

- **Parameter description:**

  - `data_state`: `int` 0 ~ 1

  - `0`: Read

  - `1`: Write

  - `data_addr`: `int` 0 ~ 255

  - `data_len`: `int`: 0 ~ 64

  - `data_value`: `int` 0 ~ 255

- **Return value:** 1

## 34 `drag_teach_start()`

- **Function:** Starts drag-and-drop teaching

- **Return value:** 1

## 35 `drag_teach_save()`

- **Function:** save drag-and-drop teaching

- **Return Value:** 1

## 36 `drag_teach_pause()`

- **Function:** Pause drag-and-drop teaching

- **Return Value:** 1

## 37 `drag_teach_resume()`

- **Function:** Resume drag-and-drop teaching

- **Return Value:** 1

## 38 `drag_teach_stop()`

- **Function:** Stop drag-and-drop teaching

- **Return Value:** 1

## 39 `drag_teach_execute()`

- **Function:** Execute drag-and-drop teaching

- **Return Value:** 1

## 40 `wifi_open()`

- **Function:** Turn on Wi-Fi

- **Return Value:** 1

## 41 `get_system_screen_version()`

- **Function:** Reads the screen firmware major version number

- **Return Value:** Major version number

## 42 `get_modify_screen_version()`

- **Function:** Reads the screen firmware correction version number

- **Return Value:** Correction version number

## 43 `set_communication_baud_rate(baud_rate)`

- **Function:** Sets the communication baud rate

- **Parameter Description:**

  - `baud_rate`: `int` standard baud rate, 115200 or 1000000

- **Return Value:** 1

## 44 `update_stm_firmware()`

- **Function:** Updates the STM32 firmware

- **Return Value:** 1

## 45 `receive_485_data()`

- **Function:** Receives 485 data

- **Return Value:** 485 data

## 46 `play_gcode_file(filename)`

- **Function:** Plays the imported track file.

- **Parameter Description:**

  - `filename`: Track file name

- **Return Value:** None