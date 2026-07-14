# UltraArm P1 API Method Details

## Preparation Before Use

Before using Case Function I, please ensure the following hardware and environment are ready:

- **Hardware Equipment**

  - ultraArm P1 robotic arm

  - USB-Type-C serial cable (for connecting the robotic arm to the computer)

  - Power adapter

- **Software and Environment**

  - Python 3.6 or later is installed

  - The `pymycobot` library is installed (installed via the terminal command `pip install pymycobot`)

  - Ensure the ultraArm P1 is properly powered on and in standby mode

---

## USB Serial Communication

[toc]

**When using the following function interfaces, please import our API library at the beginning; otherwise, the code will not run successfully. Enter the following code:**

```python 
from pymycobot import UltraArmP1
```

**Note:** If our API library is not installed, please refer to the [README.md](../README.md) document for installation instructions.

```python
# demo
from pymycobot import UltraArmP1

ua = UltraArmP1("COM3",1000000)

res = ua.get_angles_info()
print(res)

ua.set_angles([0, 0, 90, 0],50)
...
```

### 1 `set_joint_release(joint_id)`

- **Function:** Releases the joint

- **Parameter Description:**

  - `joint_id`(`int`): Joint number (1~4); 0 indicates all joints.

- **Return value：** ok

### 2 `set_joint_enable(joint_id)`

- **Function:** Locks the joint

- **Parameter Description:**

  - `joint_id`(`int`): Joint number (1~4); 0 indicates all joints.

- **Return value：** ok

### 3 `get_system_version()`

- **Function:** Reads the firmware major version number

- **Return Value:** `float`, the correction version number

### 4 `get_modify_version()`

- **Function:** Reads the firmware correction version number

- **Return Value:** `int`, the correction version number

### 5 `get_angles_info()`

- **Function:** Gets the current angle of the robotic arm.

- **Return Value:** `list` is a list of floating-point values ​​representing the angles of all joints. [J1, J2, J3, J4]

### 6 `set_angle(id, angle, speed, _async=True)`

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

    <td>-165 ~ 165</td>

    </tr>

    <tr>

    <td>2</td>

    <td>-18 ~ 85</td>

    </tr>

    <tr>

    <td>3</td>

    <td>89 ~ 200</td>

    </tr>

    <tr>
    <td>4</td>

    <td>-179 ~ 179</td>

    </tr>


    </table>


  - `speed`: Represents the speed of the robotic arm's movement, ranging from 1 to 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Returns "ok" in closed-loop mode, 1 in open-loop mode.

### 7 `set_angles(angles, speed, _async=True)`

- **Function:** Sends all angles to all joints of the robotic arm.

- **Parameter Description:**

  - `degrees`: (List[float]) Contains the angles of all joints. A four-axis robot has four joints, so the length is 4, represented as [20,20,90, 20].

  - `speed`: Represents the speed of the robotic arm, ranging from 1 to 100.

  - `_async`: Provides feedback on movement position, enabled by default.

- **Return Value:** Returns "ok" in closed-loop mode, 1 in open-loop mode.

### 8 `get_coords_info()`

- **Function:** Gets the current coordinates of the robotic arm.

- **Return Value:** A list of coordinates, with a length of 4, in the format [x, y, z, rx].

### 9 `set_coords_max_speed(coords, _async=True)`

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

      <td>-350 ~ 362.43</td>

      </tr>

      <tr>

      <td>Y</td>

      <td>-362.43 ~ 362.43</td>

      </tr> <tr>

      <td>Z</td>

      <td>-186.265 ~ 93.44</td>

      </tr>

      <tr>

      <td>Rx</td>

      <td>-180 ~ 180</td>

      </tr>

    </table>


  - `_async`: Motion positioning feedback, enabled by default.

- **Return value:** Closed-loop returns "ok", open-loop returns 1

### 10 `set_coords(coords, speed, _async=True)`

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

      <td>-350 ~ 362.43</td>

      </tr>

      <tr>

      <td>Y</td>

      <td>-362.43 ~ 362.43</td>

      </tr>

      <tr>
      <td>Z</td>

      <td>-186.265 ~ 93.44</td>

      </tr>

      <tr>

      <td>Rx</td>

      <td>-180 ~ 180</td>

      </tr>


    </table>


  - `speed`: Represents the speed of the robotic arm's movement, ranging from 1 to 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return value:** Closed-loop returns "ok", open-loop returns 1.

### 11`set_coord(coord_id, coord, speed, _async=True)`

- **Function:** Sends a single axis coordinate, allowing the robotic arm head to move from its original point to a specified point.

- **Parameter Description:**

  - `coord_id`: (`str`), axis ID, 'X', 'Y', 'Z', 'R'

  - `coords`: coordinate values

    <table>

      <tr>

      <th>coordinate ID</th>

      <th>range</th>

      </tr>

      <tr>

      <td text-align: center>X</td>

      <td>-350 ~ 362.43</td>

      </tr>

      <tr>

      <td>Y</td>

      <td>-362.43 ~ 362.43</td>

      </tr>

      <tr>

      <td>Z</td>

      <td>-186.265 ~ 93.44</td>

      </tr>

      <tr>
      <td>Rx</td>

      <td>-180 ~ 180</td>
      </tr>

    </table>

  - `speed`: Represents the speed of the robotic arm's movement, ranging from 1 to 100.

  - `_async`: Feedback on movement completion, enabled by default.

- **Return value:** Closed-loop returns "ok", open-loop returns 1.

### 12 `stop()`

- **Function:** Stops the robotic arm's movement.

- **Return value：** ok

### 13 `set_jog_angle(joint_id, direction, speed, _async=True)`

- **Function:** Sets the JOG angle.

- **Parameter description:**

  - `joint_id`: Represents the joint of the robotic arm, ranging from 1 to 4.

  - `direction`: Primarily controls the direction of the robotic arm's movement, 1 - positive movement, 0 - negative movement.

  - `speed`: Speed ​​from 1 to 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

### 14 `set_jog_coord(axis_id, direction, speed, _async=True)`

- **Function:** Sets the JOG coordinate motion.

- **Parameter Description:**

  - `axis_id`: Represents the joint coordinates of the robotic arm, ranging from 1 to 4.

  - `direction`: Primarily controls the direction of the robotic arm's movement, 1 - positive movement, 0 - negative movement.

  - `speed`: Speed ​​1 to 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

### 15 `jog_increment_angle(joint_id, increment, speed, _async=True)`

- **Function:** Sets the angle step motion

- **Parameter Description:**

  - `joint_id`: Represents the joint of the robotic arm, range 1 ~ 4

  - `increment`: Angle increment value.

  - `speed`: Speed ​​1 ~ 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

### 16 `jog_increment_coord(coord_id, increment, speed, _async=True)`

- **Function:** Sets the coordinate stepping motion.

- **Parameter Description:**

  - `axis_id`: Represents the joint coordinate of the robotic arm, ranging from 1 to 4.

  - `increment`: The coordinate increment value.

  - `speed`: Speed, ranging from 1 to 100.

  - `_async`: Motion positioning feedback, enabled by default.

- **Return Value:** Closed-loop returns "ok", open-loop returns 1

### 17 `get_error_information()`

- **Function:** Reads error information

- **Return Value:** Error information

### 18 `set_zero_calibration()`

- **Function:** Sets zero-point calibration

- **Return value：** ok

### 19 `get_zero_calibration_state()`

- **Function:** Reads zero-point calibration status

- **Return Value:** `list` [1, 1, 1, 1]

### 20 `get_run_status()`

- **Function:** Read Running Status

- **Return Value:** Running Status

### 21 `set_pwm_laser_mode(state)`

- **Function:** Set pwm laser mode.
- **Parameters:** `state` (`int`) 0 ~ 1; 0 - Off; 1 - On.
- **Return Value:** `ok` - Success; `0` - Failure.

### 22 `set_pwm_laser(p_value)`

- **Function:** Set the PWM level (Laser).
- **Parameters:** `p_value` (`int`) 0 ~ 255.
- **Return Value:** `ok` - Success; `0` - Failure.

### 23 `set_pwm_custom_mode(state)`

- **Function:** Set pwm custom mode.
- **Parameters:** `state` (`int`) 0 ~ 1; 0 - Off; 1 - On.
- **Return Value:** `ok` - Success; `0` - Failure.

### 24 `set_pwm_custom(p_value)`

- **Function:** Set the PWM level (Custom).
- **Parameters:** `p_value` (`int`) 0 ~ 255.
- **Return Value:** `ok` - Success; `0` - Failure.

### 25 `set_gripper_angle(gripper_angle, gripper_speed)`

- **Function:** Sets the gripper's movement angle

- **Parameter Description:**

  - `gripper_angle`: `int`, 1 ~ 100.

  - `gripper_speed:` 1 ~ 100

- **Return value：** ok

### 26 `get_gripper_angle()`

- **Function:** Reads the gripper angle

- **Return value:** Gripper angle, 1 ~ 100

### 27 `set_gripper_parameter(addr, parameter_value)`

- **Function:** Sets the gripper parameter

- **Parameter description:**

  - `addr`: `int`, 1 ~ 69

  - `parameter_value` (`int`): 0 ~ 65535
- **Return value：** ok

### 28 `get_gripper_parameter(addr)`

- **Function:** Reads the gripper parameter
- **Parameter description:**
  - `addr`: `int`, 1 ~ 69

  - `mode:` (`int`): 1 ~ 2
- **Return value:** (int) Gripper parameter 0 ~ 65535

### 29 `set_gripper_enable_status(state):`

- **Function:** Set the gripper enable state

- **Parameter Description:**

  - `state`: `int`

    - `0`: Disabled

    - `1`: Enabled

- **Return value：** ok

### 30 `set_gripper_zero()`

- **Function:** Sets the gripper to zero position

- **Return value：** ok

### 31 `set_pump_state(pump_state)`

- **Function:** Sets the pump status

- **Parameter Description:**

  - `pump_state`: `int`

    - `0`: On

    - `1`: Off

    - `2`: Off

- **Return value：** ok

### 32 `set_base_io_output(pin_no, pin_status, pin_signal)`

- **Function:** Sets the base IO pin output status

- **Parameter Description:**

  - `pin_no`: `int` 1 ~ 10

  - `pin_status`: `int`

    - `0`: input

    - `1`: output

  - `pin_signal`: `int`

    - `0`: Low level

    - `1`: High level

- **Return value：** ok

### 33 `set_digital_io_output(pin_no, pin_signal)`

- **Function:** Sets the output state of the final I/O pin

- **Parameter description:**

  - `pin_no`: `int` 3 ~ 4

  - `pin_signal`: `int`

    - `0`: Low level

    - `1`: High level

- **Return value：** ok

### 34 `set_i2c_data(session_id, package_id, data_state, data_addr, register_addr, data_len, data_value)`

- **Function:** Communicates with sensors through the I2C protocol.

- **Parameter description:**

  - `session_id`: `int` 0 ~ 255, session ID, protocol parameter `I`
  - `package_id`: `int` 0 ~ 255, package ID, protocol parameter `U`
  - `data_state`: `int` 0 ~ 1, protocol parameter `S`
    - `0`: Read
    - `1`: Write
  - `data_addr`: `int` 0 ~ 125, slave address, protocol parameter `L`
  - `register_addr`: `int | str` 0 ~ 65535, register address, protocol parameter `H`; `0xFFFF` or `"FFFF"` means no register parameter; pass a hex string such as `"14"` to preserve `H14`
  - `data_len`: `int | str` 0 ~ 255, data length, protocol parameter `N`; pass a hex string such as `"0C"` to preserve `N0C`
  - `data_value`: `int | list[int] | bytes | str | None`, write data, protocol parameter `K`, maximum 32 bytes per command; use `None` or `[]` when reading with no write data; pass a contiguous hex string such as `"000C"` to preserve `K000C`

- **Return value:**

  - Returns `ok` when write succeeds
  - Returns `list[str]` when read succeeds, for example `['8f', '15']`
  - Returns an error code on failure, for example `1 ~ 7`

- **Example:**

```python
# Write data, sends: M300 I1 U1 S1 L13 H45 N2 K8F 1E
ua.set_i2c_data(1, 1, 1, 13, 45, 2, [0x8f, 0x1e])

# Read 2 bytes, no register, no write data, sends: M300 I1 U1 S0 L23 HFFFF N2 K
ua.set_i2c_data(1, 1, 0, 23, "FFFF", 2, None)

# Send raw hex text, sends: M300 I1 U12 S1 L29 H14 N0C K000C
ua.set_i2c_data(1, 12, 1, 29, "14", "0C", "000C")
```

### 35 `play_gcode_file(filename)`

- **Function:** Plays the imported track file.For laser engraving, please use the `play_gcode_file_laser(filename)` interface.

- **Parameter Description:**

  - `filename`: Track file name

- **Return Value:** None

### 36 `get_system_screen_version()`

- **Function:** Reads the screen firmware major version number

- **Return Value:** Major version number

### 37 `get_modify_screen_version()`

- **Function:** Reads the screen firmware correction version number

- **Return Value:** Correction version number

### 38 `set_communication_baud_rate(baud_rate)`

- **Function:** Sets the communication baud rate

- **Parameter Description:**

  - `baud_rate`: `int` standard baud rate, 115200 or 1000000

- **Return value：** ok

### 39 `receive_485_data()`

- **Function:** Receives 485 data

- **Return Value:** 485 data


### 40 `set_wifi_password(wifi_name, password=None)`

- **Function:** Sets the on-screen WiFi password.

- **Parameter Description:**
  - `wifi_name`: (`str`) SSID， WiFi name.

  - `password`: (`str`) WiFi password string, length 8~15 characters.

- **Return value：** ok

### 41 `check_sd_card()`

- **Function:** Checks if an SD card is present.

- **Return Value:** (`str`)

  - `ok`: SD card present

  - `0`: SD card not present

### 42 `download_firmware_sd(filename, show_progress=True)`

- **Function:** Downloads firmware data to the SD card.

- **Parameter Description:**

  - `filename`: (`str`) The name of the firmware file, which must be a .bin file (It is recommended to use a fixed name to avoid storing too many files on the SD card.).

  - `show_progress`: (`bool`) Whether to display the download progress; the default is to display it.

- **Return Value:** If `show_progress=True`, the download progress is returned; otherwise, no value is returned.

### 43 `upgrade_restart()`

- **Function:** Firmware upgrade and restart.

- **Return value：** ok

### 44 `get_motor_enable_status()`

- **Function:** Reads the motor enable status.

- **Return Value:** `list`, 5 motor enable statuses.

### 45 `finish_firmware_upgrade()`

- **Function:** Ends the download of firmware data to the SD card. (The upgrade can be terminated midway through the firmware download process.)

- **Return value：** ok.

### 46 `get_base_io_state()`

- **Function:** Get the state of the base IO pins
  
- **Return Value:** `list` with values in the range 0-3, length is 10

  - `0`: Input, level = 0 (low level)
  - `1`: Input, level = 1 (high level)
  - `2`: Output, level = 0 (low level)
  - `3`: Output, level = 1 (high level)

### 47 `get_base_io_state(pin_no)`

- **Function:** Get the state of the base IO pins
- **Parameter:** (`int`) Bottom IO pin number, range: 1 to 10.
- **Return Value:** `int` range 0 ~ 3

  - `0`: Input, level = 0 (low level)
  - `1`: Input, level = 1 (high level)
  - `2`: Output, level = 0 (low level)
  - `3`: Output, level = 1 (high level)

### 48 `get_end_io_state()`

- **Function:** Get the state of the end IO pins

- **Return Value:** `list` with values in the range 0-3, length is 4

  - `0`: Input, level = 0 (low level)
  - `1`: Input, level = 1 (high level)
  - `2`: Output, level = 0 (low level)
  - `3`: Output, level = 1 (high level)

### 49 `get_end_io_state(pin_no)`

- **Function:** Get the state of the end IO pins
- **Parameter:** (`int`) End IO pin number, range: 1 to 4.
- **Return Value:** `int` range 0 ~ 3

  - `0`: Input, level = 0 (low level)
  - `1`: Input, level = 1 (high level)
  - `2`: Output, level = 0 (low level)
  - `3`: Output, level = 1 (high level)

### 50 `set_end_button_enable()`

- **Function:** Enable the end button
  
- **Return value：** ok

### 51 `set_end_button_disable()`

- **Function:** Disable the end button

- **Return value：** ok

### 52 `forced_reset_zero()`

- **Function:** Set forced homing (reset to zero)
  
- **Return value：** ok

### 53 `set_conveyor_control(state, direction, speed, distance)`

- **Function:** Conveyor belt control
- **Parameter Description:**
  - `state`: (`int`) 0~1, conveyor state: 0 - off; 1 - on
  - `direction`: (`int`) 0~1, conveyor direction: 0 - forward; 1 - backward
  - `speed`: (`int`) conveyor speed, range 1~125 mm/s
  - `distance`: (`int`) conveyor distance, range 0~1200 mm, 0: Continuous movement; 1-1200: Movement 1-1200mm (maximum stroke)
- **Return value：** ok

### 54 `set_color(r, g, b)`

- **Function:** Sets the RGB color of the light panel.
- **Parameter Description:**
  - `r`: (`int`) Red component; range: 0 ~ 255.
  - `g`: (`int`) Green component; range: 0 ~ 255.
  - `b`: (`int`) Blue component; range: 0 ~ 255.

- **Return value：** ok

### 55 `set_preview_mode(coords)`

- **Function:** Sets the coordinate trajectory preview mode.
- **Parameter Description:**
  - `coords` (list[float]): A list of coordinate values ​​[X, Y, Z, R].

- **Return value：** ok

### 56 `get_sd_card_space()`

- **Function:** Retrieves the total and available storage space on the SD card.

- **Return Value:** `list` containing the total space and available space, in bytes. Example: [Total Space, Available Space]

### 57 `collision_unlock()`

- **Function:** Unlocks after collision detection.

- **Return Value:** OK - Success; 0 - Failure.

### 58 `clear_error_status()`

- **Function:** Clears the error status. If a limit has been exceeded, the joint must be manually moved back within the limit boundaries.

- **Return Value:** OK - Success; 0 - Failure

### 59 `get_queue_size()`

- **Function:** Reads the size of the buffer queue.

- **Return Value:** `int` Queue size

### 60 `set_robot_id(robot_id)`

- **Function:** Sets the device machine code

- **Parameter Description:**

  - `robot_id`: (`str`) Machine code ID, range 001 ~ 254, length 3

- **Return Value:** ok

### 61 `get_robot_id()`

- **Function:** Reads the device machine code

- **Return Value:** Machine code ID, range 001 ~ 254, length 3

### 62 `get_wifi_ip()`

- **Function:** Reads the WiFi IP address (only available when WiFi is connected)

- **Return Value:** `str` WiFi IP address

### 63 `get_bluetooth_mac()`

- **Function:** Reads the Bluetooth communication MAC address (only available when Bluetooth is enabled)

- **Return Value:** `str` Bluetooth MAC address

### 64 `get_end_button_state()`

- **Function:** Reads the state of the end button

- **Return value:**

  - `0`: Not pressed

  - `1`: Pressed

### 65 `coord_inverse_solution(coords)`

- **Function:** Inverse coordinate solution; input coordinates and read angles.

- **Parameter Description:**

  - `coords` (list[float]): List of coordinate values ​​[X, Y, Z, R]

- **Return Value:**

  - `angles` (list[float]): List of angle values ​​[J1, J2, J3, J4]

### 66 `angle_correct_solution(angles)`

- **Function:** Correct angle solution; input angles and read coordinates.

- **Parameter Description:**

  - `angles` (list[float]): List of angle values ​​[J1, J2, J3, J4]

- **Return Value:**

  - `coords` (list[float]): List of coordinate values ​​[X, Y, Z, R]

### 67 `get_wifi_signal_strength()`

- **Function:** Reads WiFi signal strength (only available when WiFi is connected)

- **Return Value:** `int` signal strength, in dBm, e.g., `-62`

### 68 `get_bluetooth_signal_strength()`

- **Function:** Reads Bluetooth signal strength (only available when Bluetooth is connected)

- **Return Value:** `int` signal strength, in dBm, e.g., `-52`

### 69 `set_collision_threshold(joint_id, threshold)`

- **Function:** Sets the joint collision threshold.

- **Parameter Description:**

- `joint_id` `(int)` : 0 ~ 4

  - 0 : All joint

  - 1: J1

  - 2: J2

  - 3: J3

  - 4: J4

- `threshold` (float/int): Threshold value, range 0.5 ~ 100

- **Return Value:**

  - `ok`

### 70 `get_communication_mode()`

- **Function:** Retrieves the current communication mode.

- **Return Value:**

  - Uart0 - Communicates using serial port 0

  - Uart1 - Communicates using serial port 1

  - WiFi - Communicates using WiFi

  - Bluetooth - Communicates using Bluetooth

### 71 `set_uart1_communication(state)`

- **Function:** Sets communication using serial port 1.

- **Parameter Description:**

  - `state` `(int)` : 0 ~ 1; 0 - Off; 1 - On

- **Return Value:**

  - `ok`

### 72 `get_collision_threshold()`

- **Function:** Reads the collision threshold of a joint.

- **Return value:** `(list[float])`: The collision threshold for all joints, e.g., [0.5, 0.5, 0.5, 0.5]

### 73 `get_pwm_status()`

- **Function:** Gets the PWM output status.

- **Return value:** `(list[int])`, length 4, e.g., [0, 0, 0, 0]

  - [0]: Laser mode status, 0 - Off, 1 - On

  - [1]: PWM value for laser mode, range 0 ~ 255

  - [2]: Custom mode status, 0 - Off, 1 - On

  - [3]: PWM value for custom mode, range 0 ~ 255

### 74 `get_limit_switch_state()`

- **Function:** Gets the limit switch state.

- **Return value:** `int`

  - `0`: Not triggered

  - `1`: Triggered

### 75 `laser_engraving_pause_time(pause_time)`

- **Function:** Sets the laser engraving pause time.

- **Parameter description:**

  - `pause_time`: `int`, pause time, range 1 ~ 1000 ms

- **Return value:** `ok`

### 76 `set_conveyor_stop()`

- **Function:** Sets the conveyor belt to stop moving.

- **Return Value:**

  - `ok`

---

## TCP Socket Communication

Uses TCP/IP to control the robotic arm.

### Server-Side

The connection mode needs to be switched to **WLAN** on the small screen on the robotic arm base, and a wireless WiFi network needs to be connected. Once the WiFi connection is successful, the server is successfully started, and the screen will display the server's IP address and port number.

### Client-Side

>> Note:<br>1. The IP address and port number are provided by the server.<br>2. The PC's network needs to be on the same network segment as the server's network.<br>3. The interface name and usage method for socket communication are the same as for serial communication.

```python

# Example
from pymycobot import UltraArmP1Socket

# Default port 9000

ua = UltraArmP1Socket("192.168.10.10",9000)

res = ua.get_angles_info()

print(res)

ua.set_angles([0, 0, 90, 0],50)

...
```

---

## BLE Bluetooth Communication

Use the BLE GATT Bluetooth communication service to control the robotic arm.

### Server-Side

The connection mode needs to be switched to **Bluetooth** on the small screen on the robotic arm base, and the Bluetooth service needs to be enabled. After enabling Bluetooth, the screen will display the Bluetooth **connection status**, **Bluetooth name**, and **Bluetooth MAC address**.

### Client-Side

>> Note:<br>1. The **Bluetooth address** can only be obtained after Bluetooth is enabled on the robotic arm screen.<br>2. After successfully connecting via Bluetooth, the Bluetooth connection status on the robotic arm's small screen will display **Connected**.<br>3. The interface name and usage method for Bluetooth communication are the same as for serial communication.

```python

# Example
from pymycobot import UltraArmP1Bluetooth

# Modify the Bluetooth address according to the actual situation
ua = UltraArmP1Bluetooth("10:51:DB:40:2C:11")

res = ua.get_angles_info()

print(res)

ua.set_angles([0, 0, 90, 0],50)

...
```

---
