# MyPalletizer 260

[toc]

## Python API usage instaructions

API (Application Programming Interface), also known as Application Programming Interface functions, are predefined functions. When using the following function interfaces, please import our API library at the beginning by entering the following code, otherwise it will not run successfully:

```python
# Example
from pymycobot import MyPalletizer260

mc = MyPalletizer260('COM3')

# Gets the current angle of all joints
angles = mc.get_angles()
print(angles)

# Set 1 joint to move to 40 and speed to 20
mc.send_angle(1, 40, 20)
```

### 1. System Status

#### `get_system_version()`

- **function：** get system version
- **Return value：** system version

#### `get_basic_version()`

- **function：** Get basic firmware version for M5 version
- **Return value：** `float` firmware version

### 2. Overall Status

#### `power_on()`

- **function:** atom open communication (default open)

#### `power_off()`

- **function:** Power off of the robotic arm

#### `is_power_on()`

- **function:** judge whether robot arms is powered on or not

- **Return value:**
  - `1`: power on
  - `0`: power off
  - `-1`: error

#### `release_all_servos()`

- **function:** release all robot arms
  - Attentions：After the joint is disabled, it needs to be enabled to control within 1 second
- **Parameters**：`data`（optional）：The way to relax the joints. The default is damping mode, and if the 'data' parameter is provided, it can be specified as non damping mode (1- Undamping).

#### `focus_servos(servo_id)`

- **function:** Power on designated servo

- **Parameters:** 
  - `servo_id:` int, 1-4

#### `is_controller_connected()`

- **function:** Wether connected with Atom

- **Return value:**
  - `1`: succeed
  - `0`: failed
  - `-1`: error data

#### `set_free_mode()`

- **function:** set to free mode
  
- **Parameters:**
  - `1`: open free mode
  - `0`: close free mode

#### `is_free_mode()`

- **function:** Check if it is free mode

- **Return value:** 
  - `1`: free mode
  - `0`: on-free mode

### 3.MDI Mode and Operation

#### `get_angles()`

- **function:** get the degree of all joints
- **Return value**: `list  `a float list of all degree

#### `send_angle(id, degree, speed)`

- **function:** send one degree of joint to robot arm
- **Parameters:**
  - `id`: Joint id(`genre.Angle`), range int 1-4
  - `degree`: degree value(`float`)
    | Joint Id | range |
    | ---- | ---- |
    | 1 | -162 ~ 162 |
    | 2 | -2 ~ 90 |
    | 3 | -92 ~ 60 |
    | 4 | -180 ~ 180 |

  - `speed`：the speed and range of the robotic arm's movement 1~100

#### `send_angles(angles, speed)`

- **function：** Send all angles to all joints of the robotic arm
- **Parameters:**
  - `angles`: a list of degree value(`List[float]`), length 4
  - `speed`: (`int`) 1 ~ 100

#### `get_coords()`

- **function:** Obtain robot arm coordinates from a base based coordinate system
- **Return value:** a float list of coord:[x, y, z, rx]

#### `send_coord(id, coord, speed)`

- **function:** send one coord to robot arm
- **Parameters:**
  - `id`:send one coord to robot arm, 1-4 corresponds to [x, y, z, rx]
  - `coord`: coord value(`float`)
    | Coord Id | range |
    | ---- | ---- |
    | x | -260 ~ 260 |
    | y | -260 ~ 260 |
    | z | -15 ~ 357.58 |
    | rx | -180 ~ 180 |
  - `speed`: (`int`) 1-100

#### `send_coords(coords, speed)`

- **function:**: Send overall coordinates and posture to move the head of the robotic arm from its original point to your specified point
- **Parameters:**
  - coords: ： a list of coords value `[x,y,z,rx]`,length 4
  - speed`(int)`: 1 ~ 100

#### `pause()`

- **function:** Control the instruction to pause the core and stop all movement instructions
- **Return value**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `sync_send_angles(angles, speed)`

- **function：** Send the angle in synchronous state and return when the target point is reached
- **Parameters:**
  - `angles`: a list of degree value(`List[float]`), length 4
  - `speed`: (`int`) 1 ~ 100

#### `sync_send_coords(coords, speed)`

- **function：** Send the coord in synchronous state and return when the target point is reached
- **Parameters:**
  - `coords`: a list of coord value(`List[float]`), length 4
  - `speed`: (`int`) 1 ~ 100

#### `get_angles_coords()`

- **function：** Get joint angles and coordinates

- **Return value:** A list with a length of 8. The first four digits are angle information, and the last four digits are coordinate information. 

#### `is_paused()`

- **function:** Check if the program has paused the move command
- **Return value:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **function:** resume the robot movement and complete the previous command

#### `stop()`

- **function:** stop all movements of robot
- **Return value**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `is_in_position(data, flag)`

- **function** : judge whether in the position.
- **Parameters:**
  - data: Provide a set of data that can be angles or coordinate values. If the input angle length range is 4, and if the input coordinate value length range is 4
  - flag data type (value range 0 or 1)
    - `0`: angle
    - `1`: coord
- **Return value**:
  - `1` - true
  - `0` - false
  - `-1 ` - error

#### `is_moving()`

- **function:** judge whether the robot is moving
- **Return value:**
  - `1` moving
  - `0` not moving
  - `-1` error

### 4. JOG Mode and Operation

#### `jog_angle(joint_id, direction, speed)`

- **function:** jog control angle
- **Parameters**:
  - `joint_id`: Represents the joints of the robotic arm, represented by joint IDs ranging from 1 to 4
  - `direction(int)`: To control the direction of movement of the robotic arm, input `0` as negative value movement and input `1` as positive value movement
  - `speed`: 1 ~ 100

#### `jog_coord(coord_id, direction, speed)`

- **function:** jog control coord.
- **Parameters:**
  - `coord_id`: (`int`) Coordinate range of the robotic arm: 1~4
  - `direction`: (`int`) To control the direction of machine arm movement, `0` - negative value movement, `1` - positive value movement
  - `speed`: 1 ~ 100

#### `jog_absolute(joint_id, angle, speed)`

- **function:** Jog absolute angle
- **Parameters:**
  - `joint_id`: (`int`) 1-4
  - `angle`: -180 ~ 180
  - `speed`: (`int`) 1 ~ 100

#### `jog_increment(joint_id, increment, speed)`

- **function:** Single joint angle increment control
- **Parameters**:
  - `joint_id`: 1-4
  - `increment`: Incremental movement based on the current position angle
  - `speed`: 1 ~ 100

#### `set_encoder(joint_id, encoder, speed)`

- **function**: Set a single joint rotation to the specified potential value

- **Parameters**

  - `joint_id`: (`int`) 1-4
  - `encoder`: 0 ~ 4096
  - `speed`: 1 ~ 100

#### `get_encoder(joint_id)`

- **function**: Set a single joint rotation to the specified potential value

- **Parameters**

  - `joint_id`: (`int`) 1-4

- **Return value:** (`int`) Joint potential value

#### `set_encoders(encoders, speed)`

- **function**: Set the six joints of the manipulator to execute synchronously to the specified position.

- **Parameters**

  - `joint_id`: (`int`) 1-4
  - `encoder`: 0 ~ 4096
  - `speed`: 1 ~ 100

#### `get_encoders()`

- **function**: Get the six joints of the manipulator.

- **Return value:** (`list`) the list of encoders

### 5. Running status and Settings

#### `get_speed()`

- **function:** Get speed

- **Return value**：`int` joint speed

#### `set_speed(speed)`

- **function:** Set speed
- **Parameters:**
  - `speed` : (int) 1-100
- **Return value**：`float` Angle value

#### `get_joint_min_angle(joint_id)`

- **function:** Gets the minimum movement angle of the specified joint
- **Parameters:**
  - `joint_id` : Enter joint ID (range 0-3)
- **Return value**：`float` Angle value

#### `get_joint_max_angle(joint_id)`

- **function:** Gets the maximum movement angle of the specified joint
- **Parameters:**
  - `joint_id` : Enter joint ID (range 0-3)
- **Return value:** `float` Angle value

#### `set_joint_min(id, angle)`

- **function:** Set minimum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 0-3)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be less than the minimum value

#### `set_joint_max(id, angle)`

- **function:** Set maximum joint angle limit
- **Parameters:**
  - `id` : Enter joint ID (range 0-3)
  - `angle`: Refer to the limit information of the corresponding joint in the [send_angle()](#send_angleid-degree-speed) interface, which must not be greater than the maximum value
  
### 6. Joint motor control

#### `is_servo_enable(servo_id)`

- **function:** Detecting joint connection status
- **Parameters:** `servo id` 1-6
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `is_all_servo_enable()`

- **function:** Detect the status of all joint connections
- **Return value:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `set_servo_calibration(servo_id)`

- **function:** The current position of the calibration joint actuator is the angle zero point
- **Parameters**:
  - `servo_id`: 1 - 4

#### `release_servo(servo_id)`

- **function:** Set the specified joint torque output to turn off
- **Parameters**:
  - `servo_id`: 1 ~ 4
- **Return value:**
  - `1`: release successful
  - `0`: release failed
  - `-1`: error

#### `focus_servo(servo_id)`

- **function**: Set the specified joint torque output to turn on
- **Parameters**: `servo_id`: 1 ~ 4
- **Return value:**
  - `1`: focus successful
  - `0`: focus failed
  - `-1`: error

#### `set_servo_data(servo_id, data_id,  value, mode=None）`

- **function:** Set the data parameters of the specified address of the steering gear
- **Parameters**：
  - `servo_id`: (`int`) joint id 1 - 4
  - `data_id`: (`int`) Data address
  - `value`: (`int`) 0 - 4096
  - `mode`: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.

#### `get_servo_data(servo_id, data_id, mode=None）`

- **function:** Read the data parameter of the specified address of the steering gear.
- **Parameters**：
  - `servo_id`: (`int`) joint id 1 - 4
  - `data_id`: (`int`) Data address
  - `mode`: 0 - indicates that value is one byte(default), 1 - 1 represents a value of two bytes.
- **Return value:** 0 ~ 4096

#### `joint_brake(joint_id）`

- **function:** Make it stop when the joint is in motion, and the buffer distance is positively related to the existing speed
- **Parameters**：
  - `joint_id`: (`int`) joint id 1 - 4

### 7. Robotic arm end IO control

#### `set_color(r, g, b)`

- **function**: Set the color of the end light of the robotic arm

- **Parameters**:

  - `r (int)`: 0 ~ 255

  - `g (int)`: 0 ~ 255

  - `b (int)`: 0 ~ 255
  
#### `set_digital_output(pin_no, pin_signal)`

- **function:** set IO statue
- **Parameters**
  - `pin_no` (int): Pin number
  - `pin_signal` (int): 0 / 1

#### `get_digital_input(pin_no)`

- **function:** read IO statue
- **Parameters**: `pin_no` (int)
- **Return value**: signal

#### `set_pin_mode(pin_no, pin_mode)`

- **function:** Set the state mode of the specified pin in atom.
- **Parameters**
  - `pin_no` (int): Pin number
  - `pin_mode` (int): 0 - input, 1 - output, 2 - input_pullup

### 8. Robotic arm end gripper control

#### `set_gripper_state(flag, speed, _type_1=None)`

- **function**: Adaptive gripper enable

- **Parameters**:

  - `flag (int) `: 0 - open 1 - close, 254 - release

  - `speed (int)`: 1 ~ 100

  - `_type_1 (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **function**: Set the gripper value

- **Parameters**:

  - `gripper_value (int) `: 0 ~ 100

  - `speed (int)`: 1 ~ 100

  - `gripper_type (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_calibration()`

- **function**: Set the current position of the gripper to zero

#### `is_gripper_moving()`

- **function**: Judge whether the gripper is moving or not
- **Return value**：
  - `0`: not moving
  - `1`: is moving
  - `-1`: error data

#### `get_gripper_value()`

- **function**: Get the value of gripper
- **Parameters**:
  - `gripper_type`: (int) default 1
    - 1: Adaptive gripper
    - 3: Parallel gripper
    - 4: Flexible gripper
- **Return value**：gripper value (int)

### 9. Set bottom IO input/output status

#### `set_basic_output(pin_no, pin_signal)`

- **function**：Set Base IO Output
- **Parameters**：
  - `pin_no` (`int`) Pin port number
  - `pin_signal` (`int`): 0 - low. 1 - high

#### `get_basic_input(pin_no)`

- **function:** Read base IO input
- **Parameters:**
  - `pin_no` (`int`) pin number
- **Return value:** 0 - low. 1 - high

### 10. WLAN Setting

#### `set_ssid_pwd(account, password)`

- **function:** Change connected wifi. (Apply to m5)
- **Parameters:**
  - `account` (`str`) new wifi account
  - `password` (`str`) new wifi password

#### `get_ssid_pwd()`

- **function:** Get connected wifi account and password. (Apply to m5)
- **Return value:** (account, password)

#### `set_server_port(port)`

- **function:** Change the connection port of the server
- **Parameters:**
  - `port` (`int`) The new connection port of the server.

### 11. TOF

#### `get_tof_distance()`

- **function:** Get the detected distance (Requires external distance detector)
- **Return value:** (int) The unit is mm.

### 12. Raspberry pi -- GPIO

#### `gpio_init()`

- **function**: Init GPIO module, and set BCM mode.

#### `gpio_output(pin, v)`

- **function**: Set GPIO port output value.

- **Parameters**

  - `pin` (`int`) Pin number.
  - `v` (`int`): 0 / 1

### 13. utils (module)

This module supports some helper methods. Use the code entered at the beginning of the file to import the module:

```python
from pymycobot import utils
```

#### `utils.get_port_list()`

- **Function**: Get a list of all current serial port numbers

- **Return value:** Serial port list (`list`)

#### `utils.detect_port_of_basic()`

- **Function**: Return the first detected serial port number of M5 Basic. (Only one serial port number will be returned)

- **Return value:** Return the detected port number. If no serial port number is detected, it will return: None

## MyPalletizer 260 Socket

> Note:
> raspberryPi version Only supports python3
> The robotic arm that uses this class of premise has a server and has been turned on.

Use TCP/IP to control the robotic arm

### Client

```python
# demo
from pymycobot import MyPalletizerSocket
# Port 9000 is used by default
mc = MyPalletizerSocket("192.168.10.10",9000)

res = mc.get_angles()
print(res)

mc.send_angles([0,0,0,0],20)
...
```

### Server

Server file is in the `demo folder`,For details, please check the [Server_260.py](../demo/Server_260.py) file in the demo folder

### socket control

> Note:
> Most of the methods are the same as the class MyPalletizer260, only the new methods are listed here.


#### `set_gpio_mode(mode)`

- **function**: Set pin coding method.

- **Parameters**

  - `mode` (`str`) "BCM" or "BOARD".

#### `set_gpio_out(pin_no, mode)`

- **function**: Set the pin as input or output.

- **Parameters**

  - `pin_no` (`int`) pin id.
  - `mode` (`str`) "in" or "out"

#### `set_gpio_output(pin_no, state)`

- **function**: Set the pin to high or low level.

- **Parameters**

  - `pin_no` (`int`) pin id.
  - `state` (`int`) 0 or 1

#### `get_gpio_in(pin_no)`

- **function**: Get pin level status.

- **Parameters**

  - `pin_no` (`int`) pin id.
