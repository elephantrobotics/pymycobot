# MyArm

[toc]
## API usage instructions

> API (Application Programming Interface), also known as Application Programming Interface functions, are predefined functions. When using the following Description interfaces, please import our API library at the beginning by entering the following code, otherwise it will not run successfully:

```python
# Example
from pymycobot import MyArmM

mam = MyArmM('/dev/ttyAMA1')

# Gets the current angle of all joints
angles = mam.get_joints_angle()
print(angles)

# Set 1 joint to move to 40 and speed to 20
mam.set_joint_angle(1, 40, 20)
```

## MyArmAPI(M&C)

> This interface is shared by `MyArmM` and `MyArmC`
 
### 1. Robot status


#### `get_robot_modified_version()`

- **Description:** Get the bot correction version number

- **Returns:**

  - **version (int): the bot correction version number**

#### `get_robot_firmware_version()`

- **Description:** Obtaining the Robot Firmware Version (Major and Minor Versions)

- **Returns:**

  - **version (int): the robot firmware**

#### `get_robot_tool_modified_version()`

- **Description:** Get the remediation version of the bot tool


#### `get_robot_tool_firmware_version()`

- **Description:** Get the Robot Tool Firmware Version (End Atom)

#### `set_robot_err_check_state(status)`

- **Description:** Set Error Detection Status You can turn off error detection, but do not turn it off unless necessary

- **Parameters:**

  - **status (int): 1 open; o close**

#### `get_robot_err_check_state()`

- **Description:** Read error detection status

#### `get_robot_error_status()`

- **Description:** Get the bot error status

- **Returns:**

  - **No error: [0,0,0,0,0,0,0,0]**

  - **assuming that error 1 and 3 are reported in section 1, it should return: [[1,3],0,0,0,0,0,0,0]**

#### `get_robot_power_status()`

- **Description:** Get the robot power status

- **Returns:**

  - **power_status (int): 0: power off, 1: power on**

#### `set_robot_power_on()`

- **Description:** Set the robot to power on

  - **Returns: (int) 1**

#### `set_robot_power_off()`

- **Description:** Set the robot to power off

  - **Returns: (int) 1**

#### `clear_robot_err()`

- **Description:** Clear the robot abnormality Ignore the error joint and continue to move

#### `set_recv_queue_max_len(max_len)`

- **Description:** Set the total length of the receiving command queue

#### `get_recv_queue_max_len()`

- **Description:** The total length of the read command queue, the default length is 100

#### `clear_recv_queue()`

- **Description:** Clear the queue for receiving commands

#### `get_recv_queue_len()`

- **Description:** The current length of the read receive queue

### 2. Joint servo control

#### `get_joint_angle(joint_id)`

- **Description:** Gets the current angle of the specified joint

- **Parameters:**

  - **joint_id (int): 0 - 254**

#### `get_joints_angle()`

- **Description:** Gets the current angle of all joints

- **Returns:**

  - **angles list(int): 0 - 254**

#### `get_joints_max()`

- **Description:** Read the maximum angle of all joints

#### `get_joints_min()`

- **Description:** Read the minimum angle of all joints

### 3. Servo motor control

#### `set_servo_calibrate(servo_id)`

- **Description:** Sets the zero position of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

#### `get_servo_encoder(servo_id)`

- **Description:** Gets the current encoder potential value for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

- **Returns:**

  - **encoder (int): 0-4095**

#### `get_servos_encoder()`

- **Description:** Obtain the current encoder potential values for multiple servo motors

#### `get_servos_speed()`

- **Description:** Gets the current movement speed of multiple servo motors

#### `is_all_servos_enabled()`

- **Description:** Get the connection status of multiple servo motors

- **Returns:**

  - **status: list[int*8] 0: The connection failed 1: The connection is successful**

#### `get_servos_temp()`

- **Description:** Obtain the temperature of multiple servo motors

#### `get_servos_voltage()`

- **Description:** Get the voltage of multiple servo motors

#### `get_servos_current()`

- **Description:** Obtain the current of multiple servo motors

#### `get_servos_status()`

- **Description:** Get all the statuses of multiple servo motors

#### `get_servos_protect_current()`

- **Description:** Obtain multiple servo motor protection currents

#### `set_servo_enabled(joint_id, state)`

- **Description:** Set the servo motor torque switch

- **Parameters:**

  - **joint_id (int): 0-254 254-all**

  - **state: 0/1**
    - **1: focus**
    - **0: release**

### 4. Servo motor system parameter modification

#### `set_servo_p(servo_id, data)`

- **Description:** Sets the proportionality factor of the position loop P of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0-254**

  - **data (int): 0-254**

#### `get_servo_p(servo_id)`

- **Description:** Reads the position loop P scale factor of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0-254**

#### `set_servo_i(servo_id, data)`

- **Description:** Set the proportional factor of the position ring I of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 254**

#### `get_servo_i(servo_id)`

- **Description:** Reads the position loop I scale factor of the specified servo motor


#### `set_servo_d(servo_id, data)`

- **Description:** Sets the proportional factor for the position ring D of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0-254**

  - **data (int): 0-254**

#### `get_servo_d(servo_id)`

- **Description:** Reads the position ring D scale factor for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0-254**

#### `set_servo_cw(servo_id, data)`

- **Description:** Sets the clockwise insensitivity zone of the encoder for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 32**
  
#### `get_servo_cw(servo_id)`

- **Description:** Reads the clockwise insensitive area of the encoder for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

#### `set_servo_cww(servo_id, data)`

- **Description:** Sets the counterclockwise insensitive zone of the encoder for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 32**

#### `get_servo_cww(servo_id)`

- **Description:** Reads the counterclockwise insensitive area of the encoder of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

#### `set_servo_system_data(servo_id, addr, data, mode)`

- **Description:** Set the system parameters for the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254:**

  - **addr (int):**

  - **data (int): 0 - 4096**

  - **mode (int): 1 - data 1byte. 2 - data 2byte**

#### `get_servo_system_data(servo_id, addr, mode)`

- **Description:** Read the system parameters of the specified servo motor

- **Parameters:**

  - **servo_id (int): 0 - 254**

  - **addr (int):**

  - **mode (int): 1 - data 1byte. 2 - data 2byte**

### 5. IO control

#### `set_master_out_io_state(io_number, status)`

- **Description:** Set the host I/O pin status

- **Parameters:**

  - **io_number: 1 - 2**

  - **status: 0/1; 0: low; 1: high. default: 1**

#### `get_master_in_io_state(io_number)`

- **Description:** get the host I/O pin status

- **Parameters:**

  - **io_number (int): 1 - 2**

- **Returns:**

  - **0 or 1. 1: high 0: low**

#### `set_tool_out_io_state(io_number, status)`

- **Description:** Set the Atom pin status

- **Parameters:**

  - **io_number (int): 1 - 2**

  - **status: 0 or 1; 0: low; 1: high. default: 1**

#### `get_tool_in_io_state(io_number)`

- **Description:** Get the Atom pin status

- **Parameters:**

  - **io_number (int): pin number**

- **Returns:**

  - **0 or 1. 1: high 0: low**

### 6. Atom control

#### `set_tool_led_color(r, g, b)`

- **Description:** Set the Atom LED color

- **Parameters:**

  - **r: 0-255**

  - **g: 0-255**

  - **b: 0-255**

#### `is_tool_btn_clicked()`

- **Description:** read the atom press status

- **Returns:**

  - **int: 0 or 1. 1: press**
  
# MyArmM

> Only `MyArmM` interfaces are available

### 1. Joint servo control

#### `set_joint_angle(joint_id, angle, speed)`

- **Description:** Sets the individual joints to move to the target angle

- **Parameters:**

  - **joint_id (int) : 0 - 254**

  - **angle (int) : 0 - 254**

  - **speed (int) : 1 - 100**

#### `set_joints_angle(angles, speed)`

- **Description:** Sets all joints to move to the target angle

- **Parameters:**

  - **angles (list[int]):  0 - 254**

  - **speed (int): 0 - 100**

#### `is_robot_moving()`

- **Description:** See if the robot is moving

- **Returns:**

  - **1: moving**

  - **0: not moving**

#### `stop_robot()`

- **Description:** The robot stops moving

### 2. Servo motor control

#### `set_servo_encoder(servo_id, encoder, speed)`

- **Description:** Sets the individual motor motion to the target encoder potential value

- **Parameters:**

  - **servo_id: (int) 0 - 254**

  - **encoder: (int) 0 - 4095**

  - **speed: (int) 1 - 100**

#### `set_servos_encoder(positions, speed)`

- **Description:** Set the encoder potential value for multiple motors moving to the target

- **Parameters:**

  - **positions (list[int * 8]): 0 - 4095:**

  - **speed (int): 1 - 100:**

#### `set_servos_encoder_drag(encoders, speeds)`

- **Description:** Set multiple servo motors with a specified speed to the target encoder potential value

### 3. IO control
#### `get_assist_in_io_state(io_number)`

- **Description:** Get the auxiliary pin status

- **Parameters:**

  - **io_number (int): 1 - 6**

- **Returns:**

  - **0 or 1. 1: high 0: low**

#### `set_assist_out_io_state(io_number, status)`

- **Description:** Set the auxiliary pin status

- **Parameters:**

  - **io_number: 1 - 6**

  - **status: 0/1; 0: low; 1: high. default: 1**
