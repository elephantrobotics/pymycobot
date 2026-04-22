## L1 Chassis Basic Control Interfaces

### 1. System & Product Information

#### get_system_version()

- **Function:** Retrieves the main firmware version number.
- **Return Value:**
  - **float: version**

#### get_modify_version()

- **Function:** Retrieves the minor firmware version number.
- **Return Value:**
  - **int: version**

#### power_on()

- **Function:** Powers on the robot.
- **Return Value:**
  - **int: Power-on result; 1: Success, 0: Failure**

#### power_on_only()

- **Function:** Powers on the robot, but does not start the control program.
- **Return Value:**
  - **int: Power-on-only result; 1: Success, 0: Failure**

#### power_off()

- **Function:** Powers off the robot.
- **Return Value:**
  - **int: Power-off result; 1: Success, 0: Failure**

#### is_power_on()

- **Function:** Checks whether the robot is powered on.
- **Return Value:**
  - **int: Power status; 1: On, 0: Off**

### 2. Motion Control

#### move_backward(speed)

- **Function:** Translates the robot backward.
- **Parameters:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### move_forward(speed)

- **Function:** Translates the robot forward.
- **Parameters:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### turn_left(speed)

- **Function:** Rotates to the left.
- **Parameters:**
  - **speed:**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### turn_right(speed)

- **Function:** Rotates to the right.
- **Parameters:**
  - **speed:**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### stop()

- **Function:** Stops movement
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### set_auto_report_state(state)

- **Function:** Sets the automatic reporting state
- **Parameters:**
  - **state(int): 0: Off, 1: On**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### get_auto_report_state()

- **Function:** Gets the automatic reporting state
- **Return Value:**
  - **int: 0: Off, 1: On**

#### get_auto_report_message()

- **Function:** Gets the automatic reporting message
- **Return Value:**
  - **list[int | list[int] | float]:**
    - **0 - (float) rx**
    - **1 - (float) ry**
    - **2 - (float) rw**
    - **3 - (list[int]) Machine Status**
    - **4 - (list[int]) Motor Information**
    - **5 - (float) Battery Voltage**
    - **6 - (int) Motor Enable Status (0: Enabled, 1: Disabled)**

### 3. Motor Assistance

#### get_motor_enable_status()

- **Function:** Gets the motor enable status
- **Return Value:**
  - **list[int]: Motor Enable Status**
    - **0: Disabled**
    - **1: Enabled**

#### get_motor_status()

- **Function:** Gets the motor status
- **Return Value:**
  - **list[int]: Motor Status**
    - **0: Normal**
    - **any: Error Code**

#### get_motor_temps()

- **Function:** Gets motor temperatures
- **Return Value:**
  - **list[float]: Motor Temperatures**

#### get_motor_speeds()

- **Function:** Gets motor speeds
- **Return Value:**
  - **list[float]: Motor Speeds**

#### get_motor_torques()

- **Function:** Retrieves motor torques.
- **Return Value:**
  - **list[float]: Motor torques**

#### set_communication_state(state)

- **Function:** Sets the communication state.
- **Parameters:**
  - **state(int):**
    - **0: Serial Communication (Default)**
    - **1: Socket Communication**
    - **2: Bluetooth Communication (Writes the MAC address to a file and endpoint, then returns the status)**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### get_communication_state()

- **Function:** Retrieves the current communication state.
- **Return Value:**
- **int: Communication state**
- **0: Serial Communication**
- **1: Socket Communication**
- **2: Bluetooth Communication**

#### set_led_color(position, brightness, color)

- **Function:** Sets the LED color.
- **Parameters:**
  - **position(int):**
    - **0: Left LED**
    - **1: Right LED**
  - **color(tuple(int, int, int)): RGB color**
  - **brightness(int): 0 - 255 (Default: 255)**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### get_motor_loss_count()

- **Function:** Retrieves the motor step-loss count.
- **Return Value:**
- **list[int]: Motor step-loss counts**

### 4. IO Control

#### get_pin_input(pin)

- **Function:** Get input IO state
- **Parameters:**
  - **pin(int): 1 - 6**
- **Return Value:**
  - **int: 0: Low level, 1: High level, -1: Invalid pin**

#### set_pin_output(pin, state)

- **Function:** Set output IO state
- **Parameters:**
  - **pin(int): 1 - 6**
  - **state(int): 0: Low level, 1: High level**
- **Return Value:**
  - **int: 1: Success, 0: Failure**

#### get_estop_state()

- **Function:** Get emergency stop (E-stop) state
- **Return Value:**
  - **int: 0: Released, 1: Pressed**

### 5. WiFi & Bluetooth

#### get_wifi_ip_port()

- **Function:** Get WiFi IP address and port
- **Return Value:**
  - **tuple(str, int): WiFi IP address, WiFi port**

#### get_wifi_account()

- **Function:** Get WiFi account credentials
- **Return Value:**
  - **tuple(str, str): WiFi SSID, WiFi password**

#### get_bluetooth_address()

- **Function:** Get Bluetooth MAC address
- **Return Value:**
  - **str: Bluetooth MAC address**

#### get_bluetooth_uuid()

- **Function:** Get Bluetooth UUIDs
- **Return Value:**
  - **tuple(str, str, str): Bluetooth name, Service UUID, Characteristic UUID**

### Use Cases

#### Get Chassis System Version

```python
from pymycobot import MercuryL1Chassis

# Initialize Mercury L1 Chassis object
mlc = MercuryL1Chassis("/dev/ttyTHS1", baudrate=1000000, debug=True)

# Get system version number
version = mlc.get_system_version()
print(version)
```

#### Controlling Chassis Movement

```python
import time
from pymycobot import MercuryL1Chassis

# Initialize the Mercury L1 Chassis object
mlc = MercuryL1Chassis("/dev/ttyTHS1", baudrate=1000000, debug=True)

# Control the Mercury L1 Chassis to move forward at a speed of 0.5 m/s
mlc.move_forward(0.5)

# Sleep for 3 seconds
time.sleep(3)

# Stop moving
mlc.stop()
```

---