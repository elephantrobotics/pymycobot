# class MyAGVPro()

### 1. System & Product Information

#### get_system_version():
- **function:** Obtain the major firmware version number
- **Return value:**
  - **float: version**

#### get_modify_version():
- **function:** Obtain the minor firmware version number
- **Return value:**
  - **int: version**

#### power_on():
- **function:** Turn on the robot
- **Return value:**
  - **int: Power-on result, 1: Success, 0: Failed**

#### power_on_only():
- **function:** Turn on the robot, but not start the control program
- **Return value:**
  - **int: Power-on result only, 1: Success, 0: Failed**

#### power_off():
- **function:** Turn off the robot
- **Return value:**
  - **int: Power-off result only, 1: Success, 0: Failed**

#### is_power_on():
- **function:** Check if the robot is powered on
- **Return value:**
  - **int: power state, 1: Power-on, 0: Power-off**

### 2. Motion control

#### move_backward(speed):
- **function:** Pan the robot backward
- **parameter:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### move_forward(speed):
- **function:** Pan the robot forward
- **parameter:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### move_left_lateral(speed):
- **function:** Pan the robot left
- **parameter:**
  - **speed(float): 0 ~ 1 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### move_right_lateral(speed):
- **function:** Pan the robot right
- **parameter:**
  - **speed(float): 0 ~ 1 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### turn_left(speed):
- **function:** Rotate to the left
- **parameter:**
  - **speed:**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### turn_right(speed):
- **function:** Rotate to the right
- **parameter:**
  - **speed:**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### stop():
- **function:** Stop moving
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### set_auto_report_state(state):
- **function:** Set the auto-report state
- **parameter:**
  - **state(int): 0: Close, 1: Open**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### get_auto_report_state():
- **function:** Get the auto-report state
- **Return value:**
  - **int: 0: Close, 1: Open**

#### get_auto_report_message():
- **function:** Get the auto-report message
- **Return value:**
  - **list[int | list[int] | float]:**
  - **0 - (float)rx**
  - **1 - (float)ry**
  - **2 - (float)rw**
  - **3 - (list[int])Machine status**
  - **4 - (list[int])Motor information**
  - **5 - (float)Battery voltage**
  - **6 - (int)Motor enable status 0: Enabled, 1: Disabled**

### 3. Motor assisted

#### get_motor_enable_status():
- **function:** Get the motor enabled status
- **Return value:**
  - **list[int]: Motor enabled status**
  - **0: Disable**
  - **1: Enable**

#### get_motor_status():
- **function:** Get the motor status
- **Return value:**
  - **list[int]: Motor status**
  - **0: normal**
  - **any: error code**

#### get_motor_temps():
- **function:** Get the motor temperature
- **Return value:**
  - **list[float]: Motor temperature**

#### get_motor_speeds():
- **function:** Get the motor speeds
- **Return value:**
  - **list[float]: Motor speeds**

#### get_motor_torques():
- **function:** Get the motor torques
- **Return value:**
  - **list[float]: Motor torques**

#### set_communication_state(state):
- **function:** Set the communication state
- **parameter:**
  - **state(int):**
  - **0: Serial communication (default)**
  - **1: Socket communication**
  - **2: Bluetooth communication (Write the MAC address to the file and the endpoint, and then return to the state)**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### get_communication_state():
- **function:** Get the communication state
- **Return value:**
  - **int: communication state**
  - **0: Serial communication,**
  - **1: Socket communication,**
  - **2: Bluetooth communication**

#### set_led_color(position, brightness, color):
- **function:** Set the LED color
- **parameter:**
  - **position(int):**
  - **0: Left LED**
  - **1: Right LED**
  - **brightness(int): 0 - 255**
  - **color(tuple(int, int, int)): RGB color**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### get_motor_loss_count():
- **function:** Get the motor loss count
- **Return value:**
  - **list[int]: Motor loss count**

### 4. IO Control

#### get_pin_input(pin):
- **function:** Get the input IO
- **parameter:**
  - **pin(int): 1, 2, 3, 4, 5, 6, 7, 8, 254**
- **Return value:**
  - **int: 0: Low, 1: High, -1: There is no such pin**

#### set_pin_output(pin, state):
- **function:** Set the output IO
- **parameter:**
  - **pin(int): 1 - 6**
  - **state(int): 0: Low, 1: High**
- **Return value:**
  - **int: 1: Success, 0: Failed**

### 5. WiFi & Bluetooth

#### get_wifi_ip_port():
- **function:** Get the wi-fi ip and port
- **Return value:**
  - **tuple(str, int): wi-fi ip, wi-fi port**

#### get_wifi_account():
- **function:** Get the wi-fi account
- **Return value:**
  - **tuple(str, str): wi-fi account, wi-fi password**

#### get_bluetooth_address():
- **function:** Get the bluetooth MAC address
- **Return value:**
  - **str: bluetooth MAC address**

#### get_bluetooth_uuid():
- **function:** Get the bluetooth uuid
- **Return value:**
  - **tuple(str, str, str): bluetooth name, service uuid, characteristic uuid**

### 6. Use Cases:
#### 6.1 Get the system version number of AGVPro
```python
from pymycobot import MyAGVPro

# Initialize the AGVPro object
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)

# Obtain the system version number
version = agv_pro.get_system_version()
print(version)
```
#### 6.2 Control AGVPro to move forward at a speed of 0.5ms for 3 seconds
```python
import time
from pymycobot import MyAGVPro

# Initialize the AGVPro object
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)

# The control AGVPro moves forward at a speed of 0.5ms
agv_pro.move_forward(0.5)

# Sleep for 3 seconds
time.sleep(3)

# Stop moving
agv_pro.stop()
```