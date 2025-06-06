# class MyAGVPro()

### 1. System & Product Information

#### def get_system_version(self) -> None:
- **function:** Obtain the major firmware version number
- **Return value:**
  - **float: version**

#### def get_modify_version(self) -> None:
- **function:** Obtain the minor firmware version number
- **Return value:**
  - **int: version**

#### def power_on(self) -> None:
- **function:** Turn on the robot
- **Return value:**
  - **int: Power-on result, 1: Success, 0: Failed**

#### def power_on_only(self) -> None:
- **function:** Turn on the robot, but not start the control program
- **Return value:**
  - **int: Power-on result only, 1: Success, 0: Failed**

#### def power_off(self) -> None:
- **function:** Turn off the robot
- **Return value:**
  - **int: Power-off result only, 1: Success, 0: Failed**

#### def is_power_on(self) -> None:
- **function:** Check if the robot is powered on
- **Return value:**
  - **int: power state, 1: Power-on, 0: Power-off**

### 2. Motion control

#### def move_backward(self, speed) -> None:
- **function:** Pan the robot backward
- **parameter:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def move_forward(self, speed) -> None:
- **function:** Pan the robot forward
- **parameter:**
  - **speed(float): 0 ~ 1.5 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def move_left_lateral(self, speed) -> None:
- **function:** Pan the robot left
- **parameter:**
  - **speed(float): 0 ~ 1 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def move_right_lateral(self, speed) -> None:
- **function:** Pan the robot right
- **parameter:**
  - **speed(float): 0 ~ 1 m/s**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def turn_left(self, speed) -> None:
- **function:** Rotate to the left
- **parameter:**
  - **speed:**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def turn_right(self, speed) -> None:
- **function:** Rotate to the right
- **parameter:**
  - **speed:**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def stop(self) -> None:
- **function:** Stop moving
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def set_auto_report_state(self, state) -> None:
- **function:** Set the auto-report state
- **parameter:**
  - **state(int): 0: Close, 1: Open**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def get_auto_report_state(self) -> None:
- **function:** Get the auto-report state
- **Return value:**
  - **int: 0: Close, 1: Open**

#### def get_auto_report_message(self) -> None:
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

#### def get_motor_enable_status(self) -> None:
- **function:** Get the motor enabled status
- **Return value:**
  - **list[int]: Motor enabled status**
  - **0: Disable**
  - **1: Enable**

#### def get_motor_status(self) -> None:
- **function:** Get the motor status
- **Return value:**
  - **list[int]: Motor status**
  - **0: normal**
  - **any: error code**

#### def get_motor_temps(self) -> None:
- **function:** Get the motor temperature
- **Return value:**
  - **list[float]: Motor temperature**

#### def get_motor_speeds(self) -> None:
- **function:** Get the motor speeds
- **Return value:**
  - **list[float]: Motor speeds**

#### def get_motor_torques(self) -> None:
- **function:** Get the motor torques
- **Return value:**
  - **list[float]: Motor torques**

#### def set_communication_state(self, state) -> None:
- **function:** Set the communication state
- **parameter:**
  - **state(int):**
  - **0: Serial communication (default)**
  - **1: Socket communication**
  - **2: Bluetooth communication (Write the MAC address to the file and the endpoint, and then return to the state)**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def get_communication_state(self) -> None:
- **function:** Get the communication state
- **Return value:**
  - **int: communication state**
  - **0: Serial communication,**
  - **1: Socket communication,**
  - **2: Bluetooth communication**

#### def set_led_color(self, position, brightness, color) -> None:
- **function:** Set the LED color
- **parameter:**
  - **position(int):**
  - **0: Left LED**
  - **1: Right LED**
  - **brightness(int): 0 - 255**
  - **color(tuple(int, int, int)): RGB color**
- **Return value:**
  - **int: 1: Success, 0: Failed**

#### def get_motor_loss_count(self) -> None:
- **function:** Get the motor loss count
- **Return value:**
  - **list[int]: Motor loss count**

### 4. IO Control

#### def get_pin_input(self, pin) -> None:
- **function:** Get the input IO
- **parameter:**
  - **pin(int): 1, 2, 3, 4, 5, 6, 7, 8, 254**
- **Return value:**
  - **int: 0: Low, 1: High, -1: There is no such pin**

#### def set_pin_output(self, pin, state) -> None:
- **function:** Set the output IO
- **parameter:**
  - **pin(int): 1 - 6**
  - **state(int): 0: Low, 1: High**
- **Return value:**
  - **int: 1: Success, 0: Failed**

### 5. WiFi & Bluetooth

#### def get_wifi_ip_port(self) -> None:
- **function:** Get the wi-fi ip and port
- **Return value:**
  - **tuple(str, int): wi-fi ip, wi-fi port**

#### def get_wifi_account(self) -> None:
- **function:** Get the wi-fi account
- **Return value:**
  - **tuple(str, str): wi-fi account, wi-fi password**

#### def get_bluetooth_address(self) -> None:
- **function:** Get the bluetooth MAC address
- **Return value:**
  - **str: bluetooth MAC address**

#### def get_bluetooth_uuid(self) -> None:
- **function:** Get the bluetooth uuid
- **Return value:**
  - **tuple(str, str, str): bluetooth name, service uuid, characteristic uuid**