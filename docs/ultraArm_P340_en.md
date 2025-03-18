# Detailed Description of API Method

**When using the following function interfaces, please import our API library at the beginning, otherwise the operation cannot succeed, that is, enter the following code:**

```python
#For Mira Series
from pymycobot.ultraArmP340 import ultraArmP340
```

**Note：** If our API library is not installed, please refer to  [Python environment setup](../4-Python/1-InstallingthePythonEnvironment.md) section to install。

## 1 Overall operation state of mechanical arm

**1.1** `go_zero()`

- **Function：** The mechanical arm returns to zero.
- **Return value:** None

**1.2** `power_on()`

- **Function：** All joints of the mechanical arm are powered on.
- **Return value:** None

**1.3** `release_all_servos()`	   

- **Function:** Power failure of all joints of mechanical arm
- **Return value:** None

**1.4** `is_moving_end()`

- **Function:** Robot movement end flag
- **Return value:**
  - `1`: Movement end
  - `0`: Movement not end

**1.5** `set_system_value(id, address, value, mode=None)`

- **Function:** Set system parameters
- **Parameter description:**
  - `id`: `int`, Motor ID, 4 or 7
  - `address`: `int`, Parameter register address, 7 ~ 69
  - `value`: `int`, Corresponding register parameter value
  - `mode`: `int`, 1 or 2, can be empty, default mode is 1
    - `1`: setting range is 0-255, address 21 (P value) can be used 
    - `2`: setting value range is 0-65535, address 56 (setting position) can be used
- **Return value:** None

**1.6** `get_system_value(id, address,mode=None)`

- **Function:** Read system parameters
- **Parameter description:**
  - `id`: `int`, Motor ID, 4 or 7
  - `address`: `int`, Parameter register address, 0 ~ 69
  - `mode`: `int`, 1 or 2, can be empty, default mode is 1
    - `1`: read range is 0-255, address 21 (P value) can be used 
    - `2`: read value range is 0-65535, address 56 (read position) can be used
- **Return value:** `int`, Corresponding register parameter value

**1.7** `get_system_version()`

- **Function:** Read the firmware major and minor versions

- **Return value:** `float`, firmware version number

**1.8** `get_modify_version()`

- **Function:** Read the firmware modified version number

- **Return value:** `int`, modified version number

## 2 Input program control mode (MDI mode)

  > Note：The limit of different series of mechanical arms is different, and the attitude value can be set is also different. For details, see the parameter introduction of the corresponding model

**2.1** `get_angles_info()`

- **Function:** Get the current angle of the mechanical arm.
- **Return value:** `list` a list of floating point values representing the angles of all joints

**2.2** `set_angle(id, degree, speed)`

- **Function:** Send the specified single joint motion to the specified angle
- **Parameter description:**
  - `id`: Represents the joint of the mechanical arm. The three axes have three joints, which can be represented by the number 1-3.
  - `degree`: Represents the angle of the joint

      <table>
        <tr>
            <th>Joint Id</th>
            <th>Range</th>
        </tr>
        <tr>
            <td text-align: center>1</td>
            <td>-150 ~ 170</td>
        </tr>
        <tr>
            <td>2</td>
            <td>-20 ~ 90</td>
        </tr>
        <tr>
            <td>3</td>
            <td>-5 ~ 110</td>
        </tr>
        <tr>
            <td>4（Accessories）</td>
            <td> -179 ~ 179</td>
        </tr>

    </table>

  - `speed`：It represents the speed of the mechanical arm movement, ranging from 0 to 200 (unit: mm/s).
- **Return value:** None

**2.3** `set_angles(degrees, speed)`

- **Function:**  Send all angles to all joints of the mechanical arm
- **Parameter description:**
  - `degrees`: (List[float]) include angles of all joints ,the three-axis robot has three joints, so the length is 3, and the representation method is: [20, 20, 20]
  - `speed`: It represents the speed of the mechanical arm movement, and the value range is 0-200 (unit: mm/s).
- **Return value:** None

**2.4** `get_coords_info()`

- **Function:** Get the current coordinates of the mechanical arm.
- **Return value:** `list` include list of coordinates, `θ` is Rotation angle at the end
  - Triaxial：The length is 3, which is in turn ` [x, y, z, θ]`


**2.5** `set_coord(id,coord,speed)`

- **Function:** Send a single coordinate value to the mechanical arm for movement
- **Parameter description:**
  - `id`:Represents the coordinates of the mechanical arm. The three axes have three coordinates and have a specific representation method.
    Representation of X coordinate:`"X"or"x"`.
  - `coord`: Enter the coordinate value you want to reach

      <table>
        <tr>
            <th>Coord Id</th>
            <th>Range</th>
        </tr>
        <tr>
            <td text-align: center>x</td>
            <td>-360 ~ 365.55</td>
        </tr>
        <tr>
            <td>y</td>
            <td>-365.55 ~ 365.55</td>
        </tr>
        <tr>
            <td>z</td>
            <td>-140 ~ 130</td>
        </tr>

      </table>

  - `speed`: It represents the speed of the mechanical arm movement, and the range is 0-200 (unit: mm/s).
- **Return value:** None

**2.6** `set_coords(coords, speed)`

- **Function:** Send the overall coordinates so that the head of the mechanical arm moves from the original point to the point you specify.
- **Parameter description:**
  - `coords`: 
    - Triaxial：The coordinate value of [x, y, z], with a length of 3
  - `speed`: It represents the speed of the mechanical arm movement, and the range is 0-200 (unit: mm/s).
- **Return value:** None

**2.7** `get_radians_info()`

- **Function：** Get the current radian value of the mechanical arm.
- **Return value:** `List ` list containing all joint radian values

**2.8** `set_radians(radians, speed)`

- **Function:** Send radian values to all joints of the mechanical arm
- **Parameter Description**
  - `radians`: List of radian values for each joint( `List[float]`)

      <table>
        <tr>
            <th>Joint Id</th>
            <th>Range</th>
        </tr>
        <tr>
            <td text-align: center>1</td>
            <td>2.6179 ~ 2.9670</td>
        </tr>
        <tr>
            <td>2</td>
            <td>-0.3490 ~ 1.5707</td>
        </tr>
        <tr>
            <td>3</td>
            <td>-0.0872 ~ 1.9198</td>
        </tr>
        <tr>
            <td>4 (Accessories)</td>
            <td> -3.1241 ~ + 3.1241</td>
        </tr>
      </table>

  - `speed`: It represents the speed of the mechanical arm movement, and the range is 0-200 (unit: mm/s).
- **Return value:** None

**2.9** `set_mode()`

- **Function:** Set Coordinate Mode
- **Parameter Description**
  - `0`:Absolute Cartesian mode
  - `1`:Relative Cartesian mode
- **Return value:** None

**2.10** `sleep()`

- **Function:** Delay
- **Parameter Description**
  - `Time`: Time delayed( `Int`type)，
- **Return value:** None

**2.12** `set_init_pose()`		

- **Function:** Set the current position as a fixed position,for example（0,0,0）,set this position to zero
- **Parameter Description**
  - `coords`: All coordinates of mechanical arm
  - `speed`: It represents the speed of the mechanical arm movement, and the range is 0-200 (unit: mm/s).
- **Return value:** None

**2.13** `set_pwm()`			 

- **Function:** Set PWM duty cycle
- **Parameter description:** P：Duty cycle，Range: 0-255
- **Return value:** None

**2.14** `set_fan_state()`

- **Function:** Set Fan Status
- **Parameter Description** 
  - `0`: close
  - `1`: open
- **Return value:** None


**2.15** `get_switch_state()`

- **Function:** Get limit switch status
- **Parameter Description** 
  - `X`：Whether joint 3 reaches the limit position
  - `Y`：Whether joint 2 reaches the limit position
  - `Z`：Whether joint 1 reaches the limit position
- **Return value:** Yes


**2.16** `set_speed_mode(mode)`

- **Function:** Set speed mode
- **Parameter Description** 
  - `0`: Uniform speed mode
  - `2`: Acceleration and deceleration mode
- **Return value:** None



## 3  JOG mode and operation

**3.1** `set_jog_angle(id, direction, speed)`

- **Function:** Set inching mode (angle)
- **Parameter Description**
  - `id`: Represents the joint of the mechanical arm, which is represented by 1~3 joint id
  - `direction`: It mainly controls the moving direction of the machine arm. Input 0 is negative move and input 1 is positive move
  - `speed`: Speed 0~200 (unit: mm/s)
- **Return value:** None

**3.2** `set_jog_coord(axis, direction, speed)`

- **Function:** Control the robot to move continuously according to the specified coordinates or attitude values
- **Parameter Description**
  - `axis`: Represents the joint of the mechanical arm, which is represented by the x/y/z given by the axis of the joint
  - `direction`: It mainly controls the moving direction of the machine arm. Input 0 is negative move and input 1 is positive move
  - `speed`: Speed 0~200 (unit: mm/s)
- **Return value:** None

**3.3** `set_jog_stop()`

- **Function:** Stop continuous movement under the control of jog
- **Return value:** None



## 4 IO control

**4.1** `set_gpio_state(state)`

- **Function:** Set suction pump status
- **Parameter Description**
  - `state` (int)：Input 0 indicates that the suction pump is started, and input 1 indicates that the suction pump is closed
- **Return value:** None

**4.2** `set_gripper_zero()`

- **Function:** Set gripper zero position（Set the current position to zero）
- **Return value:** None

**4.3** `set_gripper_state(state):`

- **Function:** Set gripper switch
- **Parameter Description** `state`：Input 0 to open the gripper, and input 1 to close the gripper
- **Return value:** None

**4.4** `get_gripper_angle():`

- **Function:** Get the gripper angle
- **Return value:** Gripper angle value

## 5 Functional interface

**5.1** `play_gcode_file(filename)`

- **Function:** Play imported track files
- **Parameter description:**
  - `filename` ：Track file name
- **Return value:** None


## 6 Use Cases

```python
import time
import platform
from pymycobot.ultraArmP340 import ultraArmP340

# Automatically select the system and connect the mechanical arm
if platform.system() == "Windows":
    ua = ultraArmP340('COM6', 115200)
    ua.go_zero()
elif platform.system() == "Linux":
    ua = ultraArmP340('/dev/ttyUSB0', 115200)
    ua.go_zero()
    
# Position of mechanical arm movement
angles = [
    [-81.71, 0.0, 0.0],
    [-90.53, 21.77, 47.56],
    [-90.53, 0.0, 0.0],
    [-59.01, 21.77, 45.84],
    [-59.01, 0.0, 0.0]
]

# Suck small pieces
ua.set_angles(angles[0], 50)
time.sleep(3)
ua.set_angles(angles[1], 50)
time.sleep(3)

# Start suction pump
ua.set_gpio_state(0)

ua.set_angles(angles[2], 50)
time.sleep(3)
ua.set_angles(angles[3], 50)

# Close the suction pump
ua.set_gpio_state(1)
time.sleep(3)

ua.set_angles(angles[4], 50)
```
