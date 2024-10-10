# Mercury X1 Chassis

---

- **Prerequisites**
  - First, open a console terminal and run the following command to start chassis communication.
  
  ```bash
  roslaunch turn_on_mercury_robot mapping.launch
  ```

- **API Usage Instructions**
  >> API (Application Programming Interface), also known as application programming interface function, is a predefined function. When using the following function interface, please enter the following code at the beginning to import our API library, otherwise it will not run successfully.

  ```python
  # demo
  from pymycobot import mercurychassis_api

  mc = ChassisControl('/dev/wheeltec_controller')

  # Get battery voltage
  voltage = mc.get_power_voltage()
  print(voltage)

  # Set the chassis RGB light strip to green
  mc.set_color(255, 0, 0)
  ```

## get_power_voltage()

- Function: Get the battery voltage.
- Return value: Battery voltage (floating point type), unit: Volt

## get_ultrasonic_value()

- Function: Get all ultrasonic data.
- Return value: List of all ultrasonic data, unit: millimeter.

## go_straight(speed)

- Function: Control the chassis to move forward.
- Parameters:
  - speed: (floating point type) the speed of chassis movement, the range is `0 ~ 1`, unit: meters per second.

## go_back(speed)

- Function: Control chassis to move backward.
- Parameters:
  - speed: (floating point type) chassis movement speed, range is `-1 ~ 0`, unit: meter per second.

## turn_left(speed)

- Function: Control the chassis to rotate to the left.
- Parameters:
  - speed: (floating point type) the speed of chassis movement, ranging from `0 ~ 1`, unit: meters per second.

## turn_right(speed)

- Function: Control chassis to rotate right.
- Parameters:
  - speed: (floating point type) chassis movement speed, range is `-1 ~ 0`, unit: meters per second.

## stop()

- Function: Stop motion.

## set_color(r, g, b)

- Function: Set the color of the chassis RGB light strip.
- Parameters:
  - r (int): 0 ~ 255
  - g (int): 0 ~ 255
  - b (int): 0 ~ 255

