# myAgv

<details>
<summary>Catalogue:</summary>

- [myAgv](#myagv)
  - [set\_led(mode, R, G, B)](#set_ledmode-r-g-b)
  - [get\_firmware\_version](#get_firmware_version)
  - [get\_motors\_current](#get_motors_current)
  - [get\_battery\_info](#get_battery_info)
  - [go\_ahead(go\_speed)](#go_aheadgo_speed)
  - [retreat(back\_speed)](#retreatback_speed)
  - [pan\_left(pan\_left\_speed)](#pan_leftpan_left_speed)
  - [pan\_right(pan\_right\_speed)](#pan_rightpan_right_speed)
  - [clockwise\_rotation(rotate\_right\_speed)](#clockwise_rotationrotate_right_speed)
  - [counterclockwise\_rotation(rotate\_left\_speed)](#counterclockwise_rotationrotate_left_speed)
  - [stop()](#stop)
  - [restore()](#restore)

</details>

## set_led(mode, R, G, B)

Set up LED lights

- **Parameters**

  - **mode** – 1 - Set LED light color. 2 - Set the LED light to blink.

  - **R** – (int) 0 ~ 255

  - **G** – (int) 0 ~ 255

  - **B** – (int) 0 ~ 255

## get_firmware_version

Get firmware version number

## get_motors_current

Get the total current of the motor

## get_battery_info

Read battery information

- **Return**
  list : [battery_data, battery_1_voltage, battery_2_voltage].
  `battery_data`:A string of length 6, represented from left to right: bit5, bit4, bit3, bit2, bit1, bit0.
  bit5 : Battery 2 is inserted into the interface 1 means inserted, 0 is not inserted.
  bit4 : Battery 1 is inserted into the interface, 1 means inserted, 0 is not inserted.
  bit3 : The adapter is plugged into the interface 1 means plugged in, 0 not plugged in.
  bit2 : The charging pile is inserted into the interface, 1 means plugged in, 0 is not plugged in.
  bit1 : Battery 2 charging light 0 means off, 1 means on.
  bit0 : Battery 1 charging light, 0 means off, 1 means on.
  `battery_1_voltage` : Battery 1 voltage in volts.
  `battery_2_voltage` : Battery 2 voltage in volts.

<!-- ## move_control(direction_1, direction_2, direction_3)

Control the car to rotate forward, backward, left, right and forward/counterclockwise.

- **Parameters**

  - **direction_1** – (int) Control forward or backward: 0 ~ 127 is backward, 129 ~ 255 is forward, 128 is stop.

  - **direction_2** – (int) control left and right movement: 0 ~ 127 is right, 129 ~ 255 is left, 128 is stop.

  - **direction_3** – (int) control rotation: 0 ~ 127 is clockwise, 129 ~ 255 is counterclockwise, 128 is stop. -->

## go_ahead(go_speed)

Control the car to move forward

- **Parameters**

  - **go_speed** – (int) 1 ~ 127 is forward.The smaller the value, the smaller the speed

## retreat(back_speed)

Control the car back

- **Parameters**

  - **back_speed** – (int) 1 ~ 127 is backward.The smaller the value, the smaller the speed

## pan_left(pan_left_speed)

Control the car to pan to the left

- **Parameters**

  - **pan_left_speed** – (int) 1 ~ 127.The smaller the value, the smaller the speed

## pan_right(pan_right_speed)

Control the car to pan to the right

- **Parameters**

  - **pan_right_speed** – (int) 1 ~ 127.The smaller the value, the smaller the speed

## clockwise_rotation(rotate_right_speed)

Control the car to rotate clockwise

- **Parameters**

  - **rotate_right_speed** – (int) 1 ~ 127.The smaller the value, the smaller the speed

## counterclockwise_rotation(rotate_left_speed)

Control the car to rotate counterclockwise

- **Parameters**

  - **rotate_left_speed** – (int) 1 ~ 127.The smaller the value, the smaller the speed

## stop()

stop motion.

## restore()

Motor stall recovery
