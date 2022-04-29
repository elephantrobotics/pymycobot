# ChangeLog for pymycobot

## v2.7.5 (2022-4-29)

- release v2.7.5
- Fixed an issue where the M5 would restart when the serial port was initialized for the first time.
- New interface: `get_tool_reference`,`set_tool_reference`,`set_world_reference`,`get_world_reference`,`set_reference_frame`,`get_reference_frame`,`set_movement_type`,`get_movement_type`,`set_end_type`,`get_end_type`,`get_plan_speed`,`get_plan_acceleration`,`set_plan_speed`,`set_plan_acceleration`,`get_servo_speeds`,`get_servo_currents`,`get_servo_voltages`,`get_servo_status`,`get_servo_temps`.

## v2.7.4 (2021-12-15)

- release v2.7.4
- Fix m5 network communication bug

## v2.7.3 (2021-12-15)

- release v2.7.3
- fix get_tof_distance() no return problem.

## v2.7.2 (2021-12-15)

- release v2.7.2
- Added network communication and distance detection functions for m5 and seed versions

## v2.7.1 (2021-11-23)

- release v2.7.1
- Refactor tcp/ip control method
- Optimize tcp/ip control gpio
- Fix the error of get_basic_input() again
- Added instructions for using MyCobotSocket

## v2.7.0 (2021-11-15)

- release v2.7.0
- Added TCP/IP to control the robotic arm
- Fixed the problem that the get_basic_input() and get_digital_input() methods return incorrect data

## v2.6.1 (2021-10-19)

- release v2.6.1
- Fix transmission angle or coordinate jam problem. (sync_send_angles() and sync_send_coords())

## v2.6.0 (2021-9-13)

- release v2.6.0
- Preliminary adaptation mypalletizer

## v2.5.9 (2021-08-30)
- release v2.5.9
- import `utils` module.
- modify `send_coord()` for atom4.0
- update document.

## v2.5.8 (2021-08-10)
- release v2.5.8
- fix `is_servo_enable` error.
- fix `send_coord` index error.

## v2.5.7 (2021-08-05)

- release v2.5.7
- fix error. [#19](https://github.com/elephantrobotics/pymycobot/issues/19) [#20](https://github.com/elephantrobotics/pymycobot/issues/20)

## v2.5.6 (2021-07-30)

- release v2.5.6
- add new api:
  - `def get_encoders(self)`
  - `def get_basic_input(self, pin, v)`
- add `utils` module.
- update api document.

---

## v2.5.5 (2021-07-22)

- release v2.5.5
- fix GPIO api.

---

## v2.5.4 (2021-07-19)

- release v2.5.4
- add methods for raspberry pi version basic control.
  - `def gpio_init(self)`
  - `def gpio_output(self, pin, v)`

---

## v2.5.3 (2021-06-24)

- release v2.5.3
- sync with Atom3.1

---

## v2.5.1 (2021-06-10)

- improved parameter checking.
- new class `MycobotCommandGenerator` that generate binary real command.
- can import needed class from `pymycobot`, like:
  ```python
  from pymycobot import Mycobot, Angle, Coord, MycobotCommandGenerator
  ```

---

## v2.4.2 (2021-05-27)

- release v2.4.2
- fixed `set_pwm_output()`

---

## 2021-05-24

- update demo.

---

## v2.4.0 (2021-04-27)

- set_free_mode -> release_all_servos
- Add new port:
  - is_controller_connected
  - set_servo_data
  - get_servo_data
  - set_servo_calibration
  - set_basic_output
- Update API document.

---

## v2.3.6 (2021-04-25)

- fix focus_servo error

---

## v2.3.5 (2021-04-20)

- fix v2.3.4 install error

---

## v2.3.4 (2021-04-19)

- update debug mode

---

## v2.3.3 (2021-04-02)

- fix bug.

---

## v2.3.1 (2021-03-29)

- fix error bug
- add new method `set_encoder`, `get_encoder`, `set_encoders`

---

## v2.3.0 (2021-03-26)

- fix `is_in_position()`
- refactor process method
- some methods can be chained
- add new methods to control pump
- change `set_led_color(rgb:str)` -> `set_color(r:int, g:int, b:in)`

---

## 2021-03-12

added more test file.

---

## v2.2.0 (2021-02-05)

- add new method for girpper and IO.

---

## v2.1.2 (2021-01-25)

- refactor pymycobot
- add Error class

---

## 2021-01-20

`v1.0.6` fix get_coords() error.

relase v2.0.0

---

## 2021-01-16

Upload to server, can use `pip` to installation now.

---

## 2021-01-09

Fix the API problem that `is_moving()` and other methods of mycobot cannot be used.

---

## 2021-01-08

Python API add new methods:

- `jog_angle()`
- `jog_coord()`
- `jog_stop()`

---

## 202-12-30

Adding usage documents to Python API.

---

## 2020-12-29

Python API supports python2.7

Modify the serial port to manual setting, support the use of window.

---
