# ChangeLog for pymycobot

## v3.6.7 (2024-11-15)

- release v3.6.7
- Fixed the issue of incorrect naming of 320 electric gripper

## v3.6.5 (2024-10-25)

- release v3.6.5
- Added 280 set_encoders_drag interface

## v3.6.4 (2024-10-22)

- release v3.6.4
- Fix set_encoder function BUG

## v3.6.3 (2024-10-17)

- release v3.6.3
- Fix bug and add new function

## v3.6.2 (2024-10-11)

- release v3.6.2
- 630 adds partial control interface for force control gripper 

## v3.6.1 (2024-10-09)

- release v3.6.1
- Update 320 limit date

## v3.6.0 (2024-10-8)

- release v3.6.0
- Refactoring by robot type

## v3.5.3 (2024-9-24)

- release v3.5.3
- Fix(mycobotpro630): Remove clear_encoder_error() calls

## v3.5.2 (2024-9-13)

- release v3.5.2
- add mycobot class  init delay 1.5S

## v3.5.1 (2024-9-12)

- release v3.5.1
- fix bug

## v3.5.0 (2024-9-9)

- release v3.5.0
- Added 320 abnormal status acquisition interface

## v3.4.9 (2024-7-18)

- release v3.4.9
- fix A1 bug

## v3.4.8 (2024-7-11)

- release v3.4.7
- fix 280 get VOLTAGES error

## v3.4.7 (2024-6-28)

- release v3.4.7
- Mycobot 280 stop interface adds return (applicable to Atom v7.0 and later versions)

## v3.4.6 (2024-4-23)

- release v3.4.6
- 1. Fix the issue of Mercury API: drag_teach_clean not being able to be used
- 2. Unified naming convention for Mercury drag teaching API
- 3. Clean up excess files

## v3.4.5 (2024-4-9)

- release v3.4.5
- Add python-can

## v3.4.4 (2024-3-26)

- release v3.4.4
- Mercury API Pause and Stop Interface Add Slow Stop Parameters

## v3.4.3 (2024-3-18)

- release v3.4.3
- Fix bug

## v3.4.2 (2024-3-13)

- release v3.4.2
- Add Mercury new interface

## v3.4.1 (2024-3-11)

- release v3.4.1
- Optimize sockets

## v3.4.0 (2024-3-1)

- Updated Mercury X1 base control interface; A
- Added 630 API
- Updated Mercury motion range limits

## v3.3.9 (2024-2-2)

- release v3.3.9
- fix bug

## v3.3.7 (2024-1-26)

- release v3.3.7
- fix bug

## v3.3.6 (2024-1-12)

- release v3.3.6
- fix bug

## v3.3.5 (2023-12-29)

- release v3.3.5
- fix bug

## v3.3.4 (2023-12-21)

- release v3.3.4
- fix bug

## v3.3.3 (2023-12-15)

- release v3.3.3
- fix bug

## v3.3.2 (2023-12-5)

- release v3.3.2
- fix bug

## v3.3.1 (2023-12-4)

- release v3.3.1
- fix bug

## v3.3.0 (2023-11-9)

- release v3.3.0
- fix bug

## v3.2.9 (2023-11-9)

- release v3.2.9
- fix bug

## v3.2.8 (2023-11-9)

- release v3.2.8
- fix bug

## v3.2.7 (2023-11-7)

- release v3.2.7
- fix bug

## v3.2.6 (2023-11-3)

- release v3.2.6
- fix bug

## v3.2.5 (2023-11-3)

- release v3.2.5
- ultraArm angle limit update

## v3.2.4 (2023-11-1)

- release v3.2.4
- ultraArm add sync function

## v3.2.3 (2023-10-20)

- release v3.2.3
- Optimize myagv interface get_battery_info

## v3.2.2 (2023-10-17)

- release v3.2.2
- Add threading lock

## v3.2.0 (2023-10-09)

- release v3.2.0
- Fix bug

## v3.1.9 (2023-09-19)

- release v3.1.9
- Fix bug

## v3.1.8 (2023-09-04)

- release v3.1.8
- Add myAGV interface.
- Increase interface robustness.

## v3.1.7 (2023-8-8)

- release v3.1.7
- Fix myArm interface error

## v3.1.6 (2023-6-19)

- release v3.1.6
- Update synchronization control interface

## v3.1.5 (2023-07-06)

- release v3.1.5
- Add mycobot interface：get_basic_version(),set_transponder_mode()

## v3.1.4 (2023-07-05)

- release v3.1.4
- Update set_gservo_round() function.
- Add pro 600 interface

## v3.1.3 (2023-06-29)

- release v3.1.3
- Add open/close serial function

## v3.1.2 (2023-06-15)

- release v3.1.2
- Fix set_joint_min(),set_joint_max() interface issue

## v3.1.1 (2023-06-14)

- release v3.1.1
- Add new interface: set_gservo_round()

## v3.1.0 (2023-06-09)

- release v3.1.0
- Fix get_joint_min_angle() and get_joint_max_angle() interface bug.
- Remove the connect function in mycobotsocket
- update demo/Server.py file
- Add new interface: get_error_information(),clear_error_information()

## v3.0.9 (2023-04-27)

- release v3.0.9
- Fix UltraArm interface set_gpio_state error issue

## v3.0.8 (2023-04-14)

- release v3.0.8
- Fix Mybuddy GPIO control bug

## v3.0.7 (2023-03-06)

- release v3.0.7
- update ultraArm API

## v3.0.6 (2023-01-29)

- release v3.0.6
- Modify the value range of ultraArm speed (100 mm/s -> 200 mm/s).

## v3.0.5 (2023-01-17)

- release v3.0.5
- Fixed the issue of coords and angles issued by the ultraArm.

## v3.0.4 (2023-01-11)

- release v3.0.4
- Fix some bug

## v3.0.3 (2023-01-10)

- release v3.0.3
- Fix MyArm interface problems:
  - set_solution_angles()
- Fix ultraArm interface problems:
  - set_jog_coord()
  - set_gpio_state()
  - set_coord()

## v3.0.2 (2022-12-20)

- release v3.0.2
- Fix MyBuddy interface problems:
  - Add get_fresh_mode()
  - remove get_servo_current()
  - get_servo_voltages()、get_servo_status()、get_servo_temps()

## v3.0.1 (2022-12-9)

- release v3.0.1
- Fix MyBuddy interface problems: get_base_coords()
- Add mybuddy serial port to read logs

## v3.0.0 (2022-12-9)

- release v3.0.0
- Fix MyBuddy interface problems: get_base_coords()、set_gripper_state()
- Add 7-axis myArm interface

## v2.9.9 (2022-11-29)

- release 2.9.9
- add 320 interface:
  - set_gripper_mode(mode)
  - get_gripper_mode()
  - get_servo_last_pdi(id)

## v2.9.8 (2022-11-22)

- release v2.9.8
- Renaming mira to ultraArm
- Fix variable name conflicts

## v2.9.7 (2022-11-14)

- release v2.9.7
- Add MechArm class: Separate mecharm from mycobot
- Fix known bug

## v2.9.6 (2022-9-13)

- release v2.9.6
- Fix MyBuddySocket get_radians()、send_radians() BUG

## v2.9.5 (2022-9-13)

- release v2.9.5
- Add MyBuddy serial open(),is_open() function
- Add MyBuddy example

## v2.9.4 (2022-9-8)

- release v2.9.4
- fix read_next_error() bug
- fix set_ssid_pwd() bug

## v2.9.3 (2022-8-31)

- release v2.9.3
- fix jog_increment() bug
- Fixed an incorrect description of the set_fresh_mode interface

## v2.9.2 (2022-8-8)

- release v2.9.2
- fix bug jog_increment()

## v2.9.1 (2022-8-1)

- release v2.9.1
- fix MyBuddy read data bug.

## v2.9.0 (2022-8-1)

- release v2.9.0

## v2.8.1 (2022-7-1)

- release v2.8.1
- Fix MyBuddy send_angles() function BUG

## v2.8.0 (2022-6-28)

- release v2.8.0
- Added the ability to obtain and set the wifi account password through the serial port
- Fix mypalletizer class BUG
- Initial release of mybuddy interface

## v2.7.6 (2022-6-15)

- release v2.7.6
- Update 320 API

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
- new class `CommandGenerator` that generate binary real command.
- can import needed class from `pymycobot`, like:
  ```python
  from pymycobot import Mycobot, Angle, Coord, CommandGenerator
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
