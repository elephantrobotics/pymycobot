# ChangeLog for pymycobot.

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
