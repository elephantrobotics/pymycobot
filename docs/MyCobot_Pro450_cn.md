# Pro 450 Python Socket API
[toc]
## API 使用介绍

API（Application Programming Interface），又称应用程序编程接口函数，是预先定义好的函数。使用以下函数接口时，请在一开始就导入我们的API库，导入方式为输入如下代码，否则将无法成功运行：

**注意：** 使用前需确保MyCobot Pro 450已开启服务端

```python
# Example
from pymycobot import Pro450Client

mc = Pro450Client('192.168.0.232', 4500)

print(mc.get_angles())
```

### 1. 系统状态

#### `get_system_version()`

- **功能：** 读取机器主控版本
- **返回值：** 主控版本号

#### `get_modified_version()`

- **功能：**读取修正版本号，仅内部使用
- **返回值：** 修正版本号

#### `get_robot_type()`

- **功能：** 检测机器型号

- **返回值：** 定义规则：实际机器型号。例如，MyCobot Pro 450 型号为 4503

#### `get_atom_version()`

- **功能：** 读取末端版本号
- **返回值：** 版本号(`float`)

#### `get_tool_modify_version()`

- **功能：** 读取末端更正版本号
- **返回值：** 更正版本号

### 2. 机器人整体运行状态

<!-- #### `power_on()`

- **功能：** atom open communication (default open)

  - Attentions： After executing poweroff or pressing emergency stop, it takes 7 seconds to power on and restore power

- **返回值:**
  - `1` - Power on completed.
  - `0` - Power on failed

#### `power_off()`

- **功能：** Power off of the robotic arm

- **返回值:**
  - `1` - Power on completed.
  - `0` - Power on failed

#### `is_power_on()`

- **功能：** judge whether robot arms is powered on or not

- **返回值:**
  - `1`: power on
  - `0`: power off
  - `-1`: error -->

#### `is_init_calibration()`

- **功能：** 检查机器是否已设置零位

- **返回值:** `bool`: 如果机器人已初始化校准零位，则为 True，否则为 False

#### `get_fresh_mode()`

- **功能:** 查询运动模式

- **返回值:** 
  - `0`: 插补模式
  - `1`: 刷新模式

#### `set_fresh_mode()`

- **功能:** 设置刷新模式
  
- **参数:**
  - `1`: 总是首先执行最新的命令。
  - `0`: 以队列的形式按顺序执行指令。

#### `get_debug_state()`

- **功能:** 获取当前机器人的调试日志模式。

- **返回值:** `int`: 当前调试日志状态。
  - `0`: 不记录任何调试日志 
  - `1`: 仅常规调试日志 (_debug.log)
  - `2`: 仅运动相关日志 (_move.log)
  - `3`: 常规 + 运动相关日志 (_debug.log + _move.log)
  - `4`: 仅电机读/控制频率日志 (_clock_rate_debug.log)
  - `5`: 常规 + 电机频率日志 (_debug.log + _clock_rate_debug.log)
  - `6`: 运动 + 电机频率日志 (_move.log + _clock_rate_debug.log)
  - `7`: 记录全部日志

#### `set_debug_state(log_state)`

- **功能:** 设置当前机器人的调试日志模式。
  
- **参数:** 
  - `log_state`: `int`, 调试日志状态（0 ~ 7）
    - `0`: 不记录任何调试日志 
    - `1`: 仅常规调试日志 (_debug.log)
    - `2`: 仅运动相关日志 (_move.log)
    - `3`: 常规 + 运动相关日志 (_debug.log + _move.log)
    - `4`: 仅电机读/控制频率日志 (_clock_rate_debug.log)
    - `5`: 常规 + 电机频率日志 (_debug.log + _clock_rate_debug.log)
    - `6`: 运动 + 电机频率日志 (_move.log + _clock_rate_debug.log)
    - `7`: 记录全部日志
- **返回值**：`int`
  - 1-成功
  - 0-失败
  - -1-错误

### 3. 机器人异常检测

#### `get_robot_status()`

- **功能：** 机器人错误安全状态
- **返回值:** 0 - 正常。其他 - 机器人触发碰撞检测

#### `servo_restore(joint_id)`

- **功能**：清除关节异常
- **参数**：
  - `joint_id`: int. 关节 id 1 - 6，254-所有关节恢复。

#### `get_comm_error_counts(joint_id)`

- **功能**：读取通信异常次数
- **参数**：
  - `joint_id`: int. 关节 id 1 - 6

### 4.MDI Mode and Operation

#### `get_angles()`

- **功能：** 获取所有关节的度数
- **返回值**：`list`所有度数的浮点列表

#### `send_angle(id, degree, speed)`

- **功能：** 向机械臂发送一个关节度数
- **参数：**
  - `id`：关节 id（`genre.Angle`），范围 int 1-6
  - `degree`：度数值（`float`）
      | 关节 Id | 范围 |
      | ---- | ---- |
      | 1 | -165 ~ 165 |
      | 2 | -120 ~ 120 |
      | 3 | -158 ~ 158 |
      | 4 | -165 ~ 165 |
      | 5 | -165 ~ 165 |
      | 6 | -175 ~ 175 |

    - `speed`：机械臂运动速度及范围 1~100

#### `send_angles(angles, speed)`

- **功能：** 将所有角度发送到机械臂的所有关节
- **参数：**
  - `angles`：度数列表（`List[float]`），长度 6
  - `speed`：（`int`）1 ~ 100

#### `get_coords()`

- **功能：** 从基于基准的坐标系获取机械臂坐标
- **返回值：** 坐标浮点列表：[x, y, z, rx, ry, rz]

#### `send_coord(id, coord, speed)`

- **功能：** 向机械臂发送一个坐标
- **参数：**
  - `id`：向机械臂发送一个坐标，1-6 对应 [x, y, z, rx, ry, rz]
  - `coord`：坐标值（`float`）
      | 坐标 ID | 范围 |
      | ---- | ---- |
      | x | -466 ~ 466 |
      | y | -466 ~ 466 |
      | z | -230 ~ 614 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |
  - `speed`：（`int`）1-100

#### `send_coords(coords, speed)`

- **功能：**：发送整体坐标和姿态，将机械臂头部从原点移动到您指定的点
- **参数：**
  - `coords`: 坐标列表，值`[x,y,z,rx,ry,rz]`，长度6
  - `speed` (`int`)：1 ~ 100

<!-- #### `pause(deceleration=False)`

- **功能：** Control the instruction to pause the core and stop all movement instructions
- **参数:**
  - deceleration: ： Whether to slow down and stop. Defaults to False.
- **返回值**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `is_paused()`

- **功能：** Check if the program has paused the move command
- **返回值:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **功能：** resume the robot movement and complete the previous command -->

#### `stop(deceleration=False)`

- **功能：** 停止机器人运动
- **参数:**
  - deceleration: ： 是否减速并停止。默认为 False。
- **返回值**:
  - `1` - 已停止
  - `0` - 未停止
  - `-1` - 错误

<!-- #### `is_in_position(data, flag)`

- **功能** : judge whether in the position.
- **参数:**
  - data: Provide a set of data that can be angles or coordinate values. If the input angle length range is 7, and if the input coordinate value length range is 6
  - flag data type (value range 0 or 1)
    - `0`: angle
    - `1`: coord
- **返回值**:
  - `1` - true
  - `0` - false
  - `-1 ` - error -->

#### `is_moving()`

- **功能：** 检测机器人是否在运动
- **返回值:**
  - `1` 正在运动
  - `0` 停止运动
  - `-1` 错误

<!-- ### 4. JOG Mode and Operation

#### `jog_angle(joint_id, direction, speed)`

- **功能：** jog control angle
- **参数**:
  - `joint_id`: Represents the joints of the robotic arm, represented by joint IDs ranging from 1 to 7
  - `direction(int)`: To control the direction of movement of the robotic arm, input `0` as negative value movement and input `1` as positive value movement
  - `speed`: 1 ~ 100

#### `jog_coord(coord_id, direction, speed)`

- **功能：** jog control coord.
- **参数:**
  - `coord_id`: (`int`) Coordinate range of the robotic arm: 1~6
  - `direction`:(`int`) To control the direction of machine arm movement, `0` - negative value movement, `1` - positive value movement
  - `speed`: 1 ~ 100

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能：** Single joint angle increment control
- **参数**:
  - `joint_id`: 1-7
  - `increment`: Incremental movement based on the current position angle
  - `speed`: 1 ~ 100

#### `jog_increment_coord(coord_id, increment, speed)`

- **功能：** Single joint angle increment control
- **参数**:
  - `joint_id`: axis id 1 - 6.
  - `increment`: Incremental movement based on the current position coord
  - `speed`: 1 ~ 100 -->

<!-- ### 5. Coordinate controlled attitude deviation angle

#### `get_solution_angles()`

- **功能：** Obtain the value of zero space deflection angle
- **返回值**：Zero space deflection angle value

#### `set_solution_angles(angle, speed)`

- **功能：** Obtain the value of zero space deflection angle

- **参数:**

  - ` angle` : Input the angle range of joint 1, angle range -90 to 90

  - `speed` : 1 - 100. -->

### 5. 软件关节限位

#### `get_joint_min_angle(joint_id)`

- **功能:** 获取指定关节的最小运动角度
- **参数:**
  - `joint_id` : 输入关节ID（范围1-6）
- **返回值**：`float` 角度值

#### `get_joint_max_angle(joint_id)`

- **功能:** 获取指定关节的最大运动角度
- **参数:**
  - `joint_id` : 输入关节ID（范围1-6）
- **返回值**：`float` 角度值

#### `参数(id, angle)`

- **功能:** 设置最小关节角度限制
- **参数:**
  - `id` : 输入关节ID（范围1-6）
  - `angle`: 参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得小于最小值

#### `set_joint_max(id, angle)`

- **功能：** 设置最大关节角度限制
- **参数：**
  - `id` ：输入关节ID（范围1-6）
  - `angle`：参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得大于最大值

### 6. 关节电机辅助控制

#### `get_servo_encoders()`

- **功能**：读取全关节编码器值
- **返回值**： 长度为6的列表
<!-- #### `is_servo_enable(servo_id)`

- **功能：** Detecting joint connection status
- **参数:** ` servo id` 1-7
- **返回值:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error

#### `is_all_servo_enable()`

- **功能：** Detect the status of all joint connections
- **返回值:**
  - `1`: Connection successful
  - `0`: not connected
  - `-1`: error -->

#### `set_servo_calibration(servo_id)`

- **功能：** 校准关节执行器的当前位置为角度零点
- **参数**:
  - `servo_id`: 1 - 6

#### `set_break（joint_id, value）`

- **功能：** 设置关节刹车
- **参数**：
  - `joint_id`: int. 关节 id 1 - 6
  - `value`: int. 0 - 掉使能, 1 - 使能
- **返回值:** 0 : 失败; 1 : 成功

#### `set_motor_enabled(joint_id, state`

- **功能：** 设置机器人力矩状态。（释放关节接口）
- **参数**：
  - `joint_id`: int. 关节 id 1 - 6, 254-所有关节
  - `state`: int. 0 - 掉使能, 1 - 使能

### 7. 运行辅助信息

#### `get_zero_pos()`

- **功能**: 读取零位编码器值
- **返回值:** `list`6个关节的零编码器的值

#### `get_servo_speeds()`

- **功能**：获取所有关节的运动速度
- **返回值**： 一个列表

#### `get_servo_currents()`

- **功能**：获取关节电流
- **返回值**：一个列表, 0 ~ 5000 mA

#### `get_servo_status()`

- **功能**：获取所有关节的运动状态
- **返回值**： 值为 0 表示没有错误

### 8. 末端 IO 控制

#### `set_digital_output(pin_no, pin_signal)`

- **功能:** 设置末端IO状态
- **参数**
  - `pin_no` (int): 引脚号，范围 1 ~ 2
  - `pin_signal` (int): 0 / 1, 0 - 低电平，1 - 高电平
- **返回值:**
  - `1`: 完成

#### `get_digital_input(pin_no)`

- **功能:** 获取末端IO状态
- **参数**: `pin_no` (int)，范围 1 ~ 2
- **返回值**: `int` 0 / 1, 0 - 低电平，1 - 高电平

<!-- ### 9. Robotic arm end gripper control

#### `set_gripper_state(flag, speed, _type_1=None)`

- **功能**: Adaptive gripper enable

- **参数**:

  - `flag (int) `: 0 - open 1 - close, 254 - release

  - `speed (int)`: 1 ~ 100

  - `_type_1 (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **功能**: Set the gripper value

- **参数**:

  - `gripper_value (int) `: 0 ~ 100

  - `speed (int)`: 1 ~ 100

  - `gripper_type (int)`:

    - `1` : Adaptive gripper (default state is 1)

    - `2` : A nimble hand with 5 fingers

    - `3` : Parallel gripper

    - `4` : Flexible gripper

#### `set_gripper_calibration()`

- **功能**: Set the current position of the gripper to zero

#### `set_gripper_enabled(value)`

- **功能**: Adaptive gripper enable setting
- **参数**:
  - `value` 1: Enable 0: Release

#### `set_gripper_mode(mode)`

- **功能**: Set gripper mode
- **参数**:
  - `value` :
    - 0: Transparent transmission mode
    - 1: normal mode

#### `get_gripper_mode()`

- **功能**: Get gripper mode
- **返回值**:
  - 0: Transparent transmission mode
  - 1: normal mode -->

<!-- ### 10. Button 功能 at the end of the robot arm

#### `is_btn_clicked()`

- **功能**: Get the status of the button at the end of the robot arm
- **返回值**:
  - 0: no clicked
  - 1: clicked

#### `set_color(r, g, b)`

- **功能**: Set the color of the end light of the robotic arm

- **参数**:

  - `r (int)`: 0 ~ 255

  - `g (int)`: 0 ~ 255

  - `b (int)`: 0 ~ 255 -->

<!-- ### 11. Drag Teaching

#### `drag_teach_save()`

- **功能：** Start recording and dragging teaching points.
  - Note: In order to display the best sports effect, the recording time should not exceed 90 seconds

#### `drag_teach_pause()`

- **功能：** Pause sampling

#### `drag_teach_execute()`

- **功能：** Start dragging the teach-in point, executing it only once. -->

<!-- ### 12. Cartesian space coordinate parameter setting

#### `set_tool_reference(coords)`

- **功能：** Set tool coordinate system.
- **参数**：`coords`: (`list`) [x, y, z, rx, ry, rz].
- **返回值:** NULL

#### `get_tool_reference(coords)`

- **功能：** Get tool coordinate system.
- **返回值:** `oords`: (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **功能：** Set world coordinate system.
- **参数**：`coords`: (`list`) [x, y, z, rx, ry, rz].
- **返回值:** NULL

#### `get_world_reference()`

- **功能：** Get world coordinate system.
- **返回值:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **功能：** Set base coordinate system.
- **参数：**`rftype`: 0 - base 1 - tool.

#### `get_reference_frame()`

- **功能：** Set base coordinate system.
- **返回值:**
  - `0` - base
  - `1` - tool.

#### `set_movement_type(move_type)`

- **功能：** Set movement type.
- **参数**：
  - `move_type`: 1 - movel, 0 - moveJ.

#### `get_movement_type()`

- **功能：** Get movement type.
- **返回值:**
  - `1` - movel
  - `0` - moveJ

#### `set_end_type(end)`

- **功能：** Get end coordinate system
- **参数:**
  - `end (int)`: `0` - flange, `1` - tool

#### `get_end_type()`

- **功能：** Obtain the end coordinate system
- **返回值:**
  - `0` - flange
  - `1` - tool

### 13. Circular motion

#### `write_move_c(transpoint, endpoint, speed)`

- 功能：Arc trajectory motion
- 参数：
  `transpoint(list)`：Arc passing through point coordinates
  `endpoint (list)`：Arc endpoint coordinates
  ` speed(int)`： 1 ~ 100-->

### 9. 底部 IO 控制

#### `set_base_io_output(pin_no, pin_signal)`

- **功能**：设置底部IO输出状态
- **参数**：
  - `pin_no` (`int`) 引脚号，范围 1 ~ 12
  - `pin_signal` (`int`): 0 - 低电平. 1 - 高电平

#### `get_base_io_output(pin_no)`

- **功能：** 获取底部IO输入状态
- **参数:**
  - `pin_no` (`int`) 引脚号，范围 1 ~ 12
- **返回值:** 0 - 低电平. 1 - 高电平 

### 10. 设置末端485通信

<!-- #### `tool_serial_restore()`

- **功能**：485 factory reset

#### `tool_serial_ready()`

- **功能：** Set up 485 communication
- **返回值:** 0 : not set 1 : Setup completed

#### `tool_serial_available()`

- **功能：** Set up 485 communication
- **返回值:** 0-Normal 1-Robot triggered collision detection -->

#### `tool_serial_read_data(data_len)`

- **功能：** 读取固定长度数据，读取前先读取缓冲区长度，读取完成后数据会被清除
- **参数**： data_len (int): 需要读取的字节数，范围1~45
- **返回值:** 0：未设置 1：设置完成

#### `tool_serial_write_data()`

- **功能：** 末端485发送数据，数据长度范围为1~45字节
- **返回值:** 0-正常 1-机器人触发碰撞检测

<!-- #### `tool_serial_flush()`

- **功能：** Clear 485 buffer
- **返回值:** 0-Normal 1-Robot triggered collision detection

#### `tool_serial_peek()`

- **功能：** View the first data in the buffer, the data will not be cleared
- **返回值:** 1 byte data

#### `tool_serial_set_baud(baud)`

- **功能：** Set 485 baud rate, default 115200
- **参数**: baud (int): baud rate
- **返回值:** NULL

#### `tool_serial_set_timeout(max_time)`

- **功能：** Set 485 timeout in milliseconds, default 30ms
- **参数**: max_time (int): timeout
- **返回值:** NULL -->

#### `set_over_time(timeout=1000)`

- **功能：** 设置超时时间(默认1s,超时时间内未读取数据缓冲区会清除)
- **参数**： timeout (int): 超时时间，单位ms，范围0~65535

#### `flash_tool_firmware()`

- **功能：** 烧录末端固件
<!-- - **返回值:** 0-Normal 1-Robot triggered collision detection -->

### 11. Pro 力控夹爪

#### `get_pro_gripper_firmware_version( gripper_id=14)`

- **功能**：读取Pro力控夹爪固件主次版本号
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

- **返回值**: (`float`) 版本号, x.x

#### `get_pro_gripper_firmware_modified_version(gripper_id=14)`

- **功能**：读取Pro力控夹爪固件修正版本号
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

- **返回值**：(`int`) 修正版本号

#### `set_pro_gripper_id(target_id, gripper_id=14)`

- **功能**：设置力控夹爪ID。
- **参数**：
  - `target_id` (`int`): 范围1 ~ 254。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功
  
#### `get_pro_gripper_id(gripper_id=14)`

- **功能**：读取力控夹爪ID。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 范围1 ~ 254。

#### `set_pro_gripper_angle(gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪角度。
- **参数**：
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功
  
#### `get_pro_gripper_angle(gripper_id=14)`

- **功能**：读取力控夹爪角度。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_open(gripper_id=14)`

- **功能**：打开力控夹爪。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_close(gripper_id=14)`

- **功能**：关闭力控夹爪。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_calibration(gripper_id=14)`

- **功能**：设置力控夹爪零位。（首次使用需要先设置零位）
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_status(gripper_id=14)`

- **功能**：读取力控夹爪夹持状态。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值:**
  - `0` - 正在运动。
  - `1` - 停止运动，未检测到夹到物体。
  - `2` - 停止运动，检测到夹到物体。
  - `3` - 检测到夹到物体之后，物体掉落。

#### `set_pro_gripper_enabled(state, gripper_id=14)`

- **功能**：设置力控夹爪使能状态。
- **参数**：
  - `state` (`bool`) ：0 或者1， 0 - 掉使能 1 - 上使能
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_torque(torque_value, gripper_id=14)`

- **功能**：设置力控夹爪扭矩。
- **参数**：
  - `torque_value` (`int`) ：扭矩值，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_torque(gripper_id=14)`

- **功能**：读取力控夹爪扭矩。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值:** (`int`) 0 ~ 100

#### `set_pro_gripper_speed(speed, gripper_id=14)`

- **功能**：设置力控夹爪速度。
- **参数**：
  - `speed` (int): 夹爪运动速度，取值范围 1 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_speed(gripper_id=14)`

- **功能**：读取力控夹爪速度。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：夹爪默认运动速度，范围 1 ~ 100。

#### `set_pro_gripper_abs_angle(gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪绝对角度。
- **参数**：
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_io_open_angle(gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪IO张开角度。
- **参数**：
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_io_open_angle(gripper_id=14)`

- **功能**：读取力控夹爪IO张开角度。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_io_close_angle(gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪IO闭合角度。
- **参数**：
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_io_close_angle(gripper_id=14)`

- **功能**：读取力控夹爪IO闭合角度。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_mini_pressure(pressure_value, gripper_id=14)`

- **功能**：设置力控夹爪最小启动力
- **参数**：
  - `pressure_value` (`int`): 启动力值，范围 0 ~ 254。
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_mini_pressure(gripper_id=14)`

- **功能**：读取力控夹爪最小启动力
- **参数**：
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`int`) 启动力值，范围 0 ~ 254。

#### `set_pro_gripper_protection_current(current_value, gripper_id=14)`

- **功能**：设置力控夹爪夹持电流
- **参数**：
  - `current_value` (`int`): 夹持电流值，范围 100 ~ 300。
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_protection_current(gripper_id=14)`

- **功能**：读取力控夹爪夹持电流
- **参数**：
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`int`) 夹持电流值，范围 100 ~ 300。