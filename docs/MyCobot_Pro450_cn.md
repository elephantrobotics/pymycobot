# Pro 450 Python Socket API
[toc]
## API 使用介绍

API（Application Programming Interface），又称应用程序编程接口函数，是预先定义好的函数。使用以下函数接口时，请在一开始就导入我们的API库，导入方式为输入如下代码，否则将无法成功运行：

**注意：** 使用前需确保MyCobot Pro 450已开启服务端

```python
# Example
from pymycobot import Pro450Client

mc = Pro450Client('192.168.0.232', 4500)

if mc.is_power_on() !=1:
    mc.power_on()

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

#### `power_on()`

- **功能：** 启动机器人，上电
- **返回值:**
  - `1` - 上电成功.
  - `2` - 上电失败
  - `0` - 未上电

#### `power_off()`

- **功能：** 关闭机器人，下电

- **返回值:**
  - `1` - 成功接收指令.

#### `is_power_on()`

- **功能：** 判断机械臂是否上电

- **返回值:**
  - `1`: 上电成功
  - `0`: 未上电
  - `2`: 上电失败

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

#### `set_communication_mode(protocol_mode)`

- **功能:** 设置当前机器人Modbus通信模式。
  
- **参数:**
  - `protocol_mode`: `int` 0 或者 1
    - `0`: 关闭 Modbus协议
    - `1`: 打开 Modbus协议
- **返回值**：`int`
  - 1-成功
  - 0-失败
  - -1-错误

#### `get_communication_mode()`

- **功能:** 获取当前机器人通信模式。
  
- **返回值:**
  - `protocol_mode`: `int`
    - `0`: 自定义协议
    - `1`: Modbus协议

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

#### `get_error_information()`

- **功能**：读取机器人错误信息
- **返回值**：`int`
  - `0`：无错误信息
  - `1~6`：对应关节超出限位位置。
  - `32~36`：坐标运动异常。
    - `32`：坐标无解，请检查臂展是否临近限位
    - `33`：直线运动无相邻解。
    - `34`: 速度融合报错
    - `35`：零空间运动无相邻解
    - `36`：奇异位置无解，请使用关节控制离开奇异点

#### `clear_error_information()`

- **功能**：清除机器人错误信息

#### `over_limit_return_zero()`

- **功能** 机器关节超限回零指令

#### `get_motors_run_err()`

- **功能**：读取机器人运动中的电机错误信息
- **返回值**：`list`, 长度为6的列表，全部是0，代表正常

#### `is_motor_pause()`

- **功能**：读取电机暂停状态
- **返回值**：`int`
  - `0`: 未暂停
  - `1`: 已暂停，可使用resume()接口恢复。
  - 
### 4. 机器人运动控制

#### `set_control_mode(mode)`

- **功能**：设置机器人运动模式
- **参数**：
  - `mode`: `int`. 0 ~ 1，默认0
    - `0`: 位置模式
    - `1`: 力矩模式

#### `get_control_mode()`

- **功能**：获取机器人运动模式
- **返回值**：
  - `0`: 位置模式
  - `1`: 力矩模式
  
#### `get_angles()`

- **功能：** 获取所有关节的角度
- **返回值**：`list`所有角度的浮点列表

#### `get_angle(joint_id)`

- **功能：** 获取单关节的角度
- **参数：**
  - `joint_id`: `int`，关节ID， 范围1 ~ 6
- **返回值**：`float` 单关节角度

#### `send_angle(id, degree, speed)`

- **功能：** 向机械臂发送一个关节角度
- **参数：**
  - `id`：关节 id（`genre.Angle`），范围 int 1-6
  - `degree`：角度值（`float`）
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

#### `pause(deceleration=0)`

- **功能：** 控制指令暂停核心并停止所有运动指令
- **参数:**
  - `deceleration`： 是否减速并停止，默认为 0。1代表缓暂停
- **返回值**:
  - `1` - stopped
  - `0` - not stop
  - `-1` - error

#### `is_paused()`

- **功能：** 检查程序是否暂停了移动命令
- **返回值:**
  - `1` - paused
  - `0` - not paused
  - `-1` - error

#### `resume()`

- **功能：** 恢复机器人运动并完成之前的命令

#### `stop(deceleration=0)`

- **功能：** 停止机器人运动
- **参数:**
  - `deceleration` ： 是否减速并停止。默认为 0。1代表缓停
- **返回值**:
  - `1` - 已停止
  - `0` - 未停止
  - `-1` - 错误

#### `is_in_position(data, flag)`

- **功能** : 判断是否到达点位。
- **参数:**
  - `data`:提供一组数据，可以是角度或坐标值。假设输入角度长度范围为 6，输入坐标值长度范围为 6
  - `flag` 数据类型（值范围 0 或 1）
    - `0`: 角度值列表
    - `1`: 坐标值列表
- **返回值**:
  - `1` - true
  - `0` - false
  - `-1 ` - error

#### `is_moving()`

- **功能：** 检测机器人是否在运动
- **返回值:**
  - `1` 正在运动
  - `0` 停止运动
  - `-1` 错误

### 5. JOG 模式和操作

#### `jog_angle(joint_id, direction, speed)`

- **功能：** jog 控制角度，关节持续运动
- **参数**:
  - `joint_id`: 表示机械臂的关节ID，范围 1 ~ 6
  - `direction(int)`: 控制机械臂运动方向，输入`0`为负值方向运动，输入`1`为正值方向运动
  - `speed`: 1 ~ 100

#### `jog_coord(coord_id, direction, speed)`

- **功能：** jog 控制坐标， 坐标持续运动.
- **参数:**
  - `coord_id`: (`int`) 机械臂坐标轴范围：1~6
  - `direction`:(`int`) 控制机械臂运动方向，输入`0`为负值方向运动，输入`1`为正值方向运动
  - `speed`: 1 ~ 100

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能：** 单关节角度增量控制
- **参数**:
  - `joint_id`: 1-6
  - `increment`: 基于当前位置角度的增量移动
  - `speed`: 1 ~ 100

#### `jog_increment_coord(coord_id, increment, speed)`

- **功能：** 单坐标增量控制
- **参数**:
  - `coord_id`: 坐标轴 1 - 6.
  - `increment`: 基于当前位置坐标的增量移动
  - `speed`: 1 ~ 100

<!-- ### 5. Coordinate controlled attitude deviation angle

#### `get_solution_angles()`

- **功能：** Obtain the value of zero space deflection angle
- **返回值**：Zero space deflection angle value

#### `set_solution_angles(angle, speed)`

- **功能：** Obtain the value of zero space deflection angle

- **参数:**

  - ` angle` : Input the angle range of joint 1, angle range -90 to 90

  - `speed` : 1 - 100.-->

### 6. 速度/加速度参数

#### `get_max_speed(mode)`

- **功能:** 获取最大运动速度
- **参数:**
  - `mode` : `int`
    - `0`: 角度速度
    - `1`: 坐标速度
- **返回值**：角速度范围1～150°/s，坐标速度范围1～200mm/s

#### `set_max_speed(mode, max_speed)`

- **功能:** 设置最大运动速度
- **参数:**
  - `mode` : `int`
    - `0`: 角度速度
    - `1`: 坐标速度
  - `max_speed`: 角度速度范围1～150°/s，坐标速度范围1～200mm/s

#### `get_max_acc(mode)`

- **功能:** 获取最大运动加速度
- **参数:**
  - `mode` : `int`
    - `0`: 角度加速度
    - `1`: 坐标加速度
- **返回值**：角度加速度范围1～150°/s，坐标加速度范围1～400mm/s

#### `set_max_acc(mode, max_acc)`

- **功能:** 设置最大运动加速度
- **参数:**
  - `mode` : `int`
    - `0`: 角度加速度
    - `1`: 坐标加速度
  - `max_acc`: 角度加速度范围1～150°/s，坐标加速度范围1～400mm/s

### 7. 软件关节限位

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

#### `set_joint_min_angle(id, angle)`

- **功能:** 设置最小关节角度限制
- **参数:**
  - `id` : 输入关节ID（范围1-6）
  - `angle`: 参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得小于最小值

#### `set_joint_max_angle(id, angle)`

- **功能：** 设置最大关节角度限制
- **参数：**
  - `id` ：输入关节ID（范围1-6）
  - `angle`：参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得大于最大值

### 8. 关节电机辅助控制

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

### 9. 拖动示教

#### `drag_teach_save()`

- **功能：** 开始录制并拖动教学点。
  - 注意：为了呈现最佳运动效果，录制时间请勿超过90秒

<!-- #### `drag_teach_pause()`

- **功能：** Pause sampling -->

#### `drag_teach_execute()`

- **功能：** 开始拖动示教点，仅执行一次。

#### `drag_teach_clean()`

- **功能：** 清除采样点。

### 10. 动力学

#### `get_collision_mode()`

- **功能**: 查询碰撞检测模式
- **返回值:**
  - `0`: 关闭
  - `1`: 打开

#### `set_collision_mode(mode)`

- **功能**设置关节碰撞阈值
- **参数**： `int`
  - `mode`:
    - `0`: 关闭
    - `1`: 打开

#### `get_collision_threshold()`

- **功能**：获取关节碰撞阈值
- **返回值**：一个列表, 全关节碰撞阈值

#### `set_torque_comp(joint_id, damping, comp_value=0)`

- **功能**设置力矩补偿系数
- **参数**：
  - `joint_id` `int`: 关节ID，范围 1 ~ 6
  - `damping` `int`: 范围 0 ~ 1。 1-打开，0-关闭
  - `comp_value`: 补偿值，范围0~250，默认0，值越小，关节拖动越吃力

#### `get_torque_comp()`

- **功能**：获取力矩补偿系数
- **返回值**：一个列表, 全关节力矩补偿系数

<!-- #### `set_identify_mode(mode)`

- **功能**: 设置动力学参数辨识模式
- **参数**： `int`
  - `mode`:
    - `0`: 关闭
    - `1`: 打开

#### `get_identify_mode()`

- **功能**：获取动力学参数辨识模式
- **返回值**：
  - `0`: 关闭
  - `1`: 打开 -->

#### `fourier_trajectories(trajectory)`

- **功能**: 执行动力学辨识轨迹
- **参数**： 
  - `trajectory`: `int`，范围 0 ~ 1

<!-- #### `set_dynamic_parameters(add, data)`

- **功能**: 设置动力学参数
- **参数**:
  - `add`: `int`，范围 0 ~ 62
  - `data`: 参数值

#### `get_dynamic_parameters(add)`

- **功能**: 读取动力学参数
- **参数**：
  - `add`: (int), 范围 0 ~ 62
- **返回值**：data * 0.001 -->

#### `parameter_identify()`

- **功能**: 动力学参数辨识

### 11. 圆弧运动

#### `write_move_c(transpoint, endpoint, speed)`

- **功能**：圆弧轨迹运动(指定途经点）
- **参数**：
  - `transpoint(list)`：圆弧坐标途经点
  - `endpoint (list)`：圆弧坐标结束点
  - `speed(int)`： 1 ~ 100

### 12. 运行辅助信息

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

### 13. 末端 IO 控制

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

#### `get_digital_inputs()`

- **功能:** 读取末端所有引脚的状态，包括：IN1、IN2、按钮 1（右侧）以及按钮 2（按钮 2 更靠近紧急停止按钮，位于左侧）。
- **返回值**: `list[int]` 0 / 1, 0 - 低电平，1 - 高电平。 eg: [0, 0, 1, 0]代表按钮1被按下。

### 14. 末端灯板功能

<!-- #### `is_btn_clicked()`

- **功能**: Get the status of the button at the end of the robot arm
- **返回值**:
  - 0: no clicked
  - 1: clicked -->

#### `set_color(r, g, b)`

- **功能**: 设置机械臂末端灯光颜色

- **参数**:

  - `r (int)`: 0 ~ 255

  - `g (int)`: 0 ~ 255

  - `b (int)`: 0 ~ 255

### 15. 底部 IO 控制

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

#### `set_base_external_config(communicate_mode, baud_rate, timeout)`

- **功能**：设置底部外部设备配置
- **参数**：
  - `communicate_mode` (`int`) 范围 1 ~ 2
    - `1`: 485
    - `2`: can
  - `baud_rate` (`int`): 波特率
    - `timeout`: (`int`) 超时时间，单位毫秒

#### `get_base_external_config()`

- **功能**：读取底部外部设备配置
- **返回值**：`list` 返回列表：[通信模式， 波特率， 超时时间]

#### `base_external_can_control(can_id, can_data)`

- **功能：** 底部外部设备can控制
- **参数:**
  - `can_id` (`int`) 范围 1 ~ 4
  - `can_data` (`list`) 列表内容为十六进制格式，最大长度是64。

#### `base_external_485_control(data)`

- **功能：** 底部外部设备485控制
- **参数:**
  - `data` (`list`) 列表内容为十六进制格式，最大长度是64。

### 16. 设置末端485通信

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

#### `tool_serial_write_data(command)`

- **功能：** 末端485发送数据，数据长度范围为1~45字节
- **参数**： 
  - `command` (`list`): modbus格式的数据指令
- **返回值:** modbus数据列表

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

#### `flash_tool_firmware(main_version, modified_version=0)`

- **功能：** 烧录末端固件
- **参数:**
  - `main_version (str)`: 主次版本号，比如 `'1.1'`
  - `modified_version (int)`: 更正版本号，范围 0 ~ 255，默认是 0 

### 17. 工具坐标系操作

#### `set_tool_reference(coords)`

- **功能:** 设置工具坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

      | 坐标 ID | 范围 |
      | ---- | ---- |
      | x | -1000 ~ 1000 |
      | y | -1000 ~ 1000 |
      | z | -1000 ~ 1000 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |

#### `get_tool_reference(coords)`

- **功能:** 获取工具坐标系
- **返回值:** (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **功能:** 设置世界坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

      | 坐标 ID | 范围 |
      | ---- | ---- |
      | x | -1000 ~ 1000 |
      | y | -1000 ~ 1000 |
      | z | -1000 ~ 1000 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |

#### `get_world_reference()`

- **功能:** 获取世界坐标系.
- **返回值:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **功能:** 设置基坐标系
- **参数：**
  - `rftype`: 0 - 基坐标（默认） 1 - 世界坐标.

#### `get_reference_frame()`

- **功能:** 获取基坐标系
- **返回值:** (`list`) [x, y, z, rx, ry, rz].

#### `set_movement_type(move_type)`

- **功能:** 设置移动类型
- **参数**：
  - `move_type`: 1 - moveL, 0 - moveJ.

#### `get_movement_type()`

- **功能:** 获取移动类型
- **返回值:**
  - `1` - moveL
  - `0` - moveJ

#### `set_end_type(end)`

- **功能:** 设置末端坐标系
- **参数:**
  - `end (int)`: `0` - 法兰（默认）, `1` - 工具

#### `get_end_type()`

- **功能:** 获取末端坐标系
- **返回值:**
  - `0` - 法兰（默认）
  - `1` - 工具

### 18. 算法参数

#### `get_vr_mode()`

- **功能:** 获取VR模式
- **返回值:**
  - `0`: 关闭
  - `1`: 打开

#### `set_vr_mode(move)`

- **功能:** 设置VR模式
- **参数**：
  - `move`: 1 - 打开, 0 - 关闭.

#### `get_model_direction()`

- **功能:** 获取关节模型方向
- **返回值:** 1-6关节的模型方向
  - `1` - 与电机同向
  - `0` - 与电机反向

#### `set_model_direction(joint_id, direction)`

- **功能:** 设置关节模型方向
- **参数:**
  - `joint_id (int)`: 1 ~ 6
  - `direction (int)`: `1` - 与电机同向. `0` - 与电机反向

#### `get_filter_len(rank)`

- **功能:** 获取滤波器参数
- **参数:**
  - `rank`: `int`
    - `1`：拖动示教采样滤波器
    - `2`：拖动示教执行滤波器
    - `3`：关节速度融合滤波器
    - `4`：坐标速度融合滤波器
    - `5`：拖动示教采样周期
- **返回值:** `int` 1 ~ 100

#### `set_filter_len(rank, value)`

- **功能:** 设置滤波器参数
- **参数:**
  - `rank (int)`: 1 ~ 5
  - `value (int)`: 1 ~ 100

#### `get_fusion_parameters(rank_mode)`

- **功能:** 获取速度融合规划参数
- **参数:** 
  - `rank_mode`: 1 ~ 4
    - `1`：融合关节速度
    - `2`：融合关节加速度
    - `3`：融合坐标速度
    - `4`：融合坐标加速度
- **返回值:**  `int`, 0 ~ 1000

#### `set_fusion_parameters(rank_mode, value)`

- **功能:** 设置速度融合规划参数
- **参数:**
  - `rank_mode (int)`: 1 ~ 4
  - `value (int)`: 0 ~ 1000

### 19. 运动学算法接口

#### `solve_inv_kinematics(target_coords, current_angles)`

- **功能** : 将坐标转为角度。
- **参数：**
  - `target_coords`: `list` 所有坐标的浮点列表。
  - `current_angles`: `list` 所有角度的浮点列表，机械臂当前角度
- **返回值**: `list` 所有角度的浮点列表。

### 20. Pro 力控夹爪

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