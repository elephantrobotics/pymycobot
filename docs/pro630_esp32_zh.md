# Pro 630 Python使用手册

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [Pro 630 Python使用手册](#pro-630-python使用手册)
- [1. Python API](#1-python-api)
  - [API 使用指南](#api-使用指南)
    - [1. 系统状态](#1-系统状态)
      - [`get_system_version()`](#get_system_version)
      - [`get_robot_type()`](#get_robot_type)
      - [`get_atom_version()`](#get_atom_version)
    - [2. 整体状态](#2-整体状态)
      - [`power_on()`](#power_on)
      - [`power_off()`](#power_off)
      - [`is_power_on()`](#is_power_on)
      - [`is_init_calibration()`](#is_init_calibration)
      - [`power_on_only()`](#power_on_only)
    - [3.机器人异常处理](#3机器人异常处理)
      - [`get_error_information()`](#get_error_information)
      - [`clear_error_information()`](#clear_error_information)
      - [`get_robot_status()`](#get_robot_status)
      - [`servo_restore(joint_id)`](#servo_restorejoint_id)
      - [`get_comm_error_counts(joint_id, type)`](#get_comm_error_countsjoint_id-type)
      - [`over_limit_return_zero()`](#over_limit_return_zero)
    - [4. MDI 模式与机器人控制](#4-mdi-模式与机器人控制)
      - [`set_control_mode(mode)`](#set_control_modemode)
      - [`get_control_mode()`](#get_control_mode)
      - [`get_angles()`](#get_angles)
      - [`send_angles(angles, speed)`](#send_anglesangles-speed)
      - [`get_angle()`](#get_angle)
      - [`send_angle(id, degree, speed)`](#send_angleid-degree-speed)
      - [`get_coords()`](#get_coords)
      - [`send_coord(id, coord, speed)`](#send_coordid-coord-speed)
      - [`send_coords(coords, speed, mode)`](#send_coordscoords-speed-mode)
      - [`pause()`](#pause)
      - [`is_paused()`](#is_paused)
      - [`resume()`](#resume)
      - [`stop()`](#stop)
      - [`is_in_position(data, flag)`](#is_in_positiondata-flag)
      - [`is_moving()`](#is_moving)
    - [5. JOG 模式和操作](#5-jog-模式和操作)
      - [`jog_angle(joint_id, direction, speed)`](#jog_anglejoint_id-direction-speed)
      - [`jog_coord(coord_id, direction, speed)`](#jog_coordcoord_id-direction-speed)
      - [`jog_increment_angle(joint_id, increment, speed)`](#jog_increment_anglejoint_id-increment-speed)
      - [`jog_increment_coord(coord_id, increment, speed)`](#jog_increment_coordcoord_id-increment-speed)
    - [6. 软件关节限位](#6-软件关节限位)
      - [`get_joint_min_angle(joint_id)`](#get_joint_min_anglejoint_id)
      - [`get_joint_max_angle(joint_id)`](#get_joint_max_anglejoint_id)
      - [`set_joint_min(id, angle)`](#set_joint_minid-angle)
      - [`set_joint_max(id, angle)`](#set_joint_maxid-angle)
    - [7. 关节电机辅助控制](#7-关节电机辅助控制)
      - [`is_servo_enable(servo_id)`](#is_servo_enableservo_id)
      - [`is_all_servo_enable()`](#is_all_servo_enable)
      - [`set_servo_calibration(servo_id)`](#set_servo_calibrationservo_id)
      - [`release_servo(servo_id)`](#release_servoservo_id)
      - [`focus_servo(servo_id)`](#focus_servoservo_id)
      - [`release_all_servos()`](#release_all_servos)
      - [`focus_all_servos()`](#focus_all_servos)
      - [`set_break（joint_id, value）`](#set_breakjoint_id-value)
      - [`get_servo_speeds()`](#get_servo_speeds)
      - [`get_servo_currents()`](#get_servo_currents)
      - [`get_servo_status()`](#get_servo_status)
    - [8. 拖动示教](#8-拖动示教)
      - [`drag_teach_save()`](#drag_teach_save)
      - [`drag_teach_pause()`](#drag_teach_pause)
      - [`drag_teach_execute()`](#drag_teach_execute)
      - [`drag_teach_clean()`](#drag_teach_clean)
    - [9. 动力学](#9-动力学)
      - [`set_collision_mode(mode)`](#set_collision_modemode)
      - [`set_collision_threshold(joint_id, value = 100)`](#set_collision_thresholdjoint_id-value--100)
      - [`get_collision_threshold()`](#get_collision_threshold)
      - [`set_torque_comp(joint_id, value=100)`](#set_torque_compjoint_id-value100)
      - [`get_torque_comp()`](#get_torque_comp)
    - [10. 算法参数](#10-算法参数)
    - [11. 底部IO输入输出](#11-底部io输入输出)
      - [`set_basic_output(pin_no, pin_signal)`](#set_basic_outputpin_no-pin_signal)
      - [`get_basic_input(pin_no)`](#get_basic_inputpin_no)
    - [12. 末端夹爪控制](#12-末端夹爪控制)
      - [`set_gripper_state(flag, speed, _type_1=None)`](#set_gripper_stateflag-speed-_type_1none)
      - [`set_gripper_value(gripper_value, speed, gripper_type=None)`](#set_gripper_valuegripper_value-speed-gripper_typenone)
      - [`set_gripper_calibration()`](#set_gripper_calibration)
      - [`set_gripper_enabled(value)`](#set_gripper_enabledvalue)
      - [`set_gripper_mode(mode)`](#set_gripper_modemode)
      - [`get_gripper_mode()`](#get_gripper_mode)
    - [13. atom末端RGB](#13-atom末端rgb)
      - [`set_color(r, g, b)`](#set_colorr-g-b)
    - [14. 末端IO控制](#14-末端io控制)
      - [`set_digital_output(pin_no, pin_signal)`](#set_digital_outputpin_no-pin_signal)
      - [`get_digital_input(pin_no)`](#get_digital_inputpin_no)
      - [`is_btn_clicked()`](#is_btn_clicked)
    - [15. 工具坐标系操作](#15-工具坐标系操作)
      - [`get_world_reference()`](#get_world_reference)
      - [`set_reference_frame(rftype)`](#set_reference_framerftype)
      - [`get_reference_frame()`](#get_reference_frame)
      - [`set_movement_type(move_type)`](#set_movement_typemove_type)
      - [`get_movement_type()`](#get_movement_type)
      - [`set_end_type(end)`](#set_end_typeend)
      - [`get_end_type()`](#get_end_type)
    - [16. 同步/异步 控制模式](#16-同步异步-控制模式)
      - [`set_pos_switch(mode)`](#set_pos_switchmode)
      - [`get_pos_switch()`](#get_pos_switch)
- [2. 使用方法](#2-使用方法)
  - [串口通信](#串口通信)
  - [TCP/IP 通信](#tcpip-通信)
    - [服务端](#服务端)
    - [客户端](#客户端)
- [打包姿态](#打包姿态)

<!-- /code_chunk_output -->

# 1. Python API

## API 使用指南

### 1. 系统状态

#### `get_system_version()`

- **功能：** 获取系统版本
- **返回值：** 系统版本

#### `get_robot_type()`

- **功能：** 获取机器人id

- **返回值：** 定义规则: 真实机器模型，例如水星A1为4500

#### `get_atom_version()`

- **功能：** 获取终端版本号
- **返回值：** 终端参数(`float`)

### 2. 整体状态

#### `power_on()`

- **功能:** 机械臂打开电源
  
  - 注意：执行断电或按下紧急停止按钮后，需要 8 秒钟才能接通电源并恢复供电。

- **返回值:**
  
  - `1` - 打开电源完成
  - `0` - 打开电源失败

#### `power_off()`

- **功能:** 关闭机械臂电源

- **返回值:**
  
  - `1` - 关闭电源完成
  - `0` - 关闭电源失败

#### `is_power_on()`

- **功能:** 判断机械臂是否上电

- **返回值:**
  
  - `1`  - 上电
  - `0`  - 未上电
  - `-1` - 错误

#### `is_init_calibration()`

- **功能:** 检查机器人是否已初始化以进行校准
- **返回值:**
  - `True`: 该机器人已初始化
  - `False`: 其它情况

#### `power_on_only()`

- **功能:** 仅上电

### 3.机器人异常处理

#### `get_error_information()`

- **功能:** 获取机器人报错信息
  
  - **返回值:**
    - `0`: 无错误
    - `1~6`: 相应关节超出极限位置
    - `16~19`: 碰撞保护
    - `32`: 运动学逆解法无解
    - `33~34`: 直线运动没有相邻解

#### `clear_error_information()`

- **功能:** 清除机器人报错信息

#### `get_robot_status()`

- **功能:** 上位机错误安全状态
- **返回值:** 
  - `0`  - 正常
  - `其它`  - 机器人触发碰撞检测

#### `servo_restore(joint_id)`

- **功能:** 关节异常恢复
- **参数:**
  - `joint_id`: `int` joint id 1 - 6

#### `get_comm_error_counts(joint_id, type)`

- **功能:** 读取通信异常次数
- **参数:**
  - `joint_id`: `int` joint id 1 - 6
  - `type`: int. Error type 1 - 4
    - `1` -  关节发送异常次数
    - `2`  - 关节获取异常次数
    - `3`  - 末端发送异常次数
    - `4`  - 末端获取异常次数

#### `over_limit_return_zero()`

- **功能:** 超限回零指令(禁止在回零结束前发送stop以外的运动指令)

### 4. MDI 模式与机器人控制

#### `set_control_mode(mode)`

- **功能:** 设置机器人运动模式
- **参数:**
  - **mode**
    - `0`  - 位置模式
    - `1`  - 力矩模式

#### `get_control_mode()`

- **功能:** 读取机器人运动模式
- **返回值:**
  - `0`  - 位置模式
  - `1`  - 力矩模式

#### `get_angles()`

- **功能:** 读取全部角度
- **返回值**: `list` 所有关节的角度值

#### `send_angles(angles, speed)`

- **功能:** 发送全部角度
- **参数:**
  - `angles`:`list` 长度为6的数组，所有关节的角度值
  - `speed`: (`int`) 1 ~ 100
- **返回值:** (`int`) 1

#### `get_angle()`

- **功能:** 读取单关节角度
- **参数**： joint_id (int): 1 ~ 6
- **返回值:** 该关节的角度值

#### `send_angle(id, degree, speed)`

- **功能:** 发送单关节角度

- **参数:**
  
  - `id`: Joint id(`genre.Angle`), 范围1~6
  
  - `degree`: `float` 角度值
    
    | Joint Id | range      |
    | -------- | ---------- |
    | 1        | -180 ~ 180 |
    | 2        | -45 ~ 225  |
    | 3        | -165 ~ 165 |
    | 4        | -90 ~ 270  |
    | 5        | -180 ~ 180 |
    | 6        | -180 ~ 180 |
  
  - `speed`：机械臂运动的速度， 范围是1~150

- **返回值:** (`int`) 1

#### `get_coords()`

- **功能:** 读取坐标
- **返回值:** 坐标数组:[x, y, z, rx, ry, rz]

#### `send_coord(id, coord, speed)`

- **功能:** 发送单独坐标

- **参数:**
  
  - `id`:向机械臂发送一个坐标, 1-6 分别表示 [x, y, z, rx, ry, rz]
  
  - `coord`: `float` 坐标值
    
    | Coord Id | range      |
    | -------- | ---------- |
    | 1        | -630 ~ 630 |
    | 2        | -630 ~ 630 |
    | 3        | -425 ~ 835 |
    | 4        | -180 ~ 180 |
    | 5        | -180 ~ 180 |
    | 6        | -180 ~ 180 |
  
  - `speed`: `int` 1-200

- **返回值:** (`int`) 1

#### `send_coords(coords, speed, mode)`

- **功能:**: 发送整体坐标和姿势，将机械臂的头部从原点移动到指定点
- **参数:**
  - coords: ：坐标值数组 `[x,y,z,rx,ry,rz]`
  - speed`int`: 1 ~ 100
- **返回值:** `int` 1

#### `pause()`

- **功能:** 控制核心暂停进行的指令，可以停止所有的移动指令

#### `is_paused()`

- **功能:** 检测程序是否暂停了移动的指令
- **返回值:**
  - `1` - 暂停
  - `0` - 未暂停
  - `-1` - 错误

#### `resume()`

- **功能:** 程序恢复移动指令，继续完成上一个指令

#### `stop()`

- **功能:** 控制核心停止所有的移动指令，程序停止
- **返回值**:
  - `1` - 停止
  - `0` - 未停止
  - `-1` - 错误

#### `is_in_position(data, flag)`

- **功能** : 是否到达点位
- **参数:**
  - data:  提供一组数据，可以是角度或坐标值
  - flag data type (value range 0 or 1)
    - `0`: 角度
    - `1`: 坐标
- **返回值**:
  - `1` - true
  - `0` - false
  - `-1` - 错误

#### `is_moving()`

- **功能:** 检测机器人是否在运动
- **返回值:**
  - `1` - 正在运动
  - `0` - 未在运动
  - `-1` - 错误

### 5. JOG 模式和操作

#### `jog_angle(joint_id, direction, speed)`

- **功能:** 关节控制
- **参数**:
  - `joint_id`: 代表机械臂的关节，由 1 至 6 个关节 ID 表示
  - `direction(int)`: 要控制机械臂的运动方向，输入 `0 `表示负值运动，输入 `1 `表示正值运动
  - `speed`: 1 ~ 100
- **返回值:** `int` 1

#### `jog_coord(coord_id, direction, speed)`

- **功能:** 坐标控制
- **参数:**
  - `coord_id`: (`int`) 机械臂关节，范围： 1~6
  - `direction`: (`int`) 控制机械臂运动方向
    -  `0` - 负值运动
    - `1` - 正值运动
  - `speed`: 1 ~ 100
- **返回值:** `int` 1

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能:** 关节步进模式
- **参数**:
  - `joint_id`: 1 - 6
  - `increment`: 基于当前位置角度的增量运动
  - `speed`: 1 ~ 100
- **返回值:** `int` 1

#### `jog_increment_coord(coord_id, increment, speed)`

- **功能:** 坐标步进模式
- **参数**:
  - `joint_id`:  1 - 6.
  - `increment`: 基于当前位置坐标的增量移动
  - `speed`: 1 ~ 100
- **返回值:** `int` 1

### 6. 软件关节限位

#### `get_joint_min_angle(joint_id)`

- **功能:** 读取关节最小角度
- **参数:**
  - `joint_id` : joint ID： 1 - 6
- **返回值**：`float` 角度值

#### `get_joint_max_angle(joint_id)`

- **功能:** 读取关节最大角度
- **参数:**
  - `joint_id` :  joint ID： 1 - 6
- **返回值:** `float` 角度值

#### `set_joint_min(id, angle)`

- **功能:** 设置关节最小角度
- **参数:**
  - `id` :  joint ID：1 - 6
  - `angle`: 请参考 send_angle() 界面中相应关节的极限信息，其值不得小于最小值

#### `set_joint_max(id, angle)`

- **功能:** 设置关节最大角度
- **参数:**
  - `id` : joint ID：1 - 6
  - `angle`:请参阅 send_angle() 界面中相应关节的极限信息，该信息不得大于最大值

### 7. 关节电机辅助控制

#### `is_servo_enable(servo_id)`

- **功能:** 检测关节链接状态
- **参数:** `servo id` 1-6
- **返回值:**
  - `1`: 连接成功
  - `0`: 未连接
  - `-1`: 错误

#### `is_all_servo_enable()`

- **功能:** 检测所有关节连接状态
- **返回值:**
  - `1`: 连接成功
  - `0`: 未连接
  - `-1`: 错误

#### `set_servo_calibration(servo_id)`

- **功能:** 设置舵机0点
- **参数**:
  - `servo_id`: 1 - 6

#### `release_servo(servo_id)`

- **功能:** 单个电机掉电
- **参数**:
  - `servo_id`: 1 ~ 6
- **返回值:**
  - `1`: 释放成功
  - `0`: 释放失败
  - `-1`: 错误

#### `focus_servo(servo_id)`

- **功能**: 单个电机上电
- **参数**: `servo_id`: 1 ~ 6
- **返回值:**
  - `1`: 上电成功
  - `0`: 上电失败
  - `-1`: 错误

#### `release_all_servos()`

- **功能:** 机器人关闭力矩输出
  
  - 注意：关节失效后，需要在 1 秒内启用才能进行控制

- **返回值:**
  
  - `1` - 完成
  - `0` - 失败

#### `focus_all_servos()`

- **功能:** 机器人打开力矩输出
- **返回值:**
  - `1`: 完成
  - `0`: 失败
  - `-1`: 错误

#### `set_break（joint_id, value）`

- **功能:** 关闭/打开刹车
- **参数**：
  - `joint_id`: joint id 1 - 6
  - `value`: `int` 
    - `0` - 关闭
    - `1` - 打开
- **返回值:** 
  - `0` : 失败;
  - `1` : 成功

#### `get_servo_speeds()`

- **功能**：获取关节运行速度
- **返回值**：单位：step/s

#### `get_servo_currents()`

- **功能**：获取关节电流
- **返回值**： 0 ~ 5000 mA

#### `get_servo_status()`

- **功能**：获取关节状态
- **返回值**： `0` 表示一切正常

### 8. 拖动示教

#### `drag_teach_save()`

- **功能:** 启动拖动示教
  - 注意：为了显示最佳运动效果，录制时间不应超过 90 秒

#### `drag_teach_pause()`

- **功能:** 暂停采样

#### `drag_teach_execute()`

- **功能:** 执行示教点位

#### `drag_teach_clean()`

- **功能:** 清除采样

### 9. 动力学

#### `set_collision_mode(mode)`

- **功能:**碰撞检测开关
- **参数:**
  - `mode` `int`: 0 / 1

#### `set_collision_threshold(joint_id, value = 100)`

- **功能:**设置碰撞阈值
- **参数:**
  - `joint_id` `int` 1 - 6
  - `value` `int` 50 ~ 250

#### `get_collision_threshold()`

- **功能:** 获取碰撞阈值
- **返回值:** 范围: 50 ~ 250

#### `set_torque_comp(joint_id, value=100)`

- **功能:** 设置力矩补偿系数
- **参数:**
  - `joint_id` (int) 1 ~ 6
  - `value` (int) 0 ~ 250

#### `get_torque_comp()`

- **功能:** 获取力矩补偿系数
- **返回值:** 范围: 0 ~ 250

### 10. 算法参数

`get_vr_mode()`

- **功能**: 获取VR模式

- **返回值:** `0` 开启; `1` 关闭

`set_vr_mode(mode)`

- **功能:** 设置VR模式

- **参数:**
  
  - `mode` `int` 0 / 1

`get_model_direction()`

- **功能:** 获取关节模型方向

- **返回值:** `tuple[bytearray, int]`

`set_model_direction(id)`

- **功能:** 设置关节模型方向

- **参数:**
  
  - `id` `int` 1 ~ 6
  
  - `direction` `int` 0/1

`get_filter_len(rank)`

- **功能:** 获取滤波器参数

- **参数:** rank(int)
  
  - `1` 拖动教学抽样过滤器
  
  - `2` 拖动教学执行滤波器
  
  - `3` 关节速度融合滤波器
  
  - `4` 坐标速度融合滤波器
  
  - `5` 拖动教学采样周期

`set_filter_len(rank, value)`

- **功能:** 设置滤波器参数

- **参数:**
  
  - rank: `int`
    
    - ``1` 拖动教学抽样过滤器
    
    - `2` 拖动教学执行滤波器
    
    - `3` 关节速度融合滤波器
    
    - `4` 坐标速度融合滤波器
    
    - `5` 拖动教学采样周期
  
  - value: `int` 1 ~ 100

### 11. 底部IO输入输出

#### `set_basic_output(pin_no, pin_signal)`

- **功能**：设置底座IO输出
- **参数**：
  - `pin_no` (`int`) Pin值
  - `pin_signal` (`int`): `0` 低 `1` 高

#### `get_basic_input(pin_no)`

- **功能:** 读取底座IO输入
- **参数:**
  - `pin_no` (`int`) pin值
- **返回值:** `0` 低 `1` 高

### 12. 末端夹爪控制

#### `set_gripper_state(flag, speed, _type_1=None)`

- **功能**: 设置夹爪开合状态

- **参数**:
  
  - `flag (int)` : 0 - 开启1 - 关闭, 254 - 释放
  
  - `speed (int)`: 1 ~ 100
  
  - `_type_1 (int)`:
    
    - `1` : 自适应抓夹

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **功能**: 设置夹爪角度

- **参数**:
  
  - `gripper_value (int)` : 0 ~ 100
  - `speed (int)`: 1 ~ 100
  - `gripper_type (int)`:
    - `1` : 自适应抓手（默认状态为 1）
    - `2` : 五指灵巧手
    - `3` : 平行机械手（可省略此参数）
    - `4` : 灵活机械手

#### `set_gripper_calibration()`

- **功能**: 夹爪设置0点

#### `set_gripper_enabled(value)`

- **功能**: pro自适应夹爪使能
- **参数**:
  - `value` 
    - `1`  - Enable 
    - `0`  - Release

#### `set_gripper_mode(mode)`

- **功能**: 设置夹爪模式
- **参数**:
  - `value` :
    - `0`  - 透明传输模式
    - `1`  - 正常模式

#### `get_gripper_mode()`

- **功能**: 获取夹爪模式
- **返回值**:
  - `0`  - 透明传输模式
  - `1`  - 正常模式

### 13. atom末端RGB

#### `set_color(r, g, b)`

- **功能**: 设置RGB颜色
- **参数**:
  - `r (int)`: 0 ~ 255
  - `g (int)`: 0 ~ 255
  - `b (int)`: 0 ~ 255

### 14. 末端IO控制

#### `set_digital_output(pin_no, pin_signal)`

- **功能:** 设置IO
- **参数**
  - `pin_no` (int): Pin number
  - `pin_signal` (int): 0 / 1

#### `get_digital_input(pin_no)`

- **功能:** 读取IO
- **参数**: `pin_no` `int`
- **返回值**: 信号值

#### `is_btn_clicked()`

- **功能**: 末端led按钮是否按下
- **返回值**:
  - `0`  - 未按下
  - `1`  - 按下

### 15. 工具坐标系操作

#### `get_world_reference()`

- **功能:** 获取世界坐标系
- **返回值:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **功能:** 设置基坐标系
- **参数：**`rftype`:
  - `0` - base 
  - `1` - tool.

#### `get_reference_frame()`

- **功能:** Set base coordinate system.
- **返回值:**
  - `0` - base
  - `1` - tool

#### `set_movement_type(move_type)`

- **功能:** 获取基坐标系
- **参数**：
  - `move_type`: 
    - `1` - movel
    - `0` - moveJ

#### `get_movement_type()`

- **功能:** 设置移动类型
- **返回值:**
  - `1` - movel
  - `0` - moveJ

#### `set_end_type(end)`

- **功能:** 设置末端坐标系
- **参数:**
  - `end (int)`:
    - `0` - flange
    - `1` - tool

#### `get_end_type()`

- **功能:** 获取末端坐标系
- **返回值:**
  - `0` - flange
  - `1` - tool


### 16. 同步/异步 控制模式

#### `set_pos_switch(mode)`

- **功能:** 设置控制模式
- **参数:**
  - `mode (int)`: `0` - 异步, `1` - 同步

#### `get_pos_switch()`

- **功能:** 获取控制模式
- **返回值:**
  - `0` - 异步
  - `1` - 同步

# 2. 使用方法

## 串口通信

1. 机器人首次开机时，需要先通电才能进行控制，使用`power_on`命令可以控制机器人通电。

```python

from pymycobot import Pro630

p = Pro630("/dev/ttyAMA0")
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

2. 如果使用过程中按下并松开急停按钮，则需要断电后再通电。

```python

from pymycobot import Pro630
import time
p = Pro630("/dev/ttyAMA0")
p.power_off()
time.sleep(1)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

3. **控制模式切换**：可以使用`set_pos_switch`接口进行模式设置：
   1. **异步模式**：发送角度或坐标后函数就会立即结束，不会等待机器人移动到目标位置。
   2. **同步模式**：送角度或坐标后，机器人移动到目标位置以后，控制函数就会结束。

> 注意：请在执行运动控制之前，使用`set_pos_switch`设置控制模式.

```python

from pymycobot import Pro630

p = Pro630("/dev/ttyAMA0")
# asynchronous mode
p.set_pos_switch(0)
# synchronous mode
# p.set_pos_switch(1)
p.send_angle(5, 100, 20)

```

## TCP/IP 通信

### 服务端

将[服务端](https://github.com/elephantrobotics/pymycobot/blob/pro630_esp32/demo/Server_Pro630.py)下载到机器人的系统内，并且运行起来.

```shell
python3 Server_Pro630.py
```

运行以后，会显示出服务端的IP地址以及绑定的端口号：

```shell
pi@raspberrypi:~/pymycobot/demo $ python3 Server_Pro630.py 
ip: 192.168.1.169 port: 9000
Binding succeeded!
This is asynchronous mode
waiting connect!------------------

```

### 客户端

将`pymycobot`更新至v3.5.0a2及以后版本:

```shell
pip uninstall pymycobot
pip install pymycobot==3.5.0a2
```

1. 机器人首次开机时，需要先通电才能进行控制，使用`power_on`命令可以控制机器人通电。

```python

from pymycobot import Pro630Client

p = Pro630Client("192.168.1.169", 9000)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

2. 如果使用过程中按下并松开急停按钮，则需要断电后再通电。

```python

from pymycobot import Pro630Client

import time
p = Pro630Client("192.168.1.169", 9000)

p.power_off()
time.sleep(1)
res = 0
res = p.power_on()
if res == 1:
    print("power on succ")
```

3. **控制模式切换**：可以使用`set_pos_switch`接口进行模式设置：
   1. **异步模式**：发送角度或坐标后函数就会立即结束，不会等待机器人移动到目标位置。
   2. **同步模式**：送角度或坐标后，机器人移动到目标位置以后，控制函数就会结束。

> 注意：请在执行运动控制之前，使用`set_pos_switch`设置控制模式.

```python

from pymycobot import Pro630Client

p = Pro630Client("192.168.1.169", 9000)

# 异步模式
p.set_pos_switch(0)
# 同步模式
# # p.set_pos_switch(1)
p.send_angle(5, 100, 20)

```

# 打包姿态

打包角度为[90, -38, 128, 180, -90, 0], 控制机器人运动到打包角度即可。

```python

from pymycobot import Pro630Client

p = Pro630("/dev/ttyAMA0")
p.send_angles([90, -38, 128, 180, -90, 0], 10)
```
