# API 方法详细说明

[toc]

**在使用下列函数接口的时候请先在开头导入我们的API库，否则无法运行成功，即输入以下代码：**

```python
from pymycobot import UltraArmP1
```

**注意：** 若没有安装我们的API库，请参考 [README.md](../README.md) 文档进行安装。

## 1 `set_reboot()`

- **功能：** 设置机械臂开发板重启。
- **返回值：** 无

## 2 `set_joint_release()`

- **功能：** 放松关节
- **返回值：** 无

## 3 `set_joint_enable()`

- **功能：** 锁紧关节

- **返回值：** 无

## 4 `get_system_version()`

- **功能：** 读取固件主版本号

- **返回值：** `float`, 更正版本号

## 5 `get_modify_version()`

- **功能：** 读取固件更正版本号

- **返回值：** `int`, 更正版本号

## 6 `get_angles_info()`

- **功能：** 获取机械臂当前角度。
- **返回值：** `list`一个浮点值的列表，表示所有关节的角度. [J1, J2, J3, J4]

## 7 `set_angle(id, angle, speed, _async=True)`

- **功能：** 发送指定的单个关节运动至指定的角度
- **参数说明：**
  - `id`: 代表机械臂的关节，用数字1-4来表示。
  - `degree`: 表示关节的角度
  
      <table>
        <tr>
            <th>关节 Id</th>
            <th>范围</th>
        </tr>
        <tr>
            <td text-align: center>1</td>
            <td>-160 ~ 160</td>
        </tr>
        <tr>
            <td>2</td>
            <td>-20 ~ 85</td>
        </tr>
        <tr>
            <td>3</td>
            <td>90 ~ 200</td>
        </tr>
        <tr>
            <td>4</td>
            <td> -180 ~ 180</td>
        </tr>

    </table>

  - `speed`：表示机械臂运动的速度，范围 1~5700 。
  - `_async`: 运动到位反馈，默认开启。

- **返回值：** 闭环返回 "ok"，开环返回1

## 8 `set_angles(angles, speed, _async=True)`

- **功能：**  发送所有角度给机械臂所有关节
- **参数说明：**
  - `degrees`: (List[float])包含所有关节的角度 ,四轴机器人有四个关节，所以长度为4，表示方法为：[20,20,90, 20]
  - `speed`: 表示机械臂运动的速度，取值范围是1~5700。
  - `_async`: 运动到位反馈，默认开启。
- **返回值：** 闭环返回 "ok"，开环返回1

## 9 `get_coords_info()`

- **功能：** 获取机械臂当前坐标。
- **返回值：** `list`包含坐标的列表, 长度为 4，依次为 `[x, y, z, rx]`

## 10 `set_coords_max_speed(coords, _async=True)`

- **功能：** 以最大速度发送坐标运动
- **参数说明：**
  - `coords`: 长度为4或者3的坐标列表，[X, Y, Z, RX] 或者 [X, Y, Z]
  
      <table>
        <tr>
            <th>坐标 Id</th>
            <th>范围</th>
        </tr>
        <tr>
            <td text-align: center>X</td>
            <td>-301.7 ~ 362.7</td>
        </tr>
        <tr>
            <td>Y</td>
            <td>-362.7 ~ 362.7</td>
        </tr>
        <tr>
            <td>Z</td>
            <td>-157 ~ 91</td>
        </tr>
        <tr>
            <td>Rx</td>
            <td>-180 ~ 180</td>
        </tr>

      </table>

  - `_async`: 运动到位反馈，默认开启。
- **返回值：** 闭环返回 "ok"，开环返回1

## 11 `set_coords(coords, speed, _async=True)`

- **功能：** 发送整体坐标,让机械臂头部从原来点移动到您指定点。
- **参数说明：**
  - `coords`: 长度为4或者3的坐标列表，[X, Y, Z, RX] 或者 [X, Y, Z]

      <table>
        <tr>
            <th>坐标 Id</th>
            <th>范围</th>
        </tr>
        <tr>
            <td text-align: center>X</td>
            <td>-301.7 ~ 362.7</td>
        </tr>
        <tr>
            <td>Y</td>
            <td>-362.7 ~ 362.7</td>
        </tr>
        <tr>
            <td>Z</td>
            <td>-157 ~ 91</td>
        </tr>
        <tr>
            <td>Rx</td>
            <td>-180 ~ 180</td>
        </tr>

      </table>

  - `speed`: 表示机械臂运动的速度，范围是1-5700。
  - `_async`: 运动到位反馈，默认开启。
- **返回值：** 闭环返回 "ok"，开环返回1

## 12 `stop()`

- **功能：** 机械臂停止运动

- **返回值：** 1

## 13 `set_jog_angle(joint_id, direction, speed, _async=True)`

- **功能：** 设置JOG角度运动

- **参数说明：**

  - `joint_id`: 代表机械臂的关节，范围 1 ~ 4

  - `direction`: 主要控制机器臂移动的方向，1 - 正向移动，0 - 负向移动

  - `speed`: 速度 1 ~ 5700。

  - `_async`: 运动到位反馈，默认打开。

- **返回值：** 闭环返回 "ok"，开环返回1

## 14 `set_jog_coord(axis_id, direction, speed, _async=True)`

- **功能：** 设置JOG坐标运动。

- **参数说明：**

  - `axis_id`: 代表机械臂的关节坐标，范围 1 ~ 4

  - `direction`: 主要控制机器臂移动的方向，1 - 正向移动，0 - 负向移动

  - `speed`: 速度 1 ~ 5700 。

  - `_async`: 运动到位反馈，默认打开。

- **返回值：** 闭环返回 "ok"，开环返回1

## 15 `jog_increment_angle(joint_id, increment, speed, _async=True)`

- **功能：** 设置角度步进运动

- **参数说明：**

  - `joint_id`: 代表机械臂的关节，范围 1 ~ 4

  - `increment`: 角度增量值。

  - `speed`: 速度 1 ~ 5700。

  - `_async`: 运动到位反馈，默认打开。

- **返回值：** 闭环返回 "ok"，开环返回1

## 16 `jog_increment_coord(coord_id, increment, speed, _async=True)`

- **功能：** 设置坐标步进运动。

- **参数说明：**

  - `axis_id`: 代表机械臂的关节坐标，范围 1 ~ 4

  - `increment`: 坐标增量值。

  - `speed`: 速度 1 ~ 5700 。

  - `_async`: 运动到位反馈，默认打开。

- **返回值：** 闭环返回 "ok"，开环返回1

## 17 `get_error_information()`	

- **功能：** 读取错误信息

- **返回值：** 错误信息

## 18 `set_pwm(p_value)` 

- **功能：** 设置PWM控制

- **参数说明：**  `p_value` 占空比，范围：0-5

- **返回值：** 1

## 19 `set_zero_calibration(joint_number)` 

- **功能：** 设置零位校准

- **参数：：** 
  - `joint_number` `(int)` : 0 ~ 4
    - 0 : All joint
    - 1: J1
    - 2: J2
    - 3: J3
    - 4: J4

- **返回值：** 1

## 19 `get_zero_calibration_state(joint_number)` 

- **功能：** 读取零位校准状态

- **返回值：**  `list` [1, 1, 1, 1]

## 20 `get_run_status()` 

- **功能：** 读取运行状态

- **返回值：**  运行状态

## 21 `open_laser()`

- **功能：** 打开激光

- **返回值：** 1

## 22 `close_laser()`

- **功能：** 关闭激光

- **返回值：** 1

## 23 `set_gripper_angle(gripper_angle, gripper_speed)`

- **功能：** 设置夹爪运动角度

- **参数说明:** 
  - `gripper_angle`： `int`, 1 ~ 100。

  - `gripper_speed:` 1 ~ 100

- **返回值：** 1

## 24 `get_gripper_angle()`

- **功能：** 读取夹爪角度

- **返回值：** 夹爪角度，1 ~ 100

## 25 `set_gripper_parameter(addr, mode, parameter_value)`

- **功能：** 设置夹爪参数

- **参数说明:** 
  - `addr`： `int`, 1 ~ 69

  - `mode:`  (`int`): 1 ~ 2
  - `parameter_value` (`int`):
    - `模式为1`：0 ~ 255
    - `模式为2`： 大于255

- **返回值：** 1

## 26 `get_gripper_parameter(addr, mode)`

- **功能：** 读取夹爪参数

- **参数说明:** 
  - `addr`： `int`, 1 ~ 69

  - `mode:`  (`int`): 1 ~ 2

- **返回值：** (int) 夹爪参数
  - `模式为1`：0 ~ 255
  - `模式为2`： 大于255

## 27 `set_gripper_enable_status(state):`

- **功能：** 设置夹爪参数

- **参数说明:** 
  - `state`： `int`
    - `0`: 失能
    - `1`: 使能

- **返回值：** 1

## 28 `set_gripper_zero()`

- **功能：** 设置夹爪零位

- **返回值：** 1

## 29 `set_pump_state(pump_state)`

- **功能：** 设置吸泵状态

- **参数说明:** 
  - `pump_state`： `int`
    - `0`: 打开
    - `1`: 释放
    - `2`: 关闭

- **返回值：** 1

## 30 `set_basic_io_output(pin_no, pin_signal)`

- **功能：** 设置底座IO引脚输出状态

- **参数说明:** 
  - `pin_no`： `int` 1 ~ 10
  - `pin_signal`： `int`
    - `0`: 低电平
    - `1`: 高电平

- **返回值：** 1

## 31 `set_digital_io_output(pin_no, pin_signal)`

- **功能：** 设置末端IO引脚输出状态

- **参数说明:** 
  - `pin_no`： `int` 1 ~ 4
  - `pin_signal`： `int`
    - `0`: 低电平
    - `1`: 高电平

- **返回值：** 1

## 32 `set_outer_shaft(shaft_state, speed)`

- **功能：** 设置外部轴

- **参数说明:** 
  - `shaft_state`： `int`
    - `0`: 关闭
    - `1`: 打开
  - `speed`： `int` 1 ~ 5700

- **返回值：** 1

## 33 `set_i2c_data(data_state, data_addr, data_len, data_value)`

- **功能：** 设置i2c数据

- **参数说明：**

  - `data_state`: `int` 0 ~ 1
    - `0`: 读
    - `1`: 写

  - `data_addr`: `int` 0 ~ 255

  - `data_len`: `int`: 0 ~ 64
  - `data_value`: `int` 0 ~ 255

- **返回值：** 1

## 34 `drag_teach_start()`

- **功能：** 拖动示教开始

- **返回值：** 1

## 35 `drag_teach_save()`

- **功能：** 拖动示教保存

- **返回值：** 1

## 36 `drag_teach_pause()`

- **功能：** 拖动示教暂停

- **返回值：** 1

## 37 `drag_teach_resume()`

- **功能：** 拖动示教恢复

- **返回值：** 1

## 38 `drag_teach_stop()`

- **功能：** 拖动示教停止

- **返回值：** 1

## 39 `drag_teach_execute()`

- **功能：** 拖动示教执行

- **返回值：** 1

## 40 `wifi_open()`

- **功能：** WIFI开启

- **返回值：** 1

## 41 `get_system_screen_version()`

- **功能：** 读取屏幕固件主版本号

- **返回值：** 主版本号

## 42 `get_modify_screen_version()`

- **功能：** 读取屏幕固件更正版本号

- **返回值：** 更正版本号

## 43 `set_communication_baud_rate(baud_rate)`

- **功能：** 设置通信波特率

- **参数说明:** 
  - `baud_rate`： `int` 标准波特率, 115200 or 1000000

- **返回值：** 1

## 44 `update_stm_firmware()`

- **功能：** 更新STM32固件

- **返回值：** 1

## 45 `receive_485_data()`

- **功能：** 接收485数据

- **返回值：** 485数据

## 46 `play_gcode_file(filename)`

- **功能：** 播放导入的轨迹文件。
- **参数说明：**
  - `filename` ：轨迹文件名称
- **返回值：** 无

## 47 `set_wifi_password(password)`

- **功能：** 设置屏幕WiFi密码。
- **参数说明：**
  - `password` ：(`str`) WiFi字符串密码，长度 8 ~ 15位。
- **返回值：** 1

## 48 `check_sd_card()`

- **功能：** 检查是否有SD卡。
- **返回值：** (`str`)
  - `"yes"`: 有SD卡
  - `"no`: 无SD卡

## 49 `download_firmware_sd(filename, show_progress=True)`

- **功能：** 下载固件数据到SD卡中。
- **参数说明：**
  - `filename` ：(`str`) 固件文件的名称，且必须是 .bin 文件（建议用一个固定名称，避免SD卡存过多文件）。
  - `show_progress` ：(`bool`) 是否显示下载进度，默认显示。
- **返回值：** 如果 `show_progress=True`，则返回下载进度，否则无返回值。

## 50 `upgrade_restart()`

- **功能：** 固件升级重启。
- **返回值：** None

## 51 `get_motor_enable_status()`

- **功能：** 读取电机使能状态。

- **返回值：** `list`，5个电机使能状态。

## 52 `clear_zero_calibration_status(joint_id)`

- **功能：** 清除关节零位校准状态。
- **参数说明：**
  - `joint_id` ：(`int`) 关节ID，范围 1 ~ 4 。
- **返回值：** 1

## 53 `set_status_light_color(color_id)`

- **功能：** 设置灯状态颜色
- **参数说明：**
  - `color_id` ：(`int`) 颜色ID，范围 1 ~ 4 。1-红色，2-绿色，3-黄色，4-蓝色。
  
- **返回值：** 1
