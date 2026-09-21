# 6.1 Python API

[toc]

## 6.1.2 API 使用说明

使用以下接口前，请先导入 API：

```python
from pymycobot import Mercury

mc = Mercury('/dev/ttyAMA1')
print(mc.get_angles())
```

### 1. 系统状态

#### `get_system_version()`

- **功能：** 获取系统版本。
- **返回值：** 系统版本。

#### `get_robot_type()`

- **功能：** 获取机器人 ID。
- **返回值：** 实际机型编号；例如 Mercury A1 为 `4500`。

#### `get_atom_version()`

- **功能：** 获取末端版本号。
- **返回值：** 末端参数（`float`）。

#### `get_atom_modify_version()`

- **功能：** 获取末端修订版本号。
- **返回值：** 末端参数（`int`）。

#### `get_robot_status()`

- **功能：** 获取上位机错误安全状态。
- **返回值：** `0` 表示正常；其他值表示机器人触发碰撞检测。

#### `get_err_protect_status()`

- **功能：** 读取电机异常断电保护开关状态。
- **返回值：** `1` 表示开启（默认）；`0` 表示关闭。

#### `set_err_protect_status(status)`

- **功能：** 设置电机异常断电保护开关。此设置断电后不保存。
- **参数：** `status`（`int`）：`1` 开启；`0` 关闭。
- **返回值：** `1` 表示设置成功；`0` 表示设置失败。
- **说明：** 开启后，检测到电机通信断开、过流等异常时，会中断当前任务并执行保护。

### 2. 整体状态

#### `power_on()`

- **功能：** 使 Atom 开启通信（默认开启）。
- **注意：** 执行断电或按下急停后，需约 7 秒恢复供电。
- **返回值：** `1` 成功；`0` 失败。

#### `power_off()`

- **功能：** 关闭机械臂电源。
- **返回值：** `1` 成功；`0` 失败。

#### `is_power_on()`

- **功能：** 查询机械臂是否上电。
- **返回值：** `1` 已上电；`0` 已断电；`-1` 数据错误。

#### `release_all_servos()`

- **功能：** 释放所有关节。
- **参数：** 可选 `data` 指定释放方式；默认阻尼模式，`1` 为非阻尼模式。
- **返回值：** `1` 成功；`0` 失败。

#### `focus_all_servos()`

- **功能：** 开启机械臂扭矩输出。
- **返回值：** `1` 成功；`0` 失败；`-1` 数据错误。

#### `get_fresh_mode()`

- **功能：** 查询运动模式。
- **返回值：** `0` 插补模式；`1` 刷新模式。

#### `set_fresh_mode(mode)`

- **功能：** 设置命令刷新模式。
- **参数：** `1` 始终优先执行最新命令；`0` 按队列顺序执行命令。

### 3. MDI 模式与操作

#### `get_angles()`

- **功能：** 获取全部关节角度。
- **返回值：** 长度为 7 的浮点角度列表。

#### `get_angle(joint_id)`

- **功能：** 获取单关节角度。
- **参数：** `joint_id`（`int`）：1 ~ 7。
- **返回值：** 对应关节角度。

#### `send_angle(id, degree, speed)`

- **功能：** 控制单关节运动。
- **参数：** `id` 为关节 ID（1 ~ 7）；`degree` 为目标角度；`speed` 为 1 ~ 100。
- **关节范围：** J1 `-178 ~ 178`，J2 `-74 ~ 130`，J3 `-178 ~ 178`，J4 `-180 ~ 10`，J5 `-178 ~ 178`，J6 `-20 ~ 273`，J7 `-180 ~ 180`。

#### `send_angles(angles, speed)`

- **功能：** 控制全部关节运动。
- **参数：** `angles` 为长度 7 的角度列表；`speed` 为 1 ~ 100。

#### `get_coords()`

- **功能：** 获取基坐标系下的机械臂坐标。
- **返回值：** `[x, y, z, rx, ry, rz]` 浮点列表。

#### `send_coord(id, coord, speed)`

- **功能：** 控制单个坐标轴运动。
- **参数：** `id` 为 1 ~ 6，对应 `[x, y, z, rx, ry, rz]`；`speed` 为 1 ~ 100。
- **坐标范围：** X/Y `-466 ~ 466`，Z `-240 ~ 531`，RX/RY/RZ `-180 ~ 180`。

#### `send_coords(coords, speed, mode)`

- **功能：** 控制末端整体坐标和姿态运动。
- **参数：** `coords` 为 `[x, y, z, rx, ry, rz]`；`speed` 为 1 ~ 100。

#### `pause(deceleration=False)`

- **功能：** 暂停核心与全部运动命令。
- **参数：** `deceleration`：是否减速停止，默认 `False`。
- **返回值：** `1` 已暂停；`0` 未暂停；`-1` 数据错误。

#### `is_paused()`

- **功能：** 查询运动命令是否暂停。
- **返回值：** `1` 已暂停；`0` 未暂停；`-1` 数据错误。

#### `resume()`

- **功能：** 恢复机械臂运动并继续先前命令。

#### `stop(deceleration=False)`

- **功能：** 停止机械臂全部运动。
- **参数：** `deceleration`：是否减速停止，默认 `False`。
- **返回值：** `1` 已停止；`0` 未停止；`-1` 数据错误。

#### `is_in_position(data, flag)`

- **功能：** 判断是否到达指定位置。
- **参数：** `data` 为长度 7 的角度或长度 6 的坐标；`flag`：`0` 角度，`1` 坐标。
- **返回值：** `1` 是；`0` 否；`-1` 数据错误。

#### `is_moving()`

- **功能：** 查询机械臂是否正在运动。
- **返回值：** `1` 运动中；`0` 未运动；`-1` 数据错误。

### 4. JOG 模式与操作

#### `jog_angle(joint_id, direction, speed)`

- **功能：** 点动控制关节。
- **参数：** `joint_id` 为 1 ~ 7；`direction`：`0` 负向、`1` 正向；`speed` 为 1 ~ 100。

#### `jog_coord(coord_id, direction, speed)`

- **功能：** 点动控制坐标轴。
- **参数：** `coord_id` 为 1 ~ 6；`direction`：`0` 负向、`1` 正向；`speed` 为 1 ~ 100。

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能：** 按增量点动关节。
- **参数：** `joint_id` 为 1 ~ 7；`increment` 为角度增量；`speed` 为 1 ~ 100。

#### `jog_increment_coord(coord_id, increment, speed)`

- **功能：** 按增量点动坐标轴。
- **参数：** `coord_id` 为 1 ~ 6；`increment` 为坐标增量；`speed` 为 1 ~ 100。

### 5. 坐标控制姿态偏移角

#### `get_solution_angles()`

- **功能：** 获取零空间偏转角。

#### `set_solution_angles(angle, speed)`

- **功能：** 设置零空间偏转角。
- **参数：** `angle` 为 J1 角度，范围 `-90 ~ 90`；`speed` 为 1 ~ 100。

### 6. 关节软件限位操作

#### `get_joint_min_angle(joint_id)` / `get_joint_max_angle(joint_id)`

- **功能：** 读取指定关节的软件最小/最大角度。
- **参数：** `joint_id` 为 1 ~ 7。
- **返回值：** `float` 角度值。

#### `set_joint_min(id, angle)` / `set_joint_max(id, angle)`

- **功能：** 设置指定关节的软件最小/最大限位。
- **参数：** `id` 为 1 ~ 7；`angle` 必须处于该关节的合法角度范围。

### 7. 关节电机控制

#### `is_servo_enable(servo_id)` / `is_all_servo_enable()`

- **功能：** 查询指定关节或全部关节的连接状态。
- **参数：** `servo_id` 为 1 ~ 7。
- **返回值：** `1` 已连接；`0` 未连接；`-1` 数据错误。

#### `set_servo_calibration(servo_id)`

- **功能：** 将当前指定关节执行器位置校准为零点。
- **参数：** `servo_id` 为 1 ~ 7。

#### `release_servo(servo_id)` / `focus_servo(servo_id)`

- **功能：** 关闭/开启指定关节的扭矩输出。
- **参数：** `servo_id` 为 1 ~ 7。
- **返回值：** `1` 成功；`0` 失败；`-1` 数据错误。

#### `set_break(joint_id, value)`

- **功能：** 设置断点。
- **参数：** `joint_id` 为 1 ~ 7；`value`：`0` 禁用，`1` 启用。
- **返回值：** `1` 成功；`0` 失败。

#### `get_servo_speeds()` / `get_servo_currents()` / `get_servo_status()`

- **功能：** 分别获取全部关节运动速度、电流和状态。
- **返回值：** 速度单位为 step/s；电流范围为 0 ~ 5000 mA；状态值 `0` 表示无错误。

#### `servo_restore(joint_id)`

- **功能：** 清除指定关节异常。
- **参数：** `joint_id` 为 1 ~ 7。

### 8. 机械臂末端 IO 控制

#### `set_digital_output(pin_no, pin_signal)` / `get_digital_input(pin_no)`

- **功能：** 设置末端数字输出/读取末端数字输入。
- **参数：** `pin_no` 为引脚号；`pin_signal`：`0` 或 `1`。

### 9. 机械臂末端夹爪控制

#### `set_gripper_state(flag, speed, _type_1=None)`

- **功能：** 设置自适应夹爪开合状态。
- **参数：** `flag`：`0` 张开、`1` 闭合、`254` 释放；`speed` 为 1 ~ 100；`_type_1`：`1` 自适应夹爪、`2` 五指灵巧手、`3` 平行夹爪、`4` 柔性夹爪。

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **功能：** 设置夹爪开度。
- **参数：** `gripper_value` 为 0 ~ 100；`speed` 为 1 ~ 100；`gripper_type` 的类型定义同上。

#### `set_gripper_calibration()`

- **功能：** 将夹爪当前位置设置为零点。

#### `set_gripper_enabled(value)`

- **功能：** 设置自适应夹爪使能状态。
- **参数：** `1` 使能；`0` 释放。

#### `set_gripper_mode(mode)` / `get_gripper_mode()`

- **功能：** 设置/读取夹爪模式。
- **参数与返回值：** `0` 透传模式；`1` 普通模式。

### 10. 机械臂末端按键功能

#### `is_btn_clicked()`

- **功能：** 获取末端按键状态。
- **返回值：** `0` 未按下；`1` 已按下。

#### `set_color(r, g, b)`

- **功能：** 设置机械臂末端灯颜色。
- **参数：** `r`、`g`、`b`（`int`）均为 0 ~ 255。

### 11. 拖动示教

#### `drag_teach_save()` / `drag_teach_pause()` / `drag_teach_execute()`

- **功能：** 开始记录拖动示教点、暂停采样、执行一次已记录的示教轨迹。
- **注意：** 为保证运动效果，记录时长不应超过 90 秒。

### 12. 笛卡尔空间坐标参数设置

#### `set_tool_reference(coords)` / `get_tool_reference()`

- **功能：** 设置/读取工具坐标系。
- **参数与返回值：** `coords` 为 `[x, y, z, rx, ry, rz]`。

#### `set_world_reference(coords)` / `get_world_reference()`

- **功能：** 设置/读取世界坐标系。
- **参数与返回值：** `coords` 为 `[x, y, z, rx, ry, rz]`。

#### `set_reference_frame(rftype)` / `get_reference_frame()`

- **功能：** 设置/读取参考坐标系。
- **参数与返回值：** `0` 基坐标系；`1` 工具坐标系。

#### `set_movement_type(move_type)` / `get_movement_type()`

- **功能：** 设置/读取运动类型。
- **参数与返回值：** `1` 为 movel；`0` 为 moveJ。

#### `set_end_type(end)` / `get_end_type()`

- **功能：** 设置/读取末端坐标系类型。
- **参数与返回值：** `0` 法兰；`1` 工具。

### 13. 圆弧运动

#### `write_move_c(transpoint, endpoint, speed)`

- **功能：** 执行圆弧轨迹运动。
- **参数：** `transpoint` 为圆弧经过点坐标；`endpoint` 为终点坐标；`speed` 为 1 ~ 100。

### 14. 底部 IO 输入/输出状态

#### `set_basic_output(pin_no, pin_signal)` / `get_basic_input(pin_no)`

- **功能：** 设置底部 IO 输出/读取底部 IO 输入。
- **参数：** `pin_no` 为 1 ~ 6；`pin_signal`：`0` 低电平，`1` 高电平。
- **返回值：** 输入状态：`0` 低电平，`1` 高电平。

### 15. 机械臂末端 485 通信

#### `tool_serial_restore()`

- **功能：** 将末端 485 恢复出厂设置。

#### `tool_serial_ready()` / `tool_serial_available()`

- **功能：** 初始化/查询末端 485 通信。
- **返回值：** `0` 未设置或正常；`1` 设置完成或触发碰撞检测，具体取决于接口。

#### `tool_serial_read_data(data_len)` / `tool_serial_write_data(data)`

- **功能：** 读取固定长度数据/向末端 485 写入数据。
- **参数：** 数据长度范围为 1 ~ 45 字节；读取后会清除已读取数据。

#### `tool_serial_flush()` / `tool_serial_peek()`

- **功能：** 清空 485 缓冲区/查看缓冲区首字节（不清除）。
- **返回值：** `tool_serial_peek()` 返回 1 字节数据。

#### `tool_serial_set_baud(baud)` / `tool_serial_set_timeout(max_time)`

- **功能：** 设置末端 485 波特率（默认 115200）/超时（默认 30 ms）。
- **参数：** `baud` 为波特率；`max_time` 为毫秒超时值。
