# MyCobot 280 X5 PI

[toc]

## Python API 使用说明

API（Application Programming Interface）即应用程序编程接口，是预先定义的函数。使用以下函数接口时，请先在开头导入我们的API库，否则无法成功运行：

```python
# 示例
from pymycobot import MyCobot280RDKX5

mc = MyCobot280RDKX5('/dev/ttyS1')

# 获取所有关节当前角度
angles = mc.get_angles()
print(angles)

# 设置1号关节移动到40度，速度20
mc.send_angle(1, 40, 20)
```

### 1. 系统状态

#### `get_modify_version()`
- **功能:** 获取修改版本号
#### `get_system_version()`
- **功能:** 获取系统版本

### 2. 整体状态

#### `clear_queue()`
- **功能:** 清空命令队列
#### `async_or_sync()`
- **功能:** 设置命令执行模式
- **返回值:**
  - **`0`: 异步模式**
  - **`1`: 同步模式**
#### `get_error_information()`
- **功能:** 获取机器人错误信息
- **返回值:**
  - **`0`: 无错误**
  - **`1~6`: 对应关节超限**
  - **`16~19`: 碰撞保护**
  - **`32`: 逆运动学无解**
  - **`33~34`: 直线运动无相邻解**
#### `clear_error_information()`
- **功能:** 清除机器人错误信息
#### `power_on()`
- **功能:** 开启与Atom通信
#### `power_off()`
- **功能:** 关闭与Atom通信
#### `is_power_on()`
- **功能:** 检测机械臂电源状态
- **返回值:**
  - **`1` - 已上电**
  - **`0` - 已断电**
  - **`-1` - 数据错误**
#### `release_all_servos(data)`
- **功能:** 释放所有关节
- **参数:**
  - **data: `1` - 取消阻尼(默认阻尼)**
#### `read_next_error()`
- **功能:** 机器人错误检测
- **返回值:**
  - **长度6的列表**
  - **`0`: 正常**
  - **`1`: 通信断开**
  - **`2`: 通信不稳定**
  - **`3`: 舵机异常**
#### `set_fresh_mode(mode)`
- **功能:** 设置命令刷新模式
- **参数:**
  - **mode: int**
  - **`1` - 优先执行最新命令**
  - **`0` - 队列顺序执行**
#### `get_fresh_mode()`
- **功能:** 查询运动模式
#### `set_vision_mode(flag)`
- **功能:** 设置视觉跟踪模式(限制刷新模式下send_coords的位姿翻转)
  - **(仅限视觉跟踪功能使用)**
- **参数:**
  - **flag: 0/1; `0` - 关闭; `1` - 开启**

### 3. 运动控制接口

#### `get_angles()`
- **功能:** 获取所有关节角度
- **返回值:**
  - **list: 角度浮点数列表**
#### `send_angle(id, degree, speed)`
- **功能:** 发送单个关节角度
- **参数:**
  - **id : 关节ID(1-6)**
  - **angle : 角度值(float)**
  - **speed : (int) 1~100**
#### `send_angles(angles, speed)`
- **功能:** 发送所有关节角度
- **参数:**
  - **angles: 角度值列表(List[float])，长度6**
  - **speed : (int) 1~100**
#### `get_coords()`
- **功能:** 获取基坐标系坐标
- **返回值:**
  - **list: 坐标浮点数列表[x, y, z, rx, ry, rz]**
#### `send_coord(id, coord, speed)`
- **功能:** 发送单个坐标值
- **参数:**
  - **id(int) : 坐标轴ID(1-6)**
  - **coord(float) : 坐标值(mm)**
  - **speed(int) : 1~100**
#### `send_coords(coords, speed, mode)`
- **功能:** 发送所有坐标值
- **参数:**
  - **coords: 坐标值列表(List[float])[x(mm), y, z, rx(角度), ry, rz]**
  - **speed : (int) 1~100**
  - **mode : (int) 0 - 角度模式, 1 - 直线模式**
#### `pause()`
- **功能:** 暂停运动
#### `is_paused()`
- **功能:** 判断机械臂是否暂停
- **返回值:**
  - **`1` - 已暂停**
  - **`0` - 运行中**
  - **`-1` - 错误**
#### `resume()`
- **功能:** 恢复运动
#### `stop()`
- **功能:** 停止运动
#### `is_in_position(data, id = 0)`
- **功能:** 判断是否到达目标位置
- **参数:**
  - **data: 位置数据列表(角度或坐标)，长度6**
  - **id: 1 - 坐标模式, 0 - 角度模式**
- **返回值:**
  - **`1` - 是**
  - **`0` - 否**
  - **`-1` - 错误**
#### `is_moving()`
- **功能:** 检测机械臂是否运动中
- **返回值:**
  - **`0` - 静止**
  - **`1` - 运动中**
  - **`-1` - 数据错误**
#### `write_angles_time_control(angles, step_time)`
- **功能:** 时间控制模式写入关节角度
- **参数:**
  - **angles: 角度值**
  - **step_time: 时间单位30ms，范围1~255**

### 4. JOG模式与操作

#### `jog_angle(joint_id, direction, speed)`
- **功能:** 关节模式点动控制
- **参数:**
  - **joint_id: 整数1-6**
  - **direction: `0` - 减方向, `1` - 增方向**
  - **speed: 整数(0-100)**
#### `jog_rpy(end_direction, direction, speed)`
- **功能:** 绕基坐标系固定轴旋转末端
- **参数:**
  - **end_direction (int):  横滚、俯仰、偏航(1-3)**
  - **direction (int): `1` - 正向旋转, `0` - 反向旋转**
  - **speed (int): 1~100**
#### `jog_coord(coord_id, direction, speed)`
- **功能:** 坐标模式点动控制
- **参数:**
  - **coord_id: 整数1-6**
  - **direction: `0` - 减方向, `1` - 增方向**
  - **speed: 整数(1-100)**
#### `jog_increment_angle(joint_id, increment, speed)`
- **功能:** 角度步进模式
- **参数:**
  - **joint_id: 整数1-6**
  - **increment: 角度增量值**
  - **speed: 整数(0-100)**
#### `jog_increment_coord(axis, increment, speed)`
- **功能:** 坐标步进模式
- **参数:**
  - **axis: 轴ID 1-6**
  - **increment: 坐标增量值**
  - **speed: 整数(1-100)**
#### `set_HTS_gripper_torque(torque)`
- **功能:** 设置自适应夹爪扭矩
- **参数:**
  - **torque (int): 150~980**
- **返回值:**
  - **0: 设置失败**
  - **1: 设置成功**
#### `get_HTS_gripper_torque()`
- **功能:** 获取夹爪扭矩
- **返回值:**
  - **int: 150~980**
#### `get_gripper_protect_current()`
- **功能:** 获取夹爪保护电流
- **返回值:**
  - **int: 1~500**
#### `init_gripper()`
- **功能:** 初始化夹爪
- **返回值:**
  - **int: 0或1(1-成功)**
#### `set_gripper_protect_current(current)`
- **功能:** 设置夹爪保护电流
- **参数:**
  - **current (int): 1~500**
#### `set_encoder(joint_id, encoder, speed)`
- **功能:** 设置单个关节电位值
- **参数:**
  - **joint_id: 整数1-6**
  - **夹爪对应关节ID为7**
  - **encoder: 目标电位值**
  - **speed : 1-100**
#### `get_encoder(joint_id)`
- **功能:** 获取指定关节电位值
- **参数:**
  - **joint_id: (int) 1-6**
  - **夹爪对应关节ID为7**
#### `set_encoders(encoders, sp)`
- **功能:** 同步设置所有关节电位值
- **参数:**
  - **encoders: 电位值列表，长度6**
  - **sp: 速度1~100**
#### `get_encoders()`
- **功能:** 获取所有关节编码器值
- **返回值:**
  - **编码器值列表**
#### `set_encoders_drag(encoders, speeds)`
- **功能:** 发送所有编码器值和速度
- **参数:**
  - **encoders: 编码器列表**
  - **speeds: 通过get_servo_speeds()方法获取的速度值**
#### `get_joint_min_angle(joint_id)`
- **功能:** 获取指定关节最小角度
- **参数:**
  - **joint_id: 1-6**
- **返回值:**
  - **角度值(float)**
#### `get_joint_max_angle(joint_id)`
- **功能:** 获取指定关节最大角度
- **参数:**
  - **joint_id: 1-6**
- **返回值:**
  - **角度值(float)**
#### `set_joint_min(id, angle)`
- **功能:** 设置关节最小角度
- **参数:**
  - **id: int**
  - **关节ID 1-6**
  - **夹爪对应关节ID为7**
  - **angle: 0~180**
#### `set_joint_max(id, angle)`
- **功能:** 设置关节最大角度
- **参数:**
  - **id: int**
  - **关节ID 1-6**
  - **夹爪对应关节ID为7**
  - **angle: 0~180**

### 5. 舵机控制

#### `is_servo_enable(servo_id)`
- **功能:** 检测单个关节连接状态
- **参数:**
  - **servo_id: 1-6**
- **返回值:**
  - **`0` - 未使能**
  - **`1` - 已使能**
  - **`-1` - 错误**
#### `is_all_servo_enable()`
- **功能:** 检测所有关节连接状态
- **返回值:**
  - **`0` - 未使能**
  - **`1` - 已使能**
  - **`-1` - 错误**
#### `set_servo_data(servo_id, data_id, value, mode)`
- **功能:** 设置舵机指定地址数据
- **参数:**
  - **servo_id: 关节舵机编号1-7**
  - **data_id: 数据地址**
  - **value: 0-4096**
  - **mode: 0 - 单字节值(默认), 1 - 双字节值**
#### `get_servo_data(servo_id, data_id, mode)`
- **功能:** 读取舵机指定地址数据
- **参数:**
  - **servo_id: 关节舵机编号1-7**
  - **data_id: 数据地址**
  - **mode: 0 - 单字节值(默认), 1 - 双字节值**
- **返回值:**
  - **0-4096范围的值**
#### `set_servo_calibration(servo_id)`
- **功能:** 校准关节当前位置为零点(对应电位值2048)
- **参数:**
  - **servo_id: 关节舵机编号1-6**
#### `joint_brake(joint_id)`
- **功能:** 关节运动中急停(缓冲距离与当前速度正相关)
- **参数:**
  - **joint_id: 1-6**
#### `release_servo(servo_id, mode)`
- **功能:** 下电指定舵机
- **参数:**
  - **servo_id: 整数1-6**
  - **mode: 默认阻尼，设为1取消阻尼**
#### `focus_servo(servo_id)`
- **功能:** 上电指定舵机
- **参数:**
  - **servo_id: 整数1-6**

### 6. 夹爪控制

#### `get_gripper_value(gripper_type)`
- **功能:** 获取夹爪状态值
- **参数:**
  - **gripper_type (int): 默认1**
  - **`1`: 自适应夹爪**
  - **`3`: 平行夹爪**
  - **`4`: 柔性夹爪**
- **返回值:**
  - **夹爪状态值(int)**
#### `set_gripper_state(flag, speed, _type_1, is_torque)`
- **功能:** 设置夹爪开关状态
- **参数:**
  - **flag  (int): 0 - 打开, 1 - 关闭, 254 - 释放**
  - **speed (int): 1~100**
  - **_type_1 (int): 默认1**
  - **`1` : 自适应夹爪**
  - **`2` : 五指灵巧手**
  - **`3` : 平行夹爪(可省略)**
  - **`4` : 柔性夹爪**
  - **is_torque (int): 无类型参数时可省略**
  - **`1`: 力控**
  - **`0`: 非力控**
#### `set_gripper_value(gripper_value, speed, gripper_type, is_torque)`
- **功能:** 设置夹爪开合度
- **参数:**
  - **gripper_value (int): 0~100**
  - **speed (int): 1~100**
  - **gripper_type (int): 默认1**
  - **`1`: 自适应夹爪**
  - **`3`: 平行夹爪(可省略)**
  - **`4`: 柔性夹爪**
  - **is_torque (int): 无类型参数时可省略**
  - **`1`: 力控**
  - **`0`: 非力控**
#### `set_gripper_calibration()`
- **功能:** 设置当前位置为零点(对应电位值2048)
#### `is_gripper_moving()`
- **功能:** 判断夹爪是否运动中
- **返回值:**
  - **`0` - 静止**
  - **`1` - 运动中**
  - **`-1` - 数据错误**

### 7. 末端ATOM功能

#### `get_tool_system_version()`
- **功能:** 读取末端主次版本号
#### `get_tool_modify_version()`
- **功能:** 读取末端修改版本号
#### `is_tool_connected()`
- **功能:** 检测末端连接状态
#### `set_color(r = 0, g = 0, b = 0)`
- **功能:** 设置机械臂顶部灯光颜色
- **参数:**
  - **r (int): 0~255**
  - **g (int): 0~255**
  - **b (int): 0~255**
#### `is_tool_button_click()`
- **功能:** 检测末端按钮是否按下
#### `set_digital_output(pin_no, pin_signal)`
- **功能:** 设置末端Atom IO状态
- **参数:**
  - **pin_no     (int): 引脚号**
  - **pin_signal (int): 0/1**
#### `get_digital_input(pin_no)`
- **功能:** 获取数字输入信号值

### 8. 运动学算法接口

#### `solve_inv_kinematics(target_coords, current_angles)`
- **功能:** 将目标坐标转换为关节角度
- **参数:**
  - **target_coords: 目标坐标浮点数列表**
  - **current_angles : 当前角度浮点数列表**
- **返回值:**
  - **list: 关节角度浮点数列表**

### 9. 坐标系接口

#### `set_tool_reference(coords)`
- **功能:** 设置工具坐标系
- **参数:**
  - **coords: 坐标值列表(List[float])**
  - **[x(mm), y, z, rx(角度), ry, rz]**
#### `get_tool_reference()`
- **功能:** 获取工具坐标系
#### `set_world_reference(coords)`
- **功能:** 设置世界坐标系
- **参数:**
  - **coords: 坐标值列表(List[float])**
  - **[x(mm), y, z, rx(角度), ry, rz]**
#### `get_world_reference()`
- **功能:** 获取世界坐标系
#### `set_reference_frame(rftype)`
- **功能:** 设置基坐标系
- **参数:**
  - **rftype: 0 - 基坐标系, 1 - 工具坐标系**
#### `get_reference_frame()`
- **功能:** 获取基坐标系
- **返回值:**
  - **`0` - 基坐标系, `1` - 工具坐标系**
#### `set_movement_type(move_type)`
- **功能:** 设置运动类型
- **参数:**
  - **move_type: `1` - 直线运动, `0` - 关节运动**
#### `get_movement_type()`
- **功能:** 获取运动类型
- **返回值:**
  - **`1` - 直线运动, `0` - 关节运动**
#### `set_end_type(end)`
- **功能:** 设置末端坐标系
- **参数:**
  - **end: int**
  - **`0` - 法兰坐标系, `1` - 工具坐标系**
#### `get_end_type()`
- **功能:** 获取末端坐标系
- **返回值:**
  - **`0` - 法兰坐标系, `1` - 工具坐标系**

### 10. 9G舵机五子棋

#### `move_round()`
- **功能:** 驱动9g舵机顺时针旋转一周
#### `set_four_pieces_zero()`
- **功能:** 设置四件套电机零位
- **返回值:**
  - **int: `0`或`1`(1-成功)**

### 11. 标准接口

#### `get_angles_coords()`
- **功能:** 获取关节角度和坐标
#### `get_quick_move_message()`
- **功能:** 获取快速移动信息
#### `get_servo_speeds()`
- **功能:** 获取关节速度
- **返回值:**
  - **速度列表(单位:步/秒)**

### 12. 舵机状态值接口

#### `get_servo_currents()`
- **功能:** 获取所有关节电流
- **返回值:**
  - **电流列表(单位mA)**
#### `get_servo_voltages()`
- **功能:** 获取关节电压
- **返回值:**
  - **电压列表(需小于24V)**
#### `get_servo_status()`
- **功能:** 获取关节状态
- **返回值:**
  - **[电压, 传感器, 温度, 电流, 角度, 过载], 0表示正常，1表示异常**
#### `get_servo_temps()`
- **功能:** 获取关节温度
- **返回值:**
  - **温度列表(单位℃)**

### 13. 拖动轨迹接口

#### `drag_start_record()`
- **功能:** 开始轨迹记录
- **返回值:**
  - **记录队列长度**
#### `drag_end_record()`
- **功能:** 结束轨迹记录
- **返回值:**
  - **记录队列长度**
#### `drag_get_record_data()`
- **功能:** 获取已记录轨迹
- **返回值:**
  - **各关节电位值(编码器值)和运行速度的列表**
  - **示例: [J1编码器值,J1运行速度,J2编码器值,J2运行速度,J3编码器值,J3运行速度,J4编码器值,J4运行速度,J5编码器值,J5运行速度,J6编码器值,J6运行速度]**
#### `drag_get_record_len()`
- **功能:** 获取已记录点总数
- **返回值:**
  - **记录队列长度**
#### `drag_clear_record_data()`
- **功能:** 清除已记录轨迹
- **返回值:**
  - **记录队列长度归零**
