# MyCobot 320

[toc]

## Python API使用说明

API（Application Programming Interface）又称应用程序编程接口函数，是预先定义好的函数，使用以下函数接口时，请在一开始就导入我们的API库，输入如下代码，否则无法运行成功：

```python
# 示例
from pymycobot import MyCobot320

mc = MyCobot320('COM3')

# 获取所有关节当前的角度
angles = mc.get_angles()
print(angles)

# 将 1关节移动到 40，速度设置为 20
mc.send_angle(1, 40, 20)
```

### 1. 系统状态

#### `get_system_version()`

- **功能:** 获取机器主控版本（pico固件版本）
- **返回值：** 固件版本号

#### `get_atom_version()`

- **功能:** 获取机器末端atom版本
- **返回值：** 固件版本号

#### `get_basic_version()`

- **功能:** 获取 M5 版本的basic固件版本
- **返回值：** `float` 固件版本

#### `get_reboot_count()`

- **功能:** 获取机器重启次数（从烧录固件后开始计算）
- **返回值：** `int` 重启次数

### 2. 机器人整体运行状态

#### `power_on()`

- **功能:** atom 打开通信（默认打开）
- **返回值:**
  - `1`: 完成

#### `power_off()`

- **功能:** 机械臂掉电
- **返回值:**
  - `1`: 完成

#### `is_power_on()`

- **功能:** 判断机械臂是否上电

- **返回值:**
  - `1`: 上电
  - `0`: 掉电
  - `-1`: 错误

#### `release_all_servos(data=None)`

- **功能：** 放松所有机械臂关节
- **参数**：`data`（可选）：关节放松方式，默认为阻尼模式，若提供 `data`参数可指定为非阻尼模式（1-Undamping）。
- **返回值:**
  - `1`: 完成

#### `focus_servo(servo_id)`

- **功能:** 单个舵机上电

- **参数:**
  - `servo_id:` int, 1-6
- **返回值:**
  - `1`: 完成

#### `is_controller_connected()`

- **功能:** Atom通信是否连接

- **返回值:**
  - `1`: 连接
  - `0`: 未连接
  - `-1`: 错误数据

#### `read_next_error()`

- **功能：** 机器人错误检测

- **返回值：** list len 6
  - `0`: 无异常
  - `1`: 通讯断开
  - `2`: 通讯不稳定
  - `3`: 伺服异常
  
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
- **返回值:**
  - `1`: 完成

#### `get_robot_status()`

- **功能:** 获取机器人自检状态，看重要参数是否都正常。
  
- **异常说明:**
  - 0: 通信异常，请检查线路、舵机固件版本是否正常、电源是否插上、固件是否烧录正确，波特率是否正确等
  - 1: 伺服电机型号错误，需要更换电机
  - 2: 伺服电机固件版本较低，需要使用FD升级
  - 3: 伺服电机p值异常，默认32，此异常会自动恢复
  - 4: 伺服电机D值异常，默认8，此异常会自动恢复
  - 5: 伺服电机I值异常，默认0，此异常会自动恢复
  - 6: 伺服电机顺时针不灵敏区参数异常，默认3，此异常会自动恢复
  - 7: 伺服电机逆时针不灵敏区参数异常，默认3，此异常会自动恢复
  - 8: 伺服电机相位异常，此异常会自动恢复
  - 9: 伺服电机返回延时异常，默认0，此异常会自动恢复
  - 10: 伺服电机最小启动力异常，默认0，此异常会自动恢复
  - 11: 伺服电机异常，当舵机异常时，无法控制机器运动，请查询舵机反馈接口get_servo_status，查看具体报错
  - 255: 未知错误

#### `focus_all_servos()`

- **功能:** 所有舵机上电

- **返回值:**
- `1`: complete

#### `set_vision_mode()`

- **功能:** 设置视觉跟踪模式，限制刷新模式下send_coords的姿态翻转。（仅适用于视觉跟踪功能）
  
- **参数:**
  - `1`: 打开
  - `0`: 关闭

- **返回值:**
  - `1`: 完成

### 3. MDI运行与操作

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
      | 1 | -168 ~ 168 |
      | 2 | -135 ~ 135 |
      | 3 | -145 ~ 145 |
      | 4 | -148 ~ 148 |
      | 5 | -168 ~ 168 |
      | 6 | -180 ~ 180 |

    - `speed`：机械臂运动速度及范围 1~100
- **返回值:**
  - `1`: 完成

#### `send_angles(angles, speed)`

- **功能：** 将所有角度发送到机械臂的所有关节
- **参数：**
  - `angles`：度数列表（`List[float]`），长度 6
  - `speed`：（`int`）1 ~ 100
- **返回值:**
  - `1`: 完成

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
      | x | -350 ~ 350 |
      | y | -350 ~ 350 |
      | z | -41 ~ 523.9 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |
  - `speed`：（`int`）1-100
- **返回值:**
  - `1`: 完成

#### `send_coords(coords, speed, mode)`

- **功能：**：发送整体坐标和姿态，将机械臂头部从原点移动到您指定的点
- **参数：**
  - `coords`: 坐标列表，值`[x,y,z,rx,ry,rz]`，长度6
  - `speed` (`int`)：1 ~ 100
  - `mode：`(`int`) 0 - 非线性，1 - 直线运动
- **返回值:**
  - `1`: 完成

#### `get_angles_plan()`

- **功能：** 获取所有关节的规划角度
- **返回值**：`list`所有度数的浮点列表

#### `get_coords_plan()`

- **功能：** 从基于基准的坐标系获取机械臂规划坐标
- **返回值：** 坐标浮点列表：[x, y, z, rx, ry, rz]

#### `pause()`

- **功能:** 控制指令暂停核心并停止所有运动指令
- **返回值**:
  - `1` - 已停止
  - `0` - 没有停止
  - `-1` - 错误

#### `sync_send_angles(angles, speed, timeout=15)`

- **功能：** 同步状态下发送角度，到达目标点后返回
- **参数：**
  - `angles`：角度值列表（`List[float]`），长度 6
  - `speed`：（`int`）1 ~ 100
  - `timeout`: 默认15秒
- **返回值:**
  - `1`: 完成

#### `sync_send_coords(coords, speed, mode=0, timeout=15)`

- **功能：** 同步状态下发送坐标，到达目标点后返回
- **参数：**
  - `coords`：坐标值列表（`List[float]`），长度6
  - `speed`：（`int`）1~100
  - `mode`：（`int`）0-非线性（默认），1-直线运动
  - `timeout`: 默认15秒
- **返回值:**
  - `1`: 完成

#### `get_angles_coords()`

- **功能：**获取关节角度和坐标

- **返回值：**一个长度为12的列表，前六位为角度信息，后六位为坐标信息。

#### `is_paused()`

- **功能:** 检查程序是否暂停移动命令
- **返回值:**
  - `1` - 已暂停
  - `0` - 没有暂停
  - `-1` - 错误

#### `resume()`

- **功能:** 恢复机器人运动并完成上一个命令
- **返回值:**
  - `1`: 完成

#### `stop()`

- **功能:** 停止机器人运动
- **返回值**:
  - `1` - 已停止
  - `0` - 没有停止
  - `-1` - 错误

#### `is_in_position(data, flag)`

- **功能** : 判断是否在某个位置。
- **参数：**
  - `data`：提供一组数据，可以是角度，也可以是坐标值，如果输入角度长度范围是6，如果输入坐标值长度范围是6
  - `flag`（值范围0或1）
    - `0`: 角度
    - `1`: 坐标
- **返回值**:
- `1` - 到达
- `0` - 未到达
- `-1 ` - 错误

#### `is_moving()`

- **功能:** 判断机器人是否运动
- **返回值:**
  - `1` 运动中
  - `0` 未运动
  - `-1` 错误
  
#### `angles_to_coords(angles)`

- **功能** : 将角度转为坐标。
- **参数：**
  - `angles`：`list` 所有角度的浮点列表。
- **返回值**: `list` 所有坐标的浮点列表。

#### `solve_inv_kinematics(target_coords, current_angles)`

- **功能** : 将坐标转为角度。
- **参数：**
  - `target_coords`: `list` 所有坐标的浮点列表。
  - `current_angles`: `list` 所有角度的浮点列表，机械臂当前角度
- **返回值**: `list` 所有角度的浮点列表。

### 4. JOG运行与操作

#### `jog_angle(joint_id, direction, speed)`

- **功能：** 点动控制角度
- **参数**：
  - `joint_id`：表示机械臂的关节，用关节ID表示，范围是1~6
  - `direction(int)`：控制机械臂运动的方向，输入`0`为负值运动，输入`1`为正值运动
  - `speed`：1~100
- **返回值:**
  - `1`: 完成

#### `jog_coord(coord_id, direction, speed)`

- **功能：** 点动控制坐标
- **参数：**
  - `coord_id`: (`int`) 机械臂坐标范围：1~6
  - `direction`: (`int`) 控制机臂运动方向，`0` - 负值运动，`1` - 正值运动
  - `speed`: 1 ~ 100
- **返回值:**
  - `1`: 完成

#### `jog_rpy(end_direction, direction, speed)`

- **功能：** 使末端绕基坐标系中固定轴旋转
- **参数：**
  - `end_direction`: (`int`) Roll、Pitch、Yaw（1-3）
  - `direction`: (`int`) 控制机臂运动方向，`1` - 正转，`0` - 反转
  - `speed`: (`int`) 1 ~ 100
- **返回值:**
  - `1`: 完成

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能**：角度步进，单关节角度增量控制
- **参数**：
  - `joint_id`：1-6
  - `increment`：基于当前位置角度的增量移动
  - `speed`：1~100
- **返回值:**
  - `1`: 完成

#### `jog_increment_coord(id, increment, speed)`

- **功能**：坐标步进，单坐标增量控制
- **参数**：
  - `id`：坐标 id 1-6
  - `increment`：基于当前位置坐标的增量移动
  - `speed`：1~100
- **返回值**：
- `1`：完成

#### `set_encoder(joint_id,coder,speed)`

- **功能**：设置单关节旋转为指定的潜在值

- **参数**
  - `joint_id`：(`int`) 1-6
  - `encoder`：0~4096
  - `speed`：1~100

#### `get_encoder(joint_id)`

- **功能**: 将单关节旋转设置为指定的电位值

- **参数**

  - `joint_id`: (`int`) 1-6

- **返回值:** (`int`) 关节电位值

#### `set_encoders(encoders, speed)`

- **功能**: 设置机械手六个关节同步执行到指定位置。

- **参数**

  - `joint_id`: (`int`) 1-6
  - `encoder`: 0 ~ 4096
  - `speed`: 1 ~ 100

#### `get_encoders()`

- **功能**：获取机械臂的六个关节电位值。

- **返回值**：（`list`）电位值列表

### 5. 运行状态及设置

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

#### `set_joint_min(id, angle)`

- **功能:** 设置最小关节角度限制
- **参数:**
  - `id` : 输入关节ID（范围1-6）
  - `angle`: 参考 [send_angle()](#send_angleid-degree-speed) 接口中对应关节的限制信息，不得小于最小值
- **返回值:**
  - `1`: 完成

#### `set_joint_max(id, angle)`

- **功能：** 设置最大关节角度限制
- **参数：**
  - `id` ：输入关节ID（范围1-6）
  - `angle`：参考 [send_angle()](#send_angleid-degree-speed) 接口中对应关节的限制信息，不得大于最大值
- **返回值:**
  - `1`: 完成
    
### 6. 关节电机控制

#### `is_servo_enable(servo_id)`

- **功能:** 检测关节连接状态
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

- **功能:** 校准指定关节，设置当前位置为角度零点，对应电位值为2048
- **参数**:
  - `servo_id`: 1 - 6
- **返回值:**
  - `1`: 完成

#### `release_servo(servo_id)`

- **功能:** 放松指定的单个舵机
- **参数**:
  - `servo_id`: 1 ~ 6
- **返回值:**
  - `1`: 放松成功
  - `0`: 放松失败
  - `-1`: 错误

#### `focus_servo(servo_id)`

- **功能**:上电指定舵机
- **参数**: `servo_id`: 1 ~ 6
- **返回值:**
  - `1`: 上电成功
  - `0`: 上电失败
  - `-1`: 错误

#### `set_servo_data(servo_id, data_id,  value, mode=None）`

- **功能:** 设置舵机指定地址的数据参数
- **参数**：
  - `servo_id`: (`int`) 关节 id 1 - 6
  - `data_id`: (`int`) 数据地址
  - `value`: (`int`) 0 - 4096
  - `mode`: 0 - 表示值为一个字节（默认），1 - 1 表示值为两个字节。
- **返回值:**
  - `1`: 完成

#### `get_servo_data(servo_id, data_id, mode=None）`

- **功能：** 读取舵机指定地址的数据参数。
- **参数**：
  - `servo_id`: (`int`) 关节id 1 - 6
  - `data_id`: (`int`) 数据地址
  - `mode`: 0 - 表示值为一个字节（默认），1 - 1 表示值为两个字节。
- **返回值：** 0 ~ 4096

#### `joint_brake(joint_id）`

- **功能：** 使关节在运动时停止，缓冲距离与现有速度正相关
- **参数**：
- `joint_id`: (`int`) 关节id 1 - 6
- **返回值:**
  - `1`: 完成

### 7. 伺服状态值

#### `get_servo_speeds()`

- **功能**：获取所有关节的运动速度
- **返回值**： 一个列表

#### `get_servo_currents()`

- **功能**：获取关节电流
- **返回值**：一个列表, 0 ~ 3250 mA

#### `get_servo_voltages()`

- **功能**：获取关节电压
- **返回值**： 一个列表， 电压值小于24 V

#### `get_servo_status()`

- **功能**：获取所有关节的运动状态
- **返回值**： 列表，[电压，传感器，温度，电流，角度，过载]，值为`0`表示无错误，值为`1`表示有错误

- **异常说明**：
  - 0: 伺服电机欠压/过压，查看电压，如果为0，需要修改舵机参数；如果大于实际，可能散热片损坏
  - 1: 伺服电机磁编码异常
  - 2: 伺服电机过温
  - 3: 伺服电机过流
  - 5: 伺服电机过载

#### `get_servo_temps()`

- **功能**：获取关节温度
- **返回值**： 一个列表，单位摄氏度

### 8. 机械臂末端IO控制

#### `set_color(r, g, b)`

- **功能**: 设置机械臂末端灯光颜色
- **参数**:

  - `r (int)`: 0 ~ 255

  - `g (int)`: 0 ~ 255

  - `b (int)`: 0 ~ 255
 - **返回值:**
  - `1`: 完成
 
#### `set_digital_output(pin_no, pin_signal)`

- **功能:** 设置IO状态
- **参数**
  - `pin_no` (int): 引脚号
  - `pin_signal` (int): 0 / 1, 输入0表示设置为运行状态，输入1表示停止状态
- **返回值:**
  - `1`: 完成

#### `get_digital_input(pin_no)`

- **功能:** 获取IO状态
- **参数**: `pin_no` (int)
- **返回值**: 当返回的值为0表示在工作状态运行，1表示停止状态

#### `set_pin_mode(pin_no, pin_mode)`

- **功能:** 设置原子中指定引脚的状态模式。
- **参数**
  - `pin_no` (int): 引脚号
  - `pin_mode` (int): 0 - 运行状态, 1 - 停止状态, 2 - 上拉模式
- **返回值:**
  - `1`: 完成

### 9. 机械臂末端夹爪控制

#### `set_gripper_state(flag, speed, _type_1=None)`

- **功能**: 让夹爪以指定的速度进入到指定的状态

- **参数**:

  - `flag (int) `: 0 - 打开 1 - 关闭, 254 - 释放

  - `speed (int)`: 1 ~ 100

  - `_type_1 (int)`:

    - `1` : 自适应夹爪 (默认是自适应夹爪)

    - `2` : 五指灵巧手

    - `3` : 平行夹爪

    - `4` : 柔性夹爪
- **返回值:**
  - `1`: 完成

#### `set_gripper_value(gripper_value, speed, gripper_type=None)`

- **功能**: 让夹爪以指定的速度转动到指定的位置

- **参数**:

  - `gripper_value (int) `: 0 ~ 100

  - `speed (int)`: 1 ~ 100

  - `gripper_type (int)`:

    - `1` : 自适应夹爪 (默认是自适应夹爪)

    - `2` : 五指灵巧手

    - `3` : 平行夹爪

    - `4` : 柔性夹爪
- **返回值:**
  - `1`: 完成

#### `set_gripper_calibration()`

- **功能**: 将夹爪的当前位置设置为零位
- **返回值:**
  - `1`: 完成

#### `init_electric_gripper()`

- **功能**：电动夹爪初始化（插入和移除夹爪后需初始化一次）
- **返回值:**
  - `1`: 完成

#### `set_electric_gripper(status)`

- **功能**：设置电动夹爪模式
- **参数**：
  - `status`：0 - 打开，1 - 关闭。
- **返回值:**
  - `1`: 完成

#### `set_gripper_mode(mode)`

- **功能**：设置夹爪模式
- **参数**：
  - `mode`：0 - 透明传输。1 - 端口模式。
- **返回值:**
  - `1`: 完成

#### `get_gripper_mode()`

- **功能**：获取夹爪模式
- **返回值**：
  - `mode`：0 - 透明传输。1 - 端口模式。

### 10. 设置底部IO输入/输出状态

#### `set_basic_output(pin_no, pin_signal)`

- **功能**：设置底部引脚号的工作状态。
- **参数**：
  - `pin_no` (`int`) 设备底部标注的编号仅取数字部分
  - `pin_signal` (`int`): 0 - 低电平，设置为运行状态. 1 - 高电平，停止状态。
- **返回值:**
  - `1`: 完成

#### `get_basic_input(pin_no)`

- **功能:** 获取底部引脚号的工作状态
- **参数:**
  - `pin_no` (`int`) 表示机器人底部的具体引脚号。
- **返回值:** 0 - 低电平，运行状态. 1 - 高电平，停止状态

### 11. WLAN 设置

#### `set_ssid_pwd(account, password)`

- **功能:** 更改连接的wifi. (适用于M5)
- **参数:**
  - `account`： (`str`) 新的 wifi 账户
  - `password`: (`str`) 新的 wifi 密码
- **返回值:**
  - `1`: 完成

#### `get_ssid_pwd()`

- **功能:** 获取连接的wifi账号和密码. (适用于M5)
- **返回值:** 当前连接的wifi账号和密码

#### `set_server_port(port)`

- **功能:** 更改服务器的连接端口
- **参数:**
  - `port`: (`int`) 服务器的新连接端口
- **返回值:**
  - `1`: 完成

### 12. TOF

#### `get_tof_distance()`

- **功能:** 获取检测到的距离(需要外部距离检测器)
- **返回值:** 检测到的距离值，单位为mm。

### 13. 坐标变换

#### `set_tool_reference(coords)`

- **功能:** 设置工具坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].
- **返回值:**
  - `1`: 完成

#### `get_tool_reference(coords)`

- **功能:** 获取工具坐标系
- **返回值:** (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **功能:** 设置世界坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].
- **返回值:**
  - `1`: 完成

#### `get_world_reference()`

- **功能:** 获取世界坐标系.
- **返回值:** `list` [x, y, z, rx, ry, rz].

#### `set_reference_frame(rftype)`

- **功能:** 设置基坐标系
- **参数：**
  - `rftype`: 0 - 基坐标（默认） 1 - 世界坐标.
- **返回值:**
  - `1`: 完成

#### `get_reference_frame()`

- **功能:** 获取基坐标系
- **返回值:** (`list`) [x, y, z, rx, ry, rz].

#### `set_movement_type(move_type)`

- **功能:** 设置移动类型
- **参数**：
  - `move_type`: 1 - movel, 0 - moveJ.
- **返回值:**
  - `1`: 完成

#### `get_movement_type()`

- **功能:** 获取移动类型
- **返回值:**
  - `1` - movel
  - `0` - moveJ

#### `set_end_type(end)`

- **功能:** 设置末端坐标系
- **参数:**
  - `end (int)`: `0` - 法兰（默认）, `1` - 工具
- **返回值:**
  - `1`: 完成

#### `get_end_type()`

- **功能:** 获取末端坐标系
- **返回值:**
  - `0` - 法兰（默认）
  - `1` - 工具


### 14. 树莓派 -- GPIO

#### `gpio_init()`

- **功能**: 初始化 GPIO 模块，设置 BCM 模式

#### `gpio_output(pin, v)`

- **功能**: 设置GPIO输出值

- **参数**

  - `pin` (`int`) 引脚编号
  - `v` (`int`):0 -设置为低电平 1 -设置为高电平

### 15. utils（模块）

该模块支持一些帮助方法，使用之前在文件开头输入代码导入模块：

```python
from pymycobot import utils
```

#### `utils.get_port_list()`

- **功能**: 获取当前所有串口号列表

- **返回值:** 串口列表(`list`)

#### `utils.detect_port_of_basic()`

- **功能**: 返回第一个检测到的 M5 Basic 的串口号。(只会返回一个串口号)

- **返回值:** 返回检测到的端口号，如果没有监测到串口号则返回：None

### 16. Pro 力控夹爪

#### `set_pro_gripper(address, value, gripper_id=14)`

- **功能**：设置Pro力控夹爪参数，可以设置多种参数功能。具体请查看如下表格。
- **参数**：
  - `address` (`int`): 夹爪的指令序号。
  - `value` ：指令序号对应的参数值。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

    | 功能 | gripper_id | address | value|
    | ---- | ---- |---- |----- |
    | 设置夹爪ID | 14 | 3 | 1 ~ 254 |
    | 设置夹爪使能状态 | 14 | 10 | 0或者1, 0 - 掉使能; 1 - 上使能 |
    | 设置夹爪顺时针可运行误差 | 14 | 21 | 0 ~ 16 |
    | 设置夹爪逆时针可运行误差 | 14 | 23 | 0 ~ 16 |
    | 设置夹爪最小启动力 | 14 | 25 | 0 ~ 254 |
    | IO输出设置 | 14 | 29 | 0, 1, 16, 17 |
    | 设置IO张开角度 | 14 | 30 | 0 ~ 100 |
    | 设置IO闭合角度 | 14 | 31 | 0 ~ 100 |
    | 设置舵机虚位数值 | 14 | 41 | 0 ~ 100 |
    | 设置夹持电流 | 14 | 43 | 1 ~ 254 |

- **返回值**:
  - 请查看如下表格：

    | 功能 | return |
    | ---- | ---- |
    | 设置夹爪ID | 0 - 失败； 1 - 成功 |
    | 设置夹爪使能状态 | 0 - 失败； 1 - 成功 |
    | 设置夹爪顺时针可运行误差 | 0 - 失败； 1 - 成功 |
    | 设置夹爪逆时针可运行误差 | 0 - 失败； 1 - 成功 |
    | 设置夹爪最小启动力 | 0 - 失败； 1 - 成功 |
    | IO输出设置 | 0 - 失败； 1 - 成功 |
    | 设置IO张开角度 | 0 - 失败； 1 - 成功 |
    | 设置IO闭合角度 | 0 - 失败； 1 - 成功 |
    | 设置舵机虚位数值 | 0 - 失败； 1 - 成功 |
    | 设置夹持电流 | 0 - 失败； 1 - 成功 |

#### `get_pro_gripper(address, gripper_id=14)`

- **功能**：获取Pro力控夹爪参数，可以获取多种参数功能。具体请查看如下表格。
- **参数**：
  - `address` (`int`): 夹爪的指令序号。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

    | 功能 | gripper_id | address |
    | ---- | ---- |---- |
    | 读取固件主版本号 | 14 | 1 |
    | 读取固件次版本号 | 14 | 2 |
    | 读取夹爪ID | 14 | 4 |
    | 读取夹爪顺时针可运行误差 | 14 | 22 |
    | 读取夹爪逆时针可运行误差 | 14 | 24 |
    | 读取夹爪最小启动力 | 14 | 26 |
    | 读取IO张开角度 | 14 | 34 |
    | 读取IO闭合角度 | 14 | 35 |
    | 获取当前队列的数据量 | 14 | 40 |
    | 读取舵机虚位数值 | 14 | 42 |
    | 读取夹持电流 | 14 | 44 |

- **返回值**：
  - 查看如下表格（若返回值为 -1，则表示读不到数据）：
  
    | 功能 | return |
    | ---- | ---- |
    | 读取固件主版本号 | 主版本号 |
    | 读取固件次版本号 | 次版本号 |
    | 读取夹爪ID | 1 ~ 254 |
    | 读取夹爪顺时针可运行误差 | 0 ~ 254 |
    | 读取夹爪逆时针可运行误差 | 0 ~ 254 |
    | 读取夹爪最小启动力 | 0 ~ 254 |
    | 读取IO张开角度 | 0 ~ 100 |
    | 读取IO闭合角度 | 0 ~ 100 |
    | 获取当前队列的数据量 | 返回当前绝对控制队列中的数据量 |
    | 读取舵机虚位数值 | 0 ~ 100 |
    | 读取夹持电流 | 1 ~ 254 |
  
#### `set_pro_gripper_angle(gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪角度。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围1 ~ 254。
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
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

#### `set_pro_gripper_torque(torque_value, gripper_id=14)`

- **功能**：设置力控夹爪扭矩。
- **参数**：
  - `torque_value` (`int`) ：扭矩值，取值范围 100 ~ 300。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_torque(gripper_id=14)`

- **功能**：读取力控夹爪扭矩。
- **参数**：
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值:** (`int`) 100 ~ 300

#### `set_pro_gripper_speed(speed, gripper_id=14)`

- **功能**：设置力控夹爪速度。
- **参数**：
  - `speed` (int): 夹爪运动速度，取值范围 1 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_default_speed(speed, gripper_id=14)`

- **功能**：读取力控夹爪默认速度。
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

#### `set_pro_gripper_pause(gripper_id=14)`

- **功能**：暂停运动。
- **参数**：
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_resume(gripper_id=14)`

- **功能**：运动恢复。
- **参数**：
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_stop(gripper_id=14)`

- **功能**：停止运动。
- **参数**：
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

### 17. myGripper H100 三指夹爪

#### `get_hand_firmware_major_version(gripper_id=14)`

- **功能**：读取固件**主**版本号  
- **参数**：
  - `gripper_id` (`int`) 夹爪 ID，默认值 14，范围 1 ~ 254  
- **返回值**：主版本号（浮点数）

#### `get_hand_firmware_minor_version(gripper_id=14)`

- **功能**：读取固件**次**版本号  
- **参数**：
  - `gripper_id` (`int`) 夹爪 ID，默认值 14，范围 1 ~ 254  
- **返回值**：次版本号（整数）

#### `set_hand_gripper_id(id_value, gripper_id=14)`

- **功能**：设置夹爪的 ID  
- **参数**：
  - `id_value` (`int`) 新 ID，范围 1 ~ 254  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_id(gripper_id=14)`

- **功能**：获取夹爪的 ID  
- **参数**：
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：夹爪 ID（整数）

#### `set_hand_gripper_angle(joint_id, gripper_angle, gripper_id=14)`

- **功能**：设置单个关节的角度  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_angle` (`int`) 0 ~ 100  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_angle(joint_id, gripper_id=14)`

- **功能**：读取单个关节的角度  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`gripper_angle` (`int`) 0 ~ 100

#### `set_hand_gripper_angles(gripper_angles, speed, gripper_id=14)`

- **功能**：设置所有关节的角度  
- **参数**：
  - `gripper_angles` (`list[int]`) 含 6 个角度值，范围 0 ~ 100  
  - `gripper_id` (`int`) 1 ~ 254，默认14
  - `speed` (`int`) 0 ~ 100  
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_angles(gripper_id=14)`

- **功能**：读取所有关节的角度  
- **参数**：
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：6 个整数组成的列表，范围 0 ~ 100

#### `set_hand_gripper_torque(joint_id, torque_value, gripper_id=14)`

- **功能**：设置某关节的力矩  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `torque_value` (`int`) 100 ~ 300  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_torque(joint_id, gripper_id=14)`

- **功能**：获取某关节的力矩  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`torque_value` (`int`) 100 ~ 300

#### `set_hand_gripper_calibrate(joint_id, gripper_id=14)`

- **功能**：校准某个关节的零点位置  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_status(gripper_id=14)`

- **功能**：获取夹爪夹持状态  
- **参数**：
  - `gripper_id` (`int`) 夹爪 ID，默认 14，范围 1 ~ 254  
- **返回值**：
  - 0 - 正在移动  
  - 1 - 停止，未夹持  
  - 2 - 停止，检测到夹持  
  - 3 - 夹持后物体掉落

#### `set_hand_gripper_enabled(flag, gripper_id=14)`

- **功能**：设置夹爪的使能状态  
- **参数**：
  - `flag` (`int`) 0 或 1 
  - `gripper_id` (`int`) 1 ~ 254，默认14 
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `set_hand_gripper_speed(joint_id, speed, gripper_id=14)`

- **功能**：设置某关节的移动速度  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `speed` (`int`) 1 ~ 100  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_default_speed(joint_id, gripper_id=14)`

- **功能**：读取某关节的默认速度  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：默认速度 (`int`) 1 ~ 100

#### `set_hand_gripper_p(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的 P 值  
- **参数**： 
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 254  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_p(joint_id, gripper_id=14)`

- **功能**：读取某关节的 P 值  
- **参数**： 
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 254

#### `set_hand_gripper_d(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的 D 值  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 254  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_d(joint_id, gripper_id=14)`

- **功能**：读取某关节的 D 值  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 254

#### `set_hand_gripper_i(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的 I 值  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 254  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_i(joint_id, gripper_id=14)`

- **功能**：读取某关节的 I 值  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 254

#### `set_hand_gripper_min_pressure(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的最小启动力  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 254  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_min_pressure(joint_id, gripper_id=14)`

- **功能**：获取某关节的最小启动力  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 254

#### `set_hand_gripper_clockwise(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的顺时针误差范围  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 16  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_clockwise(joint_id, gripper_id=14)`

- **功能**：获取某关节的顺时针误差范围  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 16

#### `set_hand_gripper_counterclockwise(joint_id, value, gripper_id=14)`

- **功能**：设置某关节的逆时针误差范围  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `value` (`int`) 0 ~ 16  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_counterclockwise(joint_id, gripper_id=14)`

- **功能**：获取某关节的逆时针误差范围  
- **参数**：
  - `joint_id` (`int`) 1 ~ 6  
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：`value` (`int`) 0 ~ 16

#### `set_hand_gripper_pinch_action(pinch_pose, rank_mode, idle_flag=False, gripper_id=14)`

- **功能**：设置夹爪配合动作及速度  
- **参数**： 
  - `pinch_pose` (`int`) 0 ~ 4  
    - 0：所有关节归零  
    - 1：食指与拇指夹持  
    - 2：中指与拇指夹持  
    - 3：食指与中指夹持  
    - 4：三指夹持（若为 4，`rank_mode` 范围为 1 ~ 20）  
  - `rank_mode` (`int`) 0 ~ 5  
  - `idle_flag` (`bool`, 可选)：默认False
  - `gripper_id` (`int`) 1 ~ 254，默认14
- **返回值**：
  - 0 - 失败  
  - 1 - 成功

#### `get_hand_gripper_type(gripper_id=14)`

- **功能**：获取设备类型（左右手）  
- **参数**：
  - `gripper_id` (`int`) 1 ~ 254，默认14  
- **返回值**：`int` 类型  
  - 0 - 左手  
  - 1 - 右手


## MyCobot 320 Socket

> 注意：
> raspberryPi版本 仅支持python3
> 使用此类前提的机械臂有服务器，并且已经开启服务。

使用TCP/IP控制机械臂

### 客户端

```python
# 示例
from pymycobot import MyCobot320Socket
# 默认使用端口 9000
mc = MyCobot320Socket("192.168.10.10",9000)

res = mc.get_angles()
print(res)

mc.send_angles([0,0,0,0,0,0],20)
...
```

### 服务端

服务端文件在`demo文件夹`中，具体请检查demo文件夹中的[Server_320.py](../demo/Server_320.py)文件

### socket 控制

> 注意：
> 大部分方法与 MyCobot320 类相同，这里只列出新方法。


#### `set_gpio_mode(mode)`

- **功能**: 设置树莓派GPIO针脚模式

- **参数**

  - `mode` (`str`) "BCM" 或者 "BOARD" 进入相应模式

#### `set_gpio_out(pin_no, mode)`

- **功能**: 设置引脚作为输入或者输出

- **参数**

  - `pin_no` (`int`) 引脚编号.
  - `mode` (`str`) `in` - 输入 ； `out` - 输出

#### `set_gpio_output(pin_no, state)`

- **功能**: 将引脚设置为高，低电平

- **参数**

  - `pin_no` (`int`) 引脚编号.
  - `state` (`int`) 0-设置为低电平 1-设置为高电平

#### `get_gpio_in(pin_no)`

- **功能**: 获取引脚电平状态

- **参数**

  - `pin_no` (`int`) 引脚编号
- **返回值:**  0 为低电平 1 为高电平
