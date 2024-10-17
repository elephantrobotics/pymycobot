# MechArm 270

[toc]

## Python API使用说明

API（Application Programming Interface）又称应用程序编程接口函数，是预先定义好的函数，使用以下函数接口时，请在一开始就导入我们的API库，输入如下代码，否则无法运行成功：

```python
# 示例
from pymycobot import MechArm270

mc = MechArm270('COM3')

# 获取所有关节当前的角度
angles = mc.get_angles()
print(angles)

# 将 1关节移动到 40，速度设置为 20
mc.send_angle(1, 40, 20)
```

### 1. 系统状态

#### `get_system_version()`

- **功能:** 获取机器主控版本（atom固件版本）
- **返回值：** 固件版本号

#### `get_basic_version()`

- **功能:** 获取 M5 版本的basic固件版本
- **返回值：** `float` 固件版本

#### `get_error_information()`

- **功能:** 获取机器人错误信息

- **返回值：** 
  - 0：无错误信息。
  - 1～6：对应关节超出极限位置。
  - 16～19：碰撞保护。
  - 32：运动学逆解无解。
  - 33～34：直线运动无相邻解。

#### `clear_error_information()`

- **功能:** 清除机器人错误信息

### 2. 机器人整体运行状态

#### `power_on()`

- **功能:** atom 打开通信（默认打开）

#### `power_off()`

- **功能:** 机械臂掉电

#### `is_power_on()`

- **功能:** 判断机械臂是否上电

- **返回值:**
  - `1`: 上电
  - `0`: 掉电
  - `-1`: 错误

#### `release_all_servos()`

- **功能：** 放松所有机械臂关节
- **参数**：`data`（可选）：关节放松方式，默认为阻尼模式，若提供 `data`参数可指定为非阻尼模式（1-Undamping）。

#### `focus_servo(servo_id)`

- **功能:** 单个舵机上电

- **参数:**
- `servo_id:` int, 1-6

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
      | 1 | -165 ~ 165 |
      | 2 | -90 ~ 90 |
      | 3 | -180 ~ 65 |
      | 4 | -165 ~ 165 |
      | 5 | -115 ~ 115 |
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
      | x | -272 ~ 272 |
      | y | -272 ~ 272 |
      | z | -36 ~ 408.9 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |
  - `speed`：（`int`）1-100

#### `send_coords(coords, speed, mode)`

- **功能：**：发送整体坐标和姿态，将机械臂头部从原点移动到您指定的点
- **参数：**
  - `coords`: 坐标列表，值`[x,y,z,rx,ry,rz]`，长度6
  - `speed` (`int`)：1 ~ 100
  - `mode：`(`int`) 0 - 非线性，1 - 直线运动

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

#### `sync_send_coords(coords, speed，mode=0, timeout=15)`

- **功能：** 同步状态下发送坐标，到达目标点后返回
- **参数：**
  - `coords`：坐标值列表（`List[float]`），长度6
  - `speed`：（`int`）1~100
  - `mode`：（`int`）0-非线性（默认），1-直线运动
  - `timeout`: 默认15秒

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

#### `jog_coord(coord_id, direction, speed)`

- **功能：** 点动控制坐标
- **参数：**
  - `coord_id`: (`int`) 机械臂坐标范围：1~6
  - `direction`: (`int`) 控制机臂运动方向，`0` - 负值运动，`1` - 正值运动
  - `speed`: 1 ~ 100

#### `jog_rpy(end_direction, direction, speed)`

- **功能：** 使末端绕基坐标系中固定轴旋转
- **参数：**
  - `end_direction`: (`int`) Roll、Pitch、Yaw（1-3）
  - `direction`: (`int`) 控制机臂运动方向，`1` - 正转，`0` - 反转
  - `speed`: (`int`) 1 ~ 100

#### `jog_increment_angle(joint_id, increment, speed)`

- **功能**：角度步进，单关节角度增量控制
- **参数**：
- `joint_id`：1-6
- `increment`：基于当前位置角度的增量移动
- `speed`：1~100

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
  - `angle`: 参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得小于最小值

#### `set_joint_max(id, angle)`

- **功能：** 设置最大关节角度限制
- **参数：**
  - `id` ：输入关节ID（范围1-6）
  - `angle`：参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得大于最大值
    
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

### 7. 伺服状态值

#### `get_servo_speeds()`

- **功能**：获取所有关节的运动速度
- **返回值**： 一个列表

#### `get_servo_voltages()`

- **功能**：获取关节电压
- **返回值**： 一个列表， 电压值小于24 V

#### `get_servo_status()`

- **功能**：获取所有关节的运动状态
- **返回值**： 列表，[电压，传感器，温度，电流，角度，过载]，值为`0`表示无错误，值为`1`表示有错误

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
  
#### `set_digital_output(pin_no, pin_signal)`

- **功能:** 设置IO状态
- **参数**
  - `pin_no` (int): 引脚号
  - `pin_signal` (int): 0 / 1, 输入0表示设置为运行状态，输入1表示停止状态

#### `get_digital_input(pin_no)`

- **功能:** 获取IO状态
- **参数**: `pin_no` (int)
- **返回值**: 当返回的值为0表示在工作状态运行，1表示停止状态

#### `set_pin_mode(pin_no, pin_mode)`

- **功能:** 设置原子中指定引脚的状态模式。
- **参数**
  - `pin_no` (int): 引脚号
  - `pin_mode` (int): 0 - 运行状态, 1 - 停止状态, 2 - 上拉模式

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

#### `set_gripper_calibration()`

- **功能**: 将夹爪的当前位置设置为零位

#### `is_gripper_moving()`

- **功能**: 判断夹爪是否正在运行
- **返回值**：
  - `0`: 没有运行
  - `1`: 正在运行
  - `-1`: 错误

#### `get_gripper_value()`

- **功能**: 获取夹爪的当前位置数据信息
- **参数**:
  - `gripper_type`: (int) 默认 1
    - 1:  自适应夹爪
    - 3: 平行夹爪
    - 4: 柔性夹爪
- **返回值**：夹爪的数据信息 (int)
  
#### `set_pwm_output(channel, frequency, pin_val)`

- **功能**: PWM 控制
- **参数**:
- `channel` : (int): IO 编号。
- `frequency`: (int): 时钟频率
- `pin_val`: (int) 占空比 0 ~ 256；128 表示 50%

#### `set_HTS_gripper_torque(torque)`

- **功能**: 设置自适应夹爪力矩
- **参数**: 
  - `torque`: (int): 150 ~ 980
- **返回值**:
  - `0`: 设置失败
  - `1`: 设置成功

#### `get_HTS_gripper_torque()`

- **功能**: 获取自适应夹爪力矩
- **返回值**:  (int) 150 ~ 980

#### `get_gripper_protect_current()`

- **功能**: 获取夹爪保护电流
- **返回值**:  (int) 1 ~ 500

#### `set_gripper_protect_current(current)`

- **功能**: 设置夹爪保护电流
- **参数**: 
  - `current`: (int): 1 ~ 500
  
#### `init_gripper()`

- **功能**: 初始化夹爪

### 10. 设置底部IO输入/输出状态

#### `set_basic_output(pin_no, pin_signal)`

- **功能**：设置底部引脚号的工作状态。
- **参数**：
  - `pin_no` (`int`) 设备底部标注的编号仅取数字部分
  - `pin_signal` (`int`): 0 - 低电平，设置为运行状态. 1 - 高电平，停止状态。

#### `get_basic_input(pin_no)`

- **功能:** 获取底部引脚号的工作状态
- **参数:**
  - `pin_no` (`int`) 表示机器人底部的具体引脚号。
- **返回值:** 0 - 低电平，运行状态. 1 - 高电平，停止状态

### 11. WLAN 设置

#### `set_ssid_pwd(account, password)`

- **功能:** 更改连接的wifi. (适用于M5)
- **参数:**
  - `account` (`str`) 新的 wifi 账户
  - `password` (`str`) 新的 wifi 密码

#### `get_ssid_pwd()`

- **功能:** 获取连接的wifi账号和密码. (适用于M5)
- **返回值:** 当前连接的wifi账号和密码

#### `set_server_port(port)`

- **功能:** 更改服务器的连接端口
- **参数:**
  - `port` (`int`) 服务器的新连接端口

### 12. TOF

#### `get_tof_distance()`

- **功能:** 获取检测到的距离(需要外部距离检测器)
- **返回值:** 检测到的距离值，单位为mm。

### 13. 通信模式

#### `set_transponder_mode(mode)`

- **功能:** 设置串口传输模式
- **参数:**
  - `mode` : 0 - 关闭透传，1 - 打开透传

#### `get_transponder_mode()`

- **功能:** 获得串口传输的配置信息
- **返回值:**
   - `1`: 打开透传，检测所有数据
   - `0`: 关闭透传

### 14. 坐标变换

#### `set_tool_reference(coords)`

- **功能:** 设置工具坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

#### `get_tool_reference(coords)`

- **功能:** 获取工具坐标系
- **返回值:** (`list`) [x, y, z, rx, ry, rz]

#### `set_world_reference(coords)`

- **功能:** 设置世界坐标系
- **参数**：
  - `coords`: (`list`) [x, y, z, rx, ry, rz].

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
  - `move_type`: 1 - movel, 0 - moveJ.

#### `get_movement_type()`

- **功能:** 获取移动类型
- **返回值:**
  - `1` - movel
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


### 15. 树莓派 -- GPIO

#### `gpio_init()`

- **功能**: 初始化 GPIO 模块，设置 BCM 模式

#### `gpio_output(pin, v)`

- **功能**: 设置GPIO输出值

- **参数**

  - `pin` (`int`) 引脚编号
  - `v` (`int`):0 -设置为低电平 1 -设置为高电平

### 16. utils（模块）

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

## MechArm 270 Socket

> 注意：
> raspberryPi版本 仅支持python3
> 使用此类前提的机械臂有服务器，并且已经开启服务。

使用TCP/IP控制机械臂

### 客户端

```python
# 示例
from pymycobot import MechArm270Socket
# 默认使用端口 9000
mc = MechArm270Socket("192.168.10.10",9000)

res = mc.get_angles()
print(res)

mc.send_angles([0,0,0,0,0,0],20)
...
```

### 服务端

服务端文件在`demo文件夹`中，具体请检查demo文件夹中的[Server_270.py](../demo/Server_270.py)文件

### socket 控制

> 注意：
> 大部分方法与 MechArm270 类相同，这里只列出新方法。


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
