# Mercury L1 Python Socket API
[toc]

## 使用前准备

在使用 Python API 之前，请先确认以下硬件和环境准备齐全：

- **硬件设备**  
  - Mercury L1 机械臂  
  - 网线（用于连接机械臂与电脑）  
  - 电源适配器  
  - 急停开关（确保安全操作）

- **软件与环境**  
  - 已安装 Python 3.9 及以上版本  
  - 已安装 `pymycobot` 库（通过 `pip install pymycobot` 终端命令安装）  
  - 确保 Mercury L1 已正确接通电源，并处于待机状态  
  - **注意**：L1 服务端会在设备上电后自动启动，无需手动操作  

- **网络配置**  
  - Mercury L1 默认 IP 地址：`192.168.1.232`  
  - 默认端口号：`6501`  
  - **注意**：PC 端需要将本机网卡 IP 设置为 **同一网段**（例如 `192.168.1.xxx`，`xxx` 为 2~254 之间的任意数，且不能与机械臂冲突）。  
  - 示例：  
    - 机械臂 IP：`192.168.1.232`  
    - PC IP：`192.168.1.100`  
    - 子网掩码：`255.255.255.0`
    - DNS服务器：`114.114.114.114`
  
  - **验证**：完成网络配置后，可在 PC 终端执行以下命令，若能成功返回数据包，则说明网络连接正常：  
  
    ```bash
    ping 192.168.1.232
    ```

---

## 手臂 API 使用介绍

API（Application Programming Interface），又称应用程序编程接口函数，是预先定义好的函数。使用以下函数接口时，请在一开始就导入我们的API库，导入方式为输入如下代码，否则将无法成功运行：

**注意：** 使用前需确保Mercury L1已开启服务端，并且 PC 与机械臂处于同一网段

```python
# 示例
from pymycobot import MercuryL1Client

# IP地址默认是"192.168.1.232"，端口号默认是6501
mc = MercuryL1Client('192.168.1.232', 6501)

if mc.is_power_on() !=1:
    mc.power_on(0)

print(mc.get_angles())
```

### 1. 系统状态

#### `get_system_version()`

- **功能：** 读取机器主控版本
- **返回值：** 主控版本号

<!-- #### `get_modified_version()`

- **功能：** 读取修正版本号，仅内部使用
- **返回值：** 修正版本号 -->

#### `get_robot_type()`

- **功能：** 检测机器型号

- **返回值：** 定义规则：实际机器型号。例如，Mercury L1 型号为 6501

#### `get_atom_version(arm_id)`

- **功能：** 读取末端版本号
- **参数:** (`int`) `arm_id` 手臂ID
  - `1`: 左臂
  - `2`: 右臂
- **返回值：** 版本号(`float`)

#### `get_tool_modify_version(arm_id)`

- **功能：** 读取末端更正版本号
- **参数:** (`int`) `arm_id` 手臂ID
  - `1`: 左臂
  - `2`: 右臂
- **返回值：** 更正版本号

### 2. 机器人整体运行状态

#### `power_on(arm_id)`

- **功能：** 启动机器人，上电
- **参数:** (`int`) `arm_id` 手臂ID
  - `0`: 左臂和右臂
  - `1`: 左臂
  - `2`: 右臂
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
  - `1`: 刷新-总是首先执行最新的命令。
  - `0`: 插补-以队列的形式按顺序执行指令。

<!-- #### `get_debug_state()`

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
  - -1-错误 -->

#### `get_free_move_mode()`

- **功能:** 读取自由移动模式

- **返回值:** 
  - `0`: 关闭自由移动模式
  - `1`: 打开自由移动模式

#### `set_free_move_mode(mode)`

- **功能:** 设置自由移动模式（仅当打开自由移动后，按住末端按钮才可放松关节）
  
- **参数:**
  - `1`: 打开自由移动模式。
  - `0`: 关闭自由移动模式。

### 3. 机器人异常检测

#### `get_robot_status()`

- **功能：** 读取左右臂机器人错误安全状态
- **返回值:** `list` ,  `[[左臂状态], [右臂状态]]`, 0 - 正常。比如 `[[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]`，其他 - 机器人异常
  - **左臂**
    - `[关节是否碰撞，是否正在运动，J1是否超限，J2是否超限，J3是否超限，J4是否超限，J5是否超限，J6是否超限，J7是否超限，J8是否超限，J1是否电机硬件报错，J2是否电机硬件报错，J3是否电机硬件报错，J4是否电机硬件报错，J5是否电机硬件报错，J6是否电机硬件报错，J7是否电机硬件报错，J8是否电机硬件报错，J1是否软件通信报错，J2是否软件通信报错，J3是否软件通信报错，J4是否软件通信报错，J5是否软件通信报错，J6是否软件通信报错，J7是否软件通信报错，J8是否软件通信报错]`
  - **右臂**
    - `[关节是否碰撞，是否正在运动，J1是否超限，J2是否超限，J3是否超限，J4是否超限，J5是否超限，J6是否超限，J7是否超限，J8是否超限，J9是否超限，J1是否电机硬件报错，J2是否电机硬件报错，J3是否电机硬件报错，J4是否电机硬件报错，J5是否电机硬件报错，J6是否电机硬件报错，J7是否电机硬件报错，J8是否电机硬件报错，J9是否电机硬件报错，J1是否软件通信报错，J2是否软件通信报错，J3是否软件通信报错，J4是否软件通信报错，J5是否软件通信报错，J6是否软件通信报错，J7是否软件通信报错，J8是否软件通信报错，J9是否软件通信报错]`

#### `servo_restore(arm_id, joint_id)`

- **功能**：清除关节异常
- **参数**：
  - `arm_id`: `int` 手臂ID  
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id`: int. 关节 id 1 - 7，254-所有关节恢复。

#### `get_comm_error_counts(arm_id, joint_id)`

- **功能**：读取通信异常次数
- **参数**：
  - `arm_id`: `int` 手臂ID  
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id`: int. 关节 id 1 - 9
- **返回值**： `list` 长度为4的列表，比如[0, 0, 0, 0]，分别代表：
  - `[0]`: 关节发送异常次数
  - `[1]`: 关节读取异常次数
  - `[2]`: 末端发送异常次数
  - `[3]`: 末端发送异常次数

#### `get_error_information()`

- **功能**：读取机器人错误信息
- **返回值**：`list[int]`, [左臂状态，右臂状态]
  - `0`：无错误信息
  - `1~6`：对应关节超出限位位置。
  - `32~36`：坐标运动异常。
    - `32`：坐标无解，请检查臂展是否临近限位
    - `33`：直线运动无相邻解。
    - `34`: 速度融合报错
    - `35`：零空间运动无相邻解
    - `36`：奇异位置无解，请使用关节控制离开奇异点
  - `81~86`: J1 ~ J6关节触发碰撞，请使用`resume`接口恢复

#### `clear_error_information(arm_id)`

- **功能**：清除机器人错误信息
- **参数:** (`int`) `arm_id` 手臂ID
  - `0`: 左臂和右臂
  - `1`: 左臂
  - `2`: 右臂

#### `over_limit_return_zero()`

- **功能** 机器关节超限回零指令

#### `get_motors_run_err()`

- **功能**：读取机器人运动中的电机错误信息
- **返回值**：`list`, 长度为6的列表，全部是0，代表正常

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
- **返回值**：`list`左右臂所有角度的浮点列表，`[[左臂角度], [右臂角度]]`

#### `get_angle(joint_id)`

- **功能：** 获取单关节的角度
- **参数：**
  - `joint_id`: `int`，关节ID， 范围1 ~ 9
- **返回值**：`list[float]` 双臂单关节角度, `[left angle, right angle]`

#### `send_angle(arm_id, joint_id, speed, left_angle=None, right_angle=None, _async=False)`

- **功能：** 向机械臂发送一个关节角度（arm_id模式为1/2时，另一条臂值任意）
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂，需同时输入 `left_angle` 和 `right_angle` 参数。
    - `1`: 左臂，只输入 `left_angle` 参数即可。
    - `2`: 右臂，只输入 `right_angle` 参数即可。
  - `joint_id`：关节 id，范围 int 1-9
  - `speed`：（`int`）1 ~ 100
  - `left_angle`：角度值（`float`）
      | 关节 Id | 范围 |
      | ---- | ---- |
      | 1 | -181 ~ 135 |
      | 2 | -46 ~ 96 |
      | 3 | -155 ~ 155 |
      | 4 | -135 ~ 18 |
      | 5 | -155 ~ 155 |
      | 6 | -115 ~ 115 |
      | 7 | -155 ~ 155 |
      | 8（腰部） | 0 ~ 40 |
  
  - `right_angle`：角度值（`float`）
      | 关节 Id | 范围 |
      | ---- | ---- |
      | 1 | -181 ~ 135 |
      | 2 | -46 ~ 96 |
      | 3 | -155 ~ 155 |
      | 4 | -135 ~ 18 |
      | 5 | -155 ~ 155 |
      | 6 | -115 ~ 115 |
      | 7 | -155 ~ 155 |
      | 8（脖子） | -50 ~ 50 |
      | 9（头部） | -82 ~ 82 |
  - `_async`: 运动闭环开关，默认开启-False；关闭-True。

#### `send_angles(arm_id, speed, left_angles=None, right_angles=None, _async=False)`

- **功能：** 将所有角度发送到机械臂的所有关节（arm_id模式为1/2时，另一条臂值任意）
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂，需同时输入 `left_angles` 和 `right_angles` 参数。
    - `1`: 左臂，只输入 `left_angles` 参数即可。
    - `2`: 右臂，只输入 `right_angles` 参数即可。
  - `speed`：（`int`）1 ~ 100
  - `left_angles`：度数列表（`List[float]`），长度 8
  - `right_angles`：度数列表（`List[float]`），长度 9
  - `_async`: 运动闭环开关，默认开启-False；关闭-True。

#### `get_coords()`

- **功能：** 从基于基准的坐标系获取机械臂坐标
- **返回值：** 左右臂坐标浮点列表：`[[x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz]]`

#### `send_coord(arm_id, coord_id, speed, left_coord=None, right_coord=None, _async=False)`

- **功能：** 向机械臂发送一个坐标（arm_id模式为1/2时，另一条臂值任意）
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂，需同时输入 `left_coord` 和 `right_coord` 参数。
    - `1`: 左臂，只输入 `left_coord` 参数即可。
    - `2`: 右臂，只输入 `right_coord` 参数即可。
  - `coord_id`：向机械臂发送一个坐标，1-6 对应 [x, y, z, rx, ry, rz]
  - `speed` (`int`)：1 ~ 100
  - `left_coord`：坐标值（`float`）
      | 坐标 ID | 范围 |
      | ---- | ---- |
      | x | -1000 ~ 1000 |
      | y | -474 ~ 1000 |
      | z | -1000 ~ 1000 |
      | rx | -180 ~ 180 |
      | ry | -180 ~ 180 |
      | rz | -180 ~ 180 |
  - `right_coord`：坐标值（`float`）,范围同 `left_coord` 一样。
  - `_async`: 运动闭环开关，默认开启-False；关闭-True。

#### `send_coords(arm_id, speed, left_coords=None, right_coords=None, _async=False)`

- **功能：**：发送整体坐标和姿态，将机械臂头部从原点移动到您指定的点（arm_id模式为1/2时，另一条臂值任意）
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂，需同时输入 `left_coords` 和 `right_coords` 参数。
    - `1`: 左臂，只输入 `left_coords` 参数即可。
    - `2`: 右臂，只输入 `right_coords` 参数即可。
  - `speed` (`int`)：1 ~ 100
  - `left_coords`: 坐标列表，值`[x,y,z,rx,ry,rz]`，长度6
  - `right_coords`: 坐标列表，值`[x,y,z,rx,ry,rz]`，长度6
  - `_async`: 运动闭环开关，默认开启-False；关闭-True。

#### `pause(arm_id, deceleration=0)`

- **功能：** 控制指令暂停核心并停止所有运动指令
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `deceleration`： 是否减速并停止，默认为 0。0和1均代表缓暂停
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

#### `resume(arm_id)`

- **功能：** 恢复机器人运动并完成之前的命令
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  
#### `stop(arm_id, deceleration=0)`

- **功能：** 停止机器人运动
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `deceleration` ： 是否减速并停止。默认为 0。0和1均代表缓停
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

#### `jog_angle(arm_id, joint_id, direction, speed)`

- **功能：** jog 控制角度，关节持续运动
- **参数**:
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id`: 表示机械臂的关节ID，范围 1 ~ 9
  - `direction(int)`: 控制机械臂运动方向，输入`0`为负值方向运动，输入`1`为正值方向运动
  - `speed`: 1 ~ 100

#### `jog_coord(arm_id, coord_id, direction, speed)`

- **功能：** jog 控制坐标， 坐标持续运动.
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `coord_id`: (`int`) 机械臂坐标轴范围：1~6
  - `direction`:(`int`) 控制机械臂运动方向，输入`0`为负值方向运动，输入`1`为正值方向运动
  - `speed`: 1 ~ 100

#### `jog_increment_angle(arm_id, joint_id, increment, speed)`

- **功能：** 单关节角度增量控制
- **参数**:
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id`: 1-9
  - `increment`: 基于当前位置角度的增量移动
  - `speed`: 1 ~ 100

#### `jog_increment_coord(arm_id, coord_id, increment, speed)`

- **功能：** 单坐标增量控制
- **参数**:
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `coord_id`: 坐标轴 1 - 6.
  - `increment`: 基于当前位置坐标的增量移动
  - `speed`: 1 ~ 100

### 6. 速度/加速度参数

#### `get_max_speed(mode)`

- **功能:** 获取最大运动速度
- **参数:**
  - `mode` : `int`
    - `0`: 角度速度
    - `1`: 坐标速度
- **返回值**：角速度范围1～150°/s，坐标速度范围1～200mm/s

#### `set_max_speed(arm_id, mode, max_speed)`

- **功能:** 设置最大运动速度
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
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

#### `set_max_acc(arm_id, mode, max_acc)`

- **功能:** 设置最大运动加速度
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `mode` : `int`
    - `0`: 角度加速度
    - `1`: 坐标加速度
  - `max_acc`: 角度加速度范围1～150°/s，坐标加速度范围1～400mm/s

### 7. 软件关节限位

#### `get_joint_min_angle(joint_id)`

- **功能:** 获取指定关节的最小运动角度
- **参数:**
  - `joint_id` : 输入关节ID（范围1-9）
- **返回值**：`float` 角度值

#### `get_joint_max_angle(joint_id)`

- **功能:** 获取指定关节的最大运动角度
- **参数:**
  - `joint_id` : 输入关节ID（范围1-9）
- **返回值**：`float` 角度值

#### `set_joint_min_angle(arm_id, joint_id, angle)`

- **功能:** 设置最小关节角度限制
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id` : 输入关节ID（范围1-6）
  - `angle`: 参考[send_angle()](#send_angleid-degree-speed)接口中对应关节的限制信息，不得小于最小值

#### `set_joint_max_angle(arm_id, joint_id, angle)`

- **功能：** 设置最大关节角度限制
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id` ：输入关节ID（范围1-6）
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

#### `set_servo_calibration(arm_id, servo_id)`

- **功能：** 校准关节执行器的当前位置为角度零点
- **参数**:
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `servo_id`: 1 - 9

#### `set_break（joint_id, value）`

- **功能：** 设置关节刹车
- **参数**：
  - `joint_id`: int. 关节 id 1 - 6
  - `value`: int. 0 - 掉使能, 1 - 使能
- **返回值:** 0 : 失败; 1 : 成功

#### `set_motor_enabled(arm_id, joint_id, state`

- **功能：** 设置机器人力矩状态。（释放关节接口）
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id`: int. 关节 id 1 - 9, 254-所有关节
  - `state`: int. 0 - 掉使能, 1 - 使能

### 9. 拖动示教

#### `drag_teach_save()`

- **功能：** 开始录制并拖动教学点。
  - 注意：为了呈现最佳运动效果，录制时间请勿超过120秒

#### `drag_teach_pause()`

- **功能：** 暂停拖动示教

#### `drag_teach_execute()`

- **功能：** 开始拖动示教点，仅执行一次。

#### `drag_teach_clean()`

- **功能：** 清除采样点。

#### `set_pro_gripper_offset(offset=2)`

- **功能：** 设置Pro力控夹爪偏移。执行带夹爪拖动示教轨迹时，实际执行的夹爪角度会减去offset（默认为2）
- **参数**： `int`
  - `offset`: 范围 -5 ~ 5，默认2

#### `get_pro_gripper_offset()`

- **功能：** 获取Pro力控夹爪偏移。执行带夹爪拖动示教轨迹时，实际执行的夹爪角度会减去offset（默认为2）
- **返回值**： `int`，范围 -5 ~ 5

### 10. 动力学

#### `get_collision_mode()`

- **功能**: 查询碰撞检测模式
- **返回值:**
  - `0`: 关闭
  - `1`: 打开

#### `set_collision_mode(mode)`

- **功能**设置关节碰撞检测
- **参数**： `int`
  - `mode`:
    - `0`: 关闭
    - `1`: 打开

#### `set_collision_threshold(joint_id, threshold_value=100)`

- **功能**设置关节碰撞检测
- **参数**：
  - `mode`: `int` 关节ID，范围 1 ~ 6
  - `threshold_value`: `int` 碰撞阈值，范围为 50 ~ 250，默认值为 100，值越小，越容易触发碰撞

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

### 11. 圆弧运动

#### `write_move_c(transpoint, endpoint, speed)`

- **功能**：圆弧轨迹运动(指定途经点)
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
- **返回值**： 一个嵌套列表，`[[左臂速度], [右臂速度]]`

#### `get_servo_currents()`

- **功能**：获取关节电流
- **返回值**：一个嵌套列表, 0 ~ 5000 mA，`[[左臂电流], [右臂电流]]`

#### `get_servo_status()`

- **功能**：获取所有关节的运动状态
- **返回值**： 一个嵌套列表，值为 0 表示没有错误，`[[左臂状态], [右臂状态]]`

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

### 16. 设置末端485通信

#### `tool_serial_write_data(arm_id, command)`

- **功能：** 末端485发送数据，数据长度范围为1~45字节
- **参数**： 
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `command` (`list`): modbus格式的数据指令
- **返回值:** modbus数据列表

#### `flash_tool_firmware(arm_id, main_version, modified_version=0)`

- **功能：** 烧录末端固件
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `main_version (str)`: 主次版本号，比如 `'1.1'`
  - `modified_version (int)`: 更正版本号，范围 0 ~ 255，默认是 0 

#### `set_tool_serial_baud_rate(arm_id, baud_rate=115200)`

- **功能：** 设置末端485波特率，默认115200
- **参数**: 
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `baud_rate` (`int`): 标准波特率，仅支持115200和1000000
- **返回值:** 1

#### `set_tool_serial_timeout(arm_id, timeou=10000)`

- **功能：** 设置末端485超时时间，默认10秒
- **参数**: 
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `timeout (int)`: 超时时间， 单位毫秒，范围 1 ~ 10000
- **返回值:** 1 

#### `get_tool_config(arm_id)`

- **功能：** 获取末端485波特率和超时时间
- **参数：**
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
- **返回值:** (`list`) 包含波特率和超时时间的列表，比如：[波特率, 超时时间]

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

<!-- #### `get_vr_mode()`

- **功能:** 获取VR模式
- **返回值:**
  - `0`: 关闭
  - `1`: 打开

#### `set_vr_mode(move)`

- **功能:** 设置VR模式
- **参数**：
  - `move`: 1 - 打开, 0 - 关闭. -->

#### `get_model_direction()`

- **功能:** 获取关节模型方向
- **返回值:** 1-6关节的模型方向
  - `1` - 与电机同向
  - `0` - 与电机反向

#### `set_model_direction(arm_id, joint_id, direction)`

- **功能:** 设置关节模型方向
- **参数:**
  - `arm_id`: (`int`) 手臂ID
    - `0`: 左臂和右臂
    - `1`: 左臂
    - `2`: 右臂
  - `joint_id (int)`: 1 ~ 9
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
- **返回值:** `int` 1 ~ 255

#### `set_filter_len(rank, value)`

- **功能:** 设置滤波器参数
- **参数:**
  - `rank (int)`: 1 ~ 5
    - `1`：拖动示教采样滤波器
    - `2`：拖动示教执行滤波器
    - `3`：关节速度融合滤波器
    - `4`：坐标速度融合滤波器
    - `5`：拖动示教采样周期
  - `value (int)`: 1 ~ 255

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

#### `get_pro_gripper_firmware_version(arm_id, gripper_id=14)`

- **功能**：读取Pro力控夹爪固件主次版本号
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

- **返回值**: (`float`) 版本号, x.x

#### `get_pro_gripper_firmware_modified_version(arm_id, gripper_id=14)`

- **功能**：读取Pro力控夹爪固件修正版本号
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。

- **返回值**：(`int`) 修正版本号

#### `set_pro_gripper_id(arm_id, target_id, gripper_id=14)`

- **功能**：设置力控夹爪ID。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `target_id` (`int`): 范围1 ~ 254。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功
  
#### `get_pro_gripper_id(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪ID。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 范围1 ~ 254。

#### `set_pro_gripper_angle(arm_id, gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功
  
#### `get_pro_gripper_angle(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_open(arm_id, gripper_id=14)`

- **功能**：打开力控夹爪。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_close(arm_id, gripper_id=14)`

- **功能**：关闭力控夹爪。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_calibration(arm_id, gripper_id=14)`

- **功能**：设置力控夹爪零位。（首次使用需要先设置零位）
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_status(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪夹持状态。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值:**
  - `0` - 正在运动。
  - `1` - 停止运动，未检测到夹到物体。
  - `2` - 停止运动，检测到夹到物体。
  - `3` - 检测到夹到物体之后，物体掉落。

#### `set_pro_gripper_enabled(arm_id, state, gripper_id=14)`

- **功能**：设置力控夹爪使能状态。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `state` (`bool`) ：0 或者1， 0 - 掉使能 1 - 上使能
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_torque(arm_id, torque_value, gripper_id=14)`

- **功能**：设置力控夹爪扭矩。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `torque_value` (`int`) ：扭矩值，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_torque(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪扭矩。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值:** (`int`) 0 ~ 100

#### `set_pro_gripper_speed(arm_id, speed, gripper_id=14)`

- **功能**：设置力控夹爪速度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `speed` (int): 夹爪运动速度，取值范围 1 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_speed(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪速度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：夹爪默认运动速度，范围 1 ~ 100。

#### `set_pro_gripper_abs_angle(arm_id, gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪绝对角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_io_open_angle(arm_id, gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪IO张开角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_io_open_angle(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪IO张开角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_io_close_angle(arm_id, gripper_angle, gripper_id=14)`

- **功能**：设置力控夹爪IO闭合角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_angle` (`int`): 夹爪角度，取值范围 0 ~ 100。
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_io_close_angle(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪IO闭合角度。
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`): 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：`int` 0 ~ 100

#### `set_pro_gripper_mini_pressure(arm_id, pressure_value, gripper_id=14)`

- **功能**：设置力控夹爪最小启动力
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `pressure_value` (`int`): 启动力值，范围 0 ~ 254。
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_mini_pressure(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪最小启动力
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`int`) 启动力值，范围 0 ~ 254。

#### `set_pro_gripper_protection_current(arm_id, current_value, gripper_id=14)`

- **功能**：设置力控夹爪夹持电流
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `current_value` (`int`): 夹持电流值，范围 100 ~ 300。
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_protection_current(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪夹持电流
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`int`) 夹持电流值，范围 100 ~ 300。

#### `set_pro_gripper_baud(arm_id, baud_rate=0, gripper_id=14)`

- **功能**：设置力控夹爪波特率
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `baud_rate` (`int`): 波特率索引，范围0 ~ 1, 默认 0 - 115200
    - `0` - 115200
    - `1` - 1000000
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `get_pro_gripper_baud(arm_id, gripper_id=14)`

- **功能**：读取力控夹爪波特率
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`int`)  波特率索引，默认 0 - 115200
  - `0` - 115200
  - `1` - 1000000

#### `set_pro_gripper_modbus(arm_id, state, custom_mode=False, gripper_id=14)`

- **功能**：设置力控夹爪modbus通信模式
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `state` (`int`): 范围 0 ~ 1。
    - `0`: 关闭modbus通信模式，打开自定义通信模式
    - `1`: 打开modbus通信模式，关闭自定义通信模式
  - `custom_mode` (`bool`): 自定义通信模式标识，默认False（当前是modbus模式）。如果当前是自定义通信模式，打开modbus通信模式，需要把custom_mode改为True. 比如：`set_pro_gripper_modbus(1, True)`
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：
  - 0 - 失败
  - 1 - 成功

#### `set_pro_gripper_init(arm_id, gripper_id=14)`

- **功能**：夹爪初始化，将夹爪恢复到 **115200波特率** 的Modbus模式
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `gripper_id` (`int`) 夹爪ID，默认14，取值范围 1 ~ 254。
- **返回值**：(`bool`) 
  - `True` - 成功
  - `False` - 失败
  
### 21. 傲意五指灵巧手

#### `get_five_fingers_angles(arm_id, hand_id=2)`

- **功能**：读取五指灵巧手全部关节角度
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。
- **返回值**：(`list[float]`)  五指全关节角度，比如 `[33.54, 173.83, 171.68, 172.1, 174.71, 1.0]` 分别代表 `[大拇指弯曲, 食指, 中指, 无名指, 小指, 大拇指旋转]`。
  
#### `get_five_fingers_angles(arm_id, finger_id, hand_id=2)`

- **功能**：读取五指中单个关节的角度
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `finger_id` (`int`): 范围 1 ~ 6。
    - `1`: 大拇指弯曲
    - `2`: 食指
    - `3`: 中指
    - `4`: 无名指
    - `5`: 小指
    - `6`: 大拇指旋转

  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。

- **返回值**：(`float`) 角度值。

#### `set_five_fingers_angles(arm_id, fingers_angles， hand_id=2)`

- **功能**：设置五指全部关节角度
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `fingers_angles (list)`: 长度为6的列表，J1-J6分别代表 `[大拇指弯曲, 食指, 中指, 无名指, 小指, 大拇指旋转]`
    - `J1`: 2.26° ~ 36.76°
    - `J2`: 100.22° ~ 178.37°
    - `J3`: 97.81° ~ 176.06°
    - `J4`: 101.38° ~ 176.54°
    - `J5`: 98.84° ~ 174.86°
    - `J6`: 0° ~ 90°
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。

#### `set_five_fingers_angle(arm_id, finger_id, finger_angle, hand_id=2)`

- **功能**：设置五指单个关节角度
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `finger_id` (`int`): 范围 1 ~ 6。
    - `1`: 大拇指弯曲
    - `2`: 食指
    - `3`: 中指
    - `4`: 无名指
    - `5`: 小指
    - `6`: 大拇指旋转
  - `finger_angle (int or float)`: 角度值。
    - `J1`: 2.26° ~ 36.76°
    - `J2`: 100.22° ~ 178.37°
    - `J3`: 97.81° ~ 176.06°
    - `J4`: 101.38° ~ 176.54°
    - `J5`: 98.84° ~ 174.86°
    - `J6`: 0° ~ 90°
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。

#### `get_five_fingers_version(arm_id, hand_id=2)`

- **功能**：读取五指灵巧手固件主次版本号
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。
- **返回值**：(`float]`)  固件主次版本号，比如 `3.1`。
  
#### `get_five_fingers_hand_id(arm_id, hand_id=2)`

- **功能**：读取五指灵巧手设备ID
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。

- **返回值**：(`int`) 五指设备ID值。

#### `set_five_fingers_hand_id(arm_id, target_hand_id, hand_id=2)`

- **功能**：设置五指灵巧手设备ID
- **参数**：
  - `arm_id`: (`int`) 手臂ID
    - `1`: 左臂
    - `2`: 右臂
  - `target_hand_id`: (`int`) 需要设置新的五指设备ID，取值范围 0 ~ 254。
  - `hand_id`: (`int`) 五指设备ID，默认2，取值范围 0 ~ 254。