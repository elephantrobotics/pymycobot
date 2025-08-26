# MyArm

[toc]
## API 使用说明

> API (Application Programming Interface), 也称为应用程序编程接口函数，是预定义的函数。使用以下函数接口时，请在开始时输入以下代码导入我们的 API 库，否则将无法成功运行

```python
# 示例
from pymycobot import MyArmM

mam = MyArmM('/dev/ttyAMA1')

# 获取所有关节的当前角度
angles = mam.get_joints_angle()
print(angles)

# 设置关节1移动到40, 速度为20
mam.set_joint_angle(1, 40, 20)
```

## MyArmAPI(M&C)

> 此接口由`MyArmM`和`MyArmC`共享
 
### 1. 机器人状态查询


#### `get_robot_modified_version()`

- **描述:** 获取机器人更正版本号

#### `get_robot_firmware_version()`

- **描述:** 获取机器人固件版本（主次版本）

#### `get_robot_tool_modified_version()`

- **描述:** 获取机器人工具修正版本

#### `get_robot_tool_firmware_version()`

- **描述:** 获取机器人工具固件版本(末端Atom)

#### `set_robot_err_check_state(status)`

- **描述:** 设置错误检测状态 您可以关闭错误检测，但除非必要，否则不要将其关闭

- **参数:**

  - **status (int): 1 打开; 0 关闭**

#### `get_robot_err_check_state()`

- **描述:** 读取错误检测状态

#### `get_robot_error_status()`

- **描述:** 获取机器人错误状态, 此接口返回在15s内

- **返回:**

  - **没有错误返回: [0,0,0,0,0,0,0,0]**

  - **假设在第 1 节中报告了错误 1 和 3，它应该返回：[[1,3]，0,0,0,0,0,0,0,0]**

#### `get_robot_power_status()`

- **描述:** 获取机器人电源状态

- **返回:**

  - **power_status (int): 0: 关机, 1: 开机**

#### `set_robot_power_on()`

- **描述:** 将机器人设置为开机状态

- **返回: (int) 1**

#### `set_robot_power_off()`

- **描述:** 将机器人设置为关机状态

- **返回: (int) 1**

#### `clear_robot_err()`

- **描述:** 清除机器人异常，忽略错误接头，继续移动

#### `set_recv_queue_max_len(max_len)`

- **描述:** 设置接收命令队列的总长度

#### `get_recv_queue_max_len()`

- **描述:** 读取命令队列的总长度，默认长度为 100

#### `clear_recv_queue()`

- **描述:** 清除接收命令的队列

#### `get_recv_queue_len()`

- **描述:** 读取接收队列的当前长度

### 2. 关节伺服控制

#### `get_joint_angle(joint_id)`

- **描述:** 获取指定关节的当前角度

- **参数:**

  - **joint_id (int): 0 - 254**

#### `get_joints_angle()`

- **描述:** 获取所有关节的当前角度

- **返回:**

  - **angles list(int): 0 - 254**

#### `get_joints_max()`

- **描述:** 读取所有关节的最大角度

#### `get_joints_min()`

- **描述:** 读取所有关节的最小角度

### 3. 伺服电机控制

#### `set_servo_calibrate(servo_id)`

- **描述:** 设置指定伺服电机的零位

- **参数:**

  - **servo_id (int): 0 - 254**

#### `get_servo_encoder(servo_id)`

- **描述:** 获取指定伺服电机的当前编码器电位值

- **参数:**

  - **servo_id (int): 0 - 254**

- **返回:**

  - **encoder (int): 0-4095**

#### `get_servos_encoder()`

- **描述:** 获取多个伺服电机的当前编码器电位值

#### `get_servos_speed()`

- **描述:** 获取多个伺服电机的当前运动速度

#### `is_all_servos_enabled()`

- **描述:** 获取多个伺服电机的连接状态

- **返回:**

  - **status: list[int*8] 0：连接失败 1：连接成功**

#### `get_servos_temp()`

- **描述:** 获取多个伺服电机的温度

#### `get_servos_voltage()`

- **描述:** 获取多个伺服电机的电压

#### `get_servos_current()`

- **描述:** 获取多个伺服电机的电流

#### `get_servos_status()`

- **描述:** 获取多个伺服电机的所有状态

#### `get_servos_protect_current()`

- **描述:** 获得多个伺服电机保护电流

#### `set_servo_enabled(joint_id, state)`

- **描述:** 设置伺服电机转矩开关

- **参数:**

  - **joint_id (int): 0-254 254-all**

  - **state: 0/1**
    - **1: 聚焦**
    - **0: 释放**

### 4. 伺服电机系统参数修改

#### `set_servo_p(servo_id, data)`

- **描述:** 设置指定伺服电机的位置环 P 的比例系数

- **参数:**

  - **servo_id (int): 0-254**

  - **data (int): 0-254**

#### `get_servo_p(servo_id)`

- **描述:** 读取指定伺服电机的位置环P比例因子

- **参数:**

  - **servo_id (int): 0-254**

#### `set_servo_i(servo_id, data)`

- **描述:** 设置指定伺服电机的位置环I比例系数

- **参数:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 254**

#### `get_servo_i(servo_id)`

- **描述:** 读取指定伺服电机的位置环I比例系数


#### `set_servo_d(servo_id, data)`

- **描述:** 设置指定伺服电机的位置环D比例系数

- **参数:**

  - **servo_id (int): 0-254**

  - **data (int): 0-254**

#### `get_servo_d(servo_id)`

- **描述:** 读取指定伺服电机的位置环D比例系数

- **参数:**

  - **servo_id (int): 0-254**

#### `set_servo_cw(servo_id, data)`

- **描述:** 设置指定伺服电机的编码器顺时针不灵敏区

- **参数:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 32**
  
#### `get_servo_cw(servo_id)`

- **描述:** 读取指定伺服电机的编码器顺时针不灵敏区

- **参数:**

  - **servo_id (int): 0 - 254**

#### `set_servo_cww(servo_id, data)`

- **描述:** 设置指定伺服电机的编码器逆时针不灵敏区

- **参数:**

  - **servo_id (int): 0 - 254**

  - **data (int): 0 - 32**

#### `get_servo_cww(servo_id)`

- **描述:** 读取指定伺服电机的编码器逆时针不灵敏区

- **参数:**

  - **servo_id (int): 0 - 254**

#### `set_servo_system_data(servo_id, addr, data, mode)`

- **描述:** 设置指定伺服电机的系统参数

- **参数:**

  - **servo_id (int): 0 - 254:**

  - **addr (int):**

  - **data (int): 0 - 4096**

  - **mode (int): 1 - data 1byte. 2 - data 2byte**

#### `get_servo_system_data(servo_id, addr, mode)`

- **描述:** 读取指定伺服电机的系统参数

- **参数:**

  - **servo_id (int): 0 - 254**

  - **addr (int):**

  - **mode (int): 1 - data 1byte. 2 - data 2byte**

### 5. IO 控制

#### `set_master_out_io_state(io_number, status)`

- **描述:** 设置主控引脚状态

- **参数:**

  - **io_number: 1 - 2**

  - **status: 0/1; 0: 低电平; 1: 高电平. 默认: 1**

#### `get_master_in_io_state(io_number)`

- **描述:** 读取主控引脚状态

- **参数:**

  - **io_number (int): 1 - 2**

- **返回:**

  - **0/1. 1: 高电平 0: 低电平**

#### `set_tool_out_io_state(io_number, status)`

- **描述:** 设置末端引脚状态

- **参数:**

  - **io_number (int): 1 - 2**

  - **status: 0/1; 0: 低电平; 1: 高电平. 默认: 1**

#### `get_tool_in_io_state(io_number)`

- **描述:** 读取末端引脚状态

- **参数:**

  - **io_number (int): 1 - 2**

- **返回:**

  - **0 or 1. 1: high 0: low**

### 6. Atom 控制

#### `set_tool_led_color(r, g, b)`

- **描述:** 设置 Atom LED 颜色

- **参数:**

  - **r: 0-255**

  - **g: 0-255**

  - **b: 0-255**

#### `is_tool_btn_clicked()`

- **描述:** 读取 Atom 按下状态

- **返回:**

  - **int: 0 or 1. 1: 按下 0: 未按下**
  
# MyArmM

> 只有`MyArmM`接口可用

### 1. 关节伺服控制

#### `set_joint_angle(joint_id, angle, speed)`

- **描述:** 将各个关节设置为移动到目标角度

- **参数:**

  - **joint_id (int) : 0 - 254**

  - **angle (int) : 0 - 254**

  - **speed (int) : 1 - 100**

#### `set_joints_angle(angles, speed)`

- **描述:** 将所有关节设置为移动到目标角度

- **参数:**

  - **angles (list[int]):  0 - 254**

  - **speed (int): 0 - 100**

#### `is_robot_moving()`

- **描述:** 查看机器人是否在移动

- **返回:**

  - **1: 正在移动**

  - **0: 静止状态**

#### `stop_robot()`

- **描述:** 机器人停止移动

### 2. 伺服电机控制

#### `set_servo_encoder(servo_id, encoder, speed)`

- **描述:** 将单个电机运动设置为目标编码器电位值

- **参数:**

  - **servo_id: (int) 0 - 254**

  - **encoder: (int) 0 - 4095**

  - **speed: (int) 1 - 100**

#### `set_servos_encoder(positions, speed)`

- **描述:** 设置移动到目标的多个电机的编码器电位值

- **参数:**

  - **positions (list[int * 8]): 0 - 4095:**

  - **speed (int): 1 - 100:**

#### `set_servos_encoder_drag(encoders, speeds)`

- **描述:** 将多个具有指定速度的伺服电机设置为目标编码器电位值

### 3. IO 控制
#### `get_assist_in_io_state(io_number)`

- **描述:** 获取辅助引脚状态

- **参数:**

  - **io_number (int): 1 - 6**

- **返回:**

  - **0 or 1. 1: 高电平 0: 低电平**

#### `set_assist_out_io_state(io_number, status)`

- **描述:** 设置辅助引脚状态

- **参数:**

  - **io_number: 1 - 6**

  - **status: 0/1; 0: 低电平; 1: 高电平. 默认: 1**
