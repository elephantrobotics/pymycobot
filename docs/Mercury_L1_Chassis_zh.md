# Mercury L1 Chassis Python API 使用

## 1. 系统 & 产品信息

### get_system_version()

- **功能:** 获取主固件版本号
- **返回值:**
  - **float: version**

### get_modify_version()

- **功能:** 获取次固件版本号
- **返回值:**
  - **int: version**

### power_on()

- **功能:** 开启机器人
- **返回值:**
  - **int: 开机结果, 1: 成功, 0: 失败**

### power_on_only()

- **功能:** 开启机器人，但不启动控制程序
- **返回值:**
  - **int: 仅开机结果, 1: 成功, 0: 失败**

### power_off()

- **功能:** 关闭机器人
- **返回值:**
  - **int: 仅关机结果, 1: 成功, 0: 失败**

### is_power_on()

- **功能:** 检查机器人是否已开机
- **返回值:**
  - **int: 电源状态, 1: 开机, 0: 关机**

## 2. 运动控制

### move_backward(speed)

- **功能:** 平移机器人向后
- **参数:**
  - **speed(float): 0 ~ 1.5 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### move_forward(speed)

- **功能:** 平移机器人向前
- **参数:**
  - **speed(float): 0 ~ 1.5 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### turn_left(speed)

- **功能:** 向左旋转
- **参数:**
  - **speed:**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### turn_right(speed)

- **功能:** 向右旋转
- **参数:**
  - **speed:**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### stop()

- **功能:** 停止移动
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### set_auto_report_state(state)

- **功能:** 设置自动报告状态
- **参数:**
  - **state(int): 0: 关闭, 1: 开启**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### get_auto_report_state()

- **功能:** 获取自动报告状态
- **返回值:**
  - **int: 0: 关闭, 1: 开启**

### get_auto_report_message()

- **功能:** 获取自动报告消息
- **返回值:**
  - **list[int | list[int] | float]:**
  - **0 - (float)rx**
  - **1 - (float)ry**
  - **2 - (float)rw**
  - **3 - (list[int])机器状态**
  - **4 - (list[int])电机信息**
  - **5 - (float)电池电压**
  - **6 - (int)电机使能状态 0: 使能, 1: 禁用**

## 3. 电机辅助

### get_motor_enable_status()

- **功能:** 获取电机使能状态
- **返回值:**
  - **list[int]: 电机使能状态**
  - **0: 禁用**
  - **1: 使能**

### get_motor_status()

- **功能:** 获取电机状态
- **返回值:**
  - **list[int]: 电机状态**
  - **0: 正常**
  - **any: 错误代码**

### get_motor_temps()

- **功能:** 获取电机温度
- **返回值:**
  - **list[float]: 电机温度**

### get_motor_speeds()

- **功能:** 获取电机速度
- **返回值:**
  - **list[float]: 电机速度**

### get_motor_torques()

- **功能:** 获取电机扭矩
- **返回值:**
  - **list[float]: 电机扭矩**

### set_communication_state(state)

- **功能:** 设置通讯状态
- **参数:**
  - **state(int):**
  - **0: 串口通讯 (默认)**
  - **1: Socket 通讯**
  - **2: 蓝牙通讯 (将MAC地址写入文件和端点，然后返回状态)**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### get_communication_state()

- **功能:** 获取通讯状态
- **返回值:**
  - **int: 通讯状态**
  - **0: 串口通讯,**
  - **1: Socket 通讯,**
  - **2: 蓝牙通讯**

### set_led_color(position, brightness, color)

- **功能:** 设置LED颜色
- **参数:**
  - **position(int):**
  - **0: 左侧LED**
  - **1: 右侧LED**
  - **color(tuple(int, int, int)): RGB 颜色**
  - **brightness(int): 0 - 255(默认 255)**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### get_motor_loss_count()

- **功能:** 获取电机丢步计数
- **返回值:**
  - **list[int]: 电机丢步计数**

## 4. IO 控制

### get_pin_input(pin)

- **功能:** 获取输入IO
- **参数:**
  - **pin(int): 1 - 6**
- **返回值:**
  - **int: 0: 低电平, 1: 高电平, -1: 没有这个引脚**

### set_pin_output(pin, state)

- **功能:** 设置输出IO
- **参数:**
  - **pin(int): 1 - 6**
  - **state(int): 0: 低电平, 1: 高电平**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### get_estop_state()

- **功能:** 获取急停状态
- **返回值:**
  - **int: 0: 释放, 1: 按下**

## 5. WiFi & 蓝牙

### get_wifi_ip_port()

- **功能:** 获取wi-fi ip和端口
- **返回值:**
  - **tuple(str, int): wi-fi ip, wi-fi 端口**

### get_wifi_account()

- **功能:** 获取wi-fi 账号
- **返回值:**
  - **tuple(str, str): wi-fi 账号, wi-fi 密码**

### get_bluetooth_address()

- **功能:** 获取蓝牙MAC地址
- **返回值:**
  - **str: 蓝牙MAC地址**

### get_bluetooth_uuid()

- **功能:** 获取蓝牙uuid
- **返回值:**
  - **tuple(str, str, str): 蓝牙名称, 服务uuid, 特征uuid**

## 6. 升降

### set_lift_control(mileage, speed)

- **功能:** 控制升降运动
- **参数:**
  - **mileage(int): 运动里程，范围 -450 ~ 450 毫米。**
  - **speed(int): 运动速度，范围 1 ~ 100。**
- **返回值:**
  - **int: 1**

### get_lift_information()

- **功能:** 获取升降信息
- **返回值:**
  - **list: 例如 [当前里程, 异常, 速度, 力矩]**

### get_lift_encoder()

- **功能:** 读取升降编码器值。
- **返回值:**
  - **list: 例如 [零位编码器, 当前编码器]**

### get_lift_recv_loss_count()

- **功能:** 查看升降电机丢包次数
- **返回值:**
  - **int: 例如 1。**

## 7. 使用案例

### 7.1 获取Mercury L1 Chassis的系统版本号

```python
from pymycobot import MercuryL1Chassis

# 初始化 Mercury L1 Chassis 对象
mlc = MercuryL1Chassis("/dev/ttyCH341USB0", baudrate=1000000, debug=True)

# 获取系统版本号
version = mlc.get_system_version()
print(version)
```

### 7.2 控制Mercury L1 Chassis以0.5ms的速度前进3秒

```python
import time
from pymycobot import MercuryL1Chassis

# 初始化 Mercury L1 Chassis 对象
mlc = MercuryL1Chassis("/dev/ttyTHS1", baudrate=1000000, debug=True)

# 控制 Mercury L1 Chassis 以 0.5ms 的速度前进
mlc.move_forward(0.5)

# 睡眠 3 秒
time.sleep(3)

# 停止移动
mlc.stop()
```
