# class MyAGVPro()

### 1. 系统 & 产品信息

#### get_system_version():
- **功能:** 获取主固件版本号
- **返回值:**
  - **float: version**

#### get_modify_version():
- **功能:** 获取次固件版本号
- **返回值:**
  - **int: version**

#### power_on():
- **功能:** 开启机器人
- **返回值:**
  - **int: 开启结果, 1: 成功, 0: 失败**

#### power_on_only():
- **功能:** 开启机器人，但不启动控制程序
- **返回值:**
  - **int: 仅开启结果, 1: 成功, 0: 失败**

#### power_off():
- **功能:** 关闭机器人
- **返回值:**
  - **int: 仅关闭结果, 1: 成功, 0: 失败**

#### is_power_on():
- **功能:** 检查机器人是否已开启
- **返回值:**
  - **int: 电源状态, 1: 开启, 0: 关闭**

### 2. 运动控制

#### move_backward(speed):
- **功能:** 平移机器人向后
- **参数:**
  - **speed(float): 0 ~ 1.5 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### move_forward(speed):
- **功能:** 平移机器人向前
- **参数:**
  - **speed(float): 0 ~ 1.5 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### move_left_lateral(speed):
- **功能:** 平移机器人向左
- **参数:**
  - **speed(float): 0 ~ 1 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### move_right_lateral(speed):
- **功能:** 平移机器人向右
- **参数:**
  - **speed(float): 0 ~ 1 m/s**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### turn_left(speed):
- **功能:** 向左旋转
- **参数:**
  - **speed:**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### turn_right(speed):
- **功能:** 向右旋转
- **参数:**
  - **speed:**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### stop():
- **功能:** 停止移动
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### set_auto_report_state(state):
- **功能:** 设置自动报告状态
- **参数:**
  - **state(int): 0: 关闭, 1: 开启**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### get_auto_report_state():
- **功能:** 获取自动报告状态
- **返回值:**
  - **int: 0: 关闭, 1: 开启**

#### get_auto_report_message():
- **功能:** 获取自动报告消息
- **返回值:**
  - **list[int | list[int] | float]:**
  - **0 - (float)rx**
  - **1 - (float)ry**
  - **2 - (float)rw**
  - **3 - (list[int])机器状态**
  - **4 - (list[int])电机信息**
  - **5 - (float)电池电压**
  - **6 - (int)电机使能状态 0: 启用, 1: 禁用**

### 3. 电机辅助

#### get_motor_enable_status():
- **功能:** 获取电机使能状态
- **返回值:**
  - **list[int]: 电机使能状态**
  - **0: 禁用**
  - **1: 启用**

#### get_motor_status():
- **功能:** 获取电机状态
- **返回值:**
  - **list[int]: 电机状态**
  - **0: 正常**
  - **any: 错误代码**

#### get_motor_temps():
- **功能:** 获取电机温度
- **返回值:**
  - **list[float]: 电机温度**

#### get_motor_speeds():
- **功能:** 获取电机速度
- **返回值:**
  - **list[float]: 电机速度**

#### get_motor_torques():
- **功能:** 获取电机扭矩
- **返回值:**
  - **list[float]: 电机扭矩**

#### set_communication_state(state):
- **功能:** 设置通讯状态
- **参数:**
  - **state(int):**
  - **0: 串口通讯 (默认)**
  - **1: Socket 通讯**
  - **2: 蓝牙通讯 (将 MAC 地址写入文件和端点，然后返回到该状态)**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### get_communication_state():
- **功能:** 获取通讯状态
- **返回值:**
  - **int: 通讯状态**
  - **0: 串口通讯,**
  - **1: Socket 通讯,**
  - **2: 蓝牙通讯**

#### set_led_color(position, brightness, color):
- **功能:** 设置 LED 颜色
- **参数:**
  - **position(int):**
  - **0: 左侧 LED**
  - **1: 右侧 LED**
  - **brightness(int): 0 - 255**
  - **color(tuple(int, int, int)): RGB 颜色**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

#### get_motor_loss_count():
- **功能:** 获取电机丢包计数
- **返回值:**
  - **list[int]: 电机丢包计数**

### 4. IO 控制

#### get_pin_input(pin):
- **功能:** 获取输入 IO
- **参数:**
  - **pin(int): 1, 2, 3, 4, 5, 6, 7, 8, 254**
- **返回值:**
  - **int: 0: 低电平, 1: 高电平, -1: 没有这个引脚**

#### set_pin_output(pin, state):
- **功能:** 设置输出 IO
- **参数:**
  - **pin(int): 1 - 6**
  - **state(int): 0: 低电平, 1: 高电平**
- **返回值:**
  - **int: 1: 成功, 0: 失败**

### 5. WiFi & 蓝牙

#### get_wifi_ip_port():
- **功能:** 获取 wi-fi ip 和端口
- **返回值:**
  - **tuple(str, int): wi-fi ip, wi-fi 端口**

#### get_wifi_account():
- **功能:** 获取 wi-fi 账号
- **返回值:**
  - **tuple(str, str): wi-fi 账号, wi-fi 密码**

#### get_bluetooth_address():
- **功能:** 获取蓝牙 MAC 地址
- **返回值:**
  - **str: 蓝牙 MAC 地址**

#### get_bluetooth_uuid():
- **功能:** 获取蓝牙 uuid
- **返回值:**
  - **tuple(str, str, str): 蓝牙名称, 服务 uuid, 特征 uuid**

### 6. 使用案例
#### 6.1 获取 AGVPro 的系统版本号
```python
from pymycobot import MyAGVPro
# 初始化 AGVPro 对象
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)
# 获取系统版本号
version = agv_pro.get_system_version()
print(version)
```
#### 6.2 控制 AGVPro以0.5m/s的速度向前移动3秒
```python
import time
from pymycobot import MyAGVPro

# 初始化 AGVPro 对象
agv_pro = MyAGVPro("/dev/ttyTHS1", baudrate=1000000, debug=True)

# 控制 agv_pro 以 0.5m/s 的速度向前移动
agv_pro.move_forward(0.5)

# 睡眠 3 秒
time.sleep(3)

# 停止移动
agv_pro.stop()
```