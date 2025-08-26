# API说明

API(Application Programming Interface)，即应用程序接口函数，是一些预先定义好的函数。在使用下列函数接口的时候请先在开头导入我们的API库，即输入下方代码，否则无法运行成功：

```python
# 适用于mybuddy
from pymycobot.mybuddy import MyBuddy
```

> **注意：**个别函数接口有返回值，但是直接输入代码，返回的结果是没有返回值的，需要使用`print`函数把结果打印出来。比如，想要获取机械臂当前设置的速度值可使用`get_speed()`，但是直接输入该函数是没有结果的，正确写法是：`print(get_speed())`即可把速度值打印出来。下面对API说明部分如果标注**无返回值**，则不需要使用`print`函数，反之则需使用`print`函数打印结果。

## myBuddy

### 1 机械臂整体运行状态

**1.1** ` power_off(id=0)`

- **功能：** 机械臂关闭电源

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

**1.2** ` power_on(id=0)`

- **功能：** 机械臂打开电源

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

**1.3** ` read_next_error(id=0)`

- **功能：** 机器人错误检测

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

**1.4** ` release_all_servos(id=0)`

- **功能：** 放松舵机

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

**1.5** ` is_power_on(id=0)`

- **功能：** 控制核心连接状态查询

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

- **返回**

    1 - 开机
    0 - 关机
    -1 - 错误数据

**1.6** ` is_controller_connected(id=0)`

- **功能：** 是否与 Atom 连接。

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

- **返回**

    0 - 断开
    1 - 连通


**1.7** ` set_free_mode(id, value)`

- **功能：** 设置自由模式

- **参数**

  - **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

  - **value** - 0 - 关闭 1 - 打开

**1.8** ` set_fresh_mode(id, mode)`

- **功能：** 设置命令刷新模式

- **参数**

  - **id** – 1/2(左/右)。

  - **mode** - int
    1 - 总是先执行最新的命令。
    0 - 以队列的形式顺序执行指令。

**1.9** `release_servo(id，servo_id)`
- **功能：** 放松指定的单个舵机

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_id** - 1 - 6。


**1.10** ` is_free_mode(id)`

- **功能：** 检查是否为自由模式

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

- **返回**

    0 - 否
    1 - 是


### 2 机械臂运行状态和设置


**2.1** ` stop(id)`

- **功能：** 停止移动

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)。



**2.2** ` resume(id)`

- **功能：** 恢复运动

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)。


**2.3** ` is_paused(id)`

- **功能：** 判断机械手是否暂停。

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)。

- **返回**

    1 - 暂停
    0 - 未暂停
    -1 - 错误

**2.4** ` get_speed(id)`

- **功能：** 获得速度

- **参数：**

    **id** – 1/2/3(左臂/右臂/腰部)。

- **返回**

    速度

- **返回类型**

    整数


**2.5** ` set_speed(id，speed)`

- **功能：** 设定速度值

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **speed** (_int_) - 0 - 100

**2.6** ` get_joint_min_angle(id，joint_id)`

- **功能：** 获取指定关节的最小移动角度

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint_id** - (int) 1 - 6

- **返回**

    角度值(浮点数)


**2.7** ` is_servo_enable(id，servo_id)`

- **功能：** 判断所有舵机是否连接

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_id** - (int) 1 ~ 6

- **返回**

    0 - 不连通
    1 - 已连通
    -1 - 错误


**2.8** ` is_all_servo_enable(id)`

- **功能：** 判断是否连接了指定舵机

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)

- **返回**

    0 - 禁用
    1 - 启用
    -1 - 错误



**2.9** ` set_joint_min(id，joint_id，angle)`

- **功能：** 设置关节最小角度

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint_id** - 整数 1-6。

  - **角度** - 0 ~ 180


**2.10** ` get_system_version(id)`

- **功能：** 获取软件版本

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

**2.11** ` get_joint_max_angle(id，joint_id`

- **功能:** 获取指定关节的最大运动角度

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint_id** - (int) 1 - 6

- **返回**

    角度值(浮点数)


**2.12** ` joint_brake(id, joint_id)`

- **功能：** 关节运动时使其停止，缓冲距离与现有速度正相关

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint_id** - 1 - 6


### 输入程序控制模式(MDI模式)


**3.1** ` get_angles(id)`

- **功能：** 获取所有关节的度数。

- **参数**

    **id** – 1/2(左/右)

- **返回**

    长度为6的列表。

- **返回类型**

    列表

**3.2** ` send_angle(id, joint, angle, speed)`

- **功能：** 向机械臂发送单关节角度。

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint** – 1 ~ 6

  - **angle** - 角度值

  - **speed** – 1 ~ 100

- **返回**
  - 无

**3.3** ` send_angles(id，degrees，speed)`

- **功能：** 将所有角度发送到指定的机械臂

- **参数**

  - **id** – 1/2(左/右)。

  - **degrees** - [angle_list] len 6

  - **speed** - 1 - 100


**3.4** ` set_joint_max(id，joint_id，angle)`

- **功能：** 设置关节最大角度

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **joint_id** - 整数 1-6。

  - **角度** - 0 ~ 180




**3.5** ` send_coord(id, coord, data, speed)`

- **功能：** 向机械臂发送单个坐标

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **coord** – 1 ~ 6 (x/y/z/rx/ry/rz)

  - **data** - 坐标值

  - **speed** - 0 ~ 100

**3.6** ` send_coords(id, coords, speed, mode)`

- **功能：** 将所有坐标发送到机械臂。

- **参数**

  - **id** – 1/2(左/右)。

  - **coords** – 坐标值列表(List[float])，长度 6，[x(mm), y, z, rx(angle), ry, rz]

  - **speed** - (int) 1 ~ 100

  - **mode** - (int) 0 - moveJ, 1 - moveL, 2 - moveC


**3.7** ` get_coord(id，joint_id)`

- **功能：** 读取单个坐标参数

- **参数**

  - **id** (_int_) – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** (_int_) – 1 - 7(7 是夹爪)



**3.8** ` get_encoder(id,joint_id)`

- **功能：** 获取指定的关节电位值。

- **参数**

  - **id** - 1/2/3(左臂/右臂/腰部)。

  - **joint_id** - (int) 1 ~ 6

- **返回**

    0 ~ 4096

**3.9** ` get_encoders(id)`

- **功能：** 获取指定机械臂的六个关节电位值

- **参数**

    **id** – 1/2(左/右)。

- **返回**

    列表


**3.10** ` get_radians(id)`

- **功能：** 获取所有关节的弧度

- **参数**

    **id** – 1/2(左/右)

- **返回**

    浮动弧度列表 [radian1, ...]

- **返回类型**

    列表

**3.11** ` send_radians(id, radians, speed)`

- **功能：** 将所有关节的弧度发送到机械臂

- **参数**

  - **id** – 1/2(左/右)。

  - **radians** – 弧度值列表(List[float])，长度为 6

  - **speed** - (int)0 ~ 100


**3.12** ` is_in_position(id, data, mode)`

- **功能：** 判断是否到达位置。

- **参数**

  - **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)。

  - **data** – 数据列表、角度或坐标。如果 id 是 1/2。数据长度为 6. 如果 id 为 0. data len 13(data==[ [left_angles / left_coords],[right_angles / right_coords],[waist_angle /waist_coord]]). 如果 id 为 3. data len 1

  - **mode** - 1 - 坐标，0 - 角度

- **返回**

    1 - 已到达
    0 - 未到达
    -1 - 错误

**3.13** ` is_moving(id)`

- **功能：** 检测机器人是否在移动

- **参数**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)。

- **返回**

    0 - 不动
    1 - 正在移动
    -1 - 错误数据


**3.14** ` set_color(id, r=0, g=0, b=0)`

- **功能：** 设置机器人手臂顶部Atom的灯光颜色。

- **参数**

  - **id** - 1/2(左/右)

  - **r** (_int_) – 0 ~ 255

  - **g** (_int_) – 0 ~ 255

  - **b** (_int_) – 0 ~ 255


**3.15** ` set_encoder(id，joint_id，encoder，speed)`

- **功能：** 发送单关节的encoder值

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** - 1 - 6。

  - **encoder** – 设置编码器的值。

**3.16** ` set_encoders(id，encoder，speed)`

- **功能：** 设置指定机械臂的六个关节同步执行到指定encoder位置。

- **参数**

  - **id** – 1/2(左/右)。

  - **encoders** - 编码器列表，长度为 6。

  - **speed** – 速度 1 ~ 100


**3.17** ` get_angle(id，joint_id)`

- **功能：** 获取单个关节的角度

- **参数**

  - **id** (_int_) – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** (_int_) – 1 - 7(7 是夹爪)

**3.18** ` set_servo_calibration(id，servo_no)`

- **功能：** 设置关节当前位置为角度零点，

    对应的encoder为2048。

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_no** – 舵机的序列号，1 - 6。

**3.19** ` set_joint_current(id，joint_id，current)`

- **功能：** 设置碰撞电流

- **参数:**

  - **id** - 0/1/2 (ALL/L/R)

  - **joint_id** - 1 - 6

  - **current** – 电流值


**3.19** ` set_encoders_drag(id, encoders, speeds)`

- **功能：** 设置末端坐标系

- **参数:**

  - **id** - 0/1/2 (ALL/L/R)

  - **encoders** - `list` 每个关节的电位值

  - **speed** - 通过`get_servo_speeds()`获得


**3.20** ` get_coords(id)`

- **功能：** 获取机械臂坐标

- **参数**

    **id** – 1/2(左/右)。

### 4 JOG模式和操作

**4.1** ` jog_absolute(id，joint_id，angle，speed)`

- **功能：** 绝对关节控制

- **参数:**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** - 整数 1-6。

  - **angle** - int

  - **speed** - int (0 - 100)

**4.2** ` jog_angle(id，joint_id，direction，speed)`

- **功能：** 关节控制。

- **参数:**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** - 整数 1-6。

  - **direction** - 0 - 减少，1 - 增加

  - **speed** - int (0 - 100)

**4.3** ` jog_coord(id, coord_id, direction, speed)`

- **功能：** 坐标控制。

- **参数:**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **coord_id** – int 1-6 (x/y/z/rx/ry/rz)。

  - **direction** - 0 - 减少，1 - 增加

  - **speed** - int (0 - 100)

**4.4** ` jog_inc_coord(axis，increment，speed)`

- **功能：** 双臂协同坐标步进

- **参数:**

  - **axis** – 1 - 6 (x/y/z/rx/ry/rz)

  - **increment** -

  - **speed** - 1 - 100

**4.5** ` jog_increment(id，joint_id，increment，speed)`

- **功能：** 步进模式

- **参数:**

  - **id** – 1/2/3 (左臂/右臂/腰部)。

  - **joint_id** - 整数 1-6。

  - **increment** -

  - **speed** - int (1 - 100)

**4.6** ` jog_stop(id)`

- **功能：** JOG停止

- **参数:**

    **id** – 1/2/3(左臂/右臂/腰部)。


### 5 舵机控制与操作

**5.1** ` focus_servo(id，servo_id)`

- **功能：** 上电指定舵机

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_id** - 1 - 6


**5.2** ` get_servo_currents(id)`

- **功能：** 获取关节电流

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)

- **返回**

    毫安单位的值

<!-- **5.3** ` get_serv**5.1o_data(id，servo_no，data_id)`

- **功能：** 读取舵机指定地址的数据参数。

- **参数：**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_no** – 舵机的序列号，1 - 6。

  - **data_id** – 数据地址。

- **返回**

    值 (0 - 4096)
    0 - 禁用
    1 - 启用
    -1 - 错误 -->

**5.3** ` get_servo_status(id)`

- **功能：** 获取关节状态

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)

- **返回**

    【电压、传感器、温度、电流、角度、过载】，值为0表示无错误

**5.4** ` get_servo_temps(id)`

- **功能：** 获取关节温度

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)

**5.5** ` get_servo_voltages(id)`

- **功能：** 获取关节电压

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)

- **返回**

    电压值 < 24 V


<!-- **5.7** ` set_servo_data(id，servo_no，data_id，value)`

- **功能：** 设置舵机指定地址的数据参数

- **参数**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **servo_no** – 舵机的序列号，1 - 6。

  - **data_id** – 数据地址。

  - **value** - 0 - 4096 -->



### 6 Atom 末端IO控制


**6.1** ` set_pin_mode(id, pin_no, pin_mode)`

- **功能：** 在 atom 中设置指定引脚的状态模式。

- **参数：**

  - **id** - 1/2(左/右)

  - **pin_no** (_int_) – 引脚编号 (1 - 5)。

  - **pin_mode** (_int_) - 0 - 输入，1 - 输出


**6.2** ` set_digital_output(id, pin_no, pin_signal)`

- **功能：** 设置 atom IO 输出电平

- **参数：**

  - **id** - 1/2(左/右)

  - **pin_no** (_int_) - 1 - 5

  - **pin_signal** (_int_) – 0 / 1


**6.3** ` get_digital_input(id, pin_no)`

- **功能：** 读取Atom IO输入电平

- **参数：**

  - **id** - 1/2(左/右)

  - **pin_no** (_int_) - 1 - 5


**6.4** ` set_pwm_output(id, channel, frequency, **6.1pin_val)`

- **功能：** 脉宽调制控制

- **参数：**

  - **id** - 1/2(左/右)

  - **channel** (_int_) – IO 编号 (1 - 5)。

  - **frequency** (_int_) – 时钟频率 (0/1: 0 - 1Mh**6.1z 1 - 10Mhz`)

  - **pin_val** (_int_) – 占空比 0 ~ 100: 0 ~ 100%

### 7 夹爪控制


**7.1** ` get_gripper_value(id)`

- **功能：** 获取夹爪的值。

- **参数**

    **id** – 1/2(左/右)

- **返回**

    夹持器值 (int)


**7.2** ` is_gripper_moving(id)`

- **功能：** 判断夹爪是否在移动

- **参数**

    **id** – 1/2(左/右)

- **返回**

    0 - 不动
    1 - 正在移动
    -1 - 错误数据


**7.3** ` set_gripper_state(id, flag)`

- **功能：** 设置夹爪开关状态

- **参数**

  - **id** - 1/2(左/右)

  - **flag** (_int_) - 0 - 关闭，1 - 打开

**7.4** ` set_gripper_value(id, value, speed)`

- **功能：** 设置夹爪的值

* **参数**

  * **id** – 1/2 (L/R)

  * **value** (_int_) – 0 ~ 100

  * **speed** (_int_) – 0 ~ 100


### 8 socket通信

```python
from pymycobot import MyBuddySocket

mst = MyBuddySocket("192.168.0.1", 9000)
mst.connect("/dev/ttyACM0", "115200")

print(mst.get_angles(1))
```

### 9 树莓派——GPIO

**9.1** ` Get_gpio_input(pin)`

- **功能：** 获取 GPIO 输入值。

- **参数**

    **pin** - (int)pin 号。

**9.2** ` set_gpio_input(pin)`

- **功能：** 设置 GPIO 输入值。

- **参数**

    **pin** - (int)pin 号。 

**9.3** ` set_gpio_mode(pin_no, mode)`

- **功能：** 初始化 GPIO 模块，设置 BCM 模式。

- **参数**

  - **pin_no** - (int) 引脚号。

  - **mode** - 0 - 输入 1 - 输出

**9.4** ` set_gpio_output(pin, v)`

- **功能：** 设置 GPIO 输出值。

- **参数**

   - **pin** - (int)pin 号。

   - **v** - (int) 0 / 1

**9.5** ` set_gpio_pwm(pin, baud, dc)`

- **功能：** 设置 GPIO PWM 值。

- **参数**

   - **pin** - (int)pin 号。

   - **baud** - (int) 10 - 1000000

   - **dc** - (int) 0 - 100


### 10 坐标变换


**10.1** ` set_tool_reference(id，coords)`

- **功能：** 设置工具坐标系

- **参数**

  - **id** - 0/1/2 (ALL/L/R)

  - **coords** – 坐标值列表(List[float])，长度为 6。[x(mm), y, z, rx(angle), ry, rz]

**10.2** ` set_world_reference(id，coords)`

- **功能：** 设置世界坐标系

- **参数**

  - **id** - 0/1/2 (ALL/L/R)

  - **coords** – 坐标值列表(List[float])，长度 6 [x(mm), y, z, rx(angle), ry, rz]


**10.3** ` get_reference_frame(id)`

- **功能：** 获取base坐标系

- **参数**

    **id** – 0/1/2 (ALL/L/R)

- **返回**

    0 - 基础 1 - 工具。


**10.4** ` get_tool_reference(id)`

- **功能：** 获取工具坐标系

- **参数**

    **id** – 0/1/2 (ALL/L/R)

**10.5** ` get_world_reference(id)`

- **功能：** 获取世界坐标系

- **参数**

    **id** – 0/1/2 (ALL/L/R)


**10.6** ` set_reference_frame(id, rftype)`

- **功能：** 设置基坐标系

- **参数**

  - **id** - 0/1/2 (ALL/L/R)

  - **rftype** - 0 - base 1 - tool。



**10.7** ` set_movement_type(id, move_type)`

- **功能：** 设置移动类型

- **参数**

  - **id** - 0/1/2 (ALL/L/R)

  - **move_type** - 1 - movel，0 - moveJ


**10.8** ` get_movement_type(id)`

- **功能：** 获取运动类型

- **参数**

    **id** – 0/1/2 (ALL/L/R)

- **返回**

    1 - 移动L
    0 - 移动J



**10.9** ` set_end_type(id, end)`

- **功能：** 设置末端坐标系

- **参数**

  - **id** - 0/1/2 (ALL/L/R)

  - **end** - 0 - 法兰，1 - 工具



**10.10** ` get_end_type(id)`

- **功能：** 获取末端坐标系

- **参数**

    **id** – 0/1/2 (ALL/L/R)

- **返回**

    0 - 法兰
    1 - 工具


**10.11** ` write_base_coords(id, coords, speed)`

- **功能：** Base坐标移动

- **参数**

  - **id** - 1/2(左/右)

  - **coords** - coords：坐标值列表(List[float])，长度 6，[x(mm), y, z, rx(angle), ry, rz]

  - **speed** - 1 - 100



**10.12** ` write_base_coord(id，axis, coord, speed)`

- **功能：**Base单坐标移动

- **参数**

  - **id** - 1/2(左/右)

  - **axis** – 1 - 6 (x/y/z/rx/ry/rz)

  - **coord** - 坐标值

  - **speed** - 1 - 100 


**10.13** ` base_to_single_coords(base_coords, arm)`

- **功能：** 将base坐标转换为坐标

- **参数：**

  - **coords** – 基坐标值 len 6 的列表

  - **arm** - 0 - 左。 1 - 对

- **返回:**

    坐标



**10.14** ` get_base_coord(id)`

- **功能：** 获取单臂的基坐标

- **参数：**

    **id** – 1/2(左/右)

**10.15** ` (\*args)get_base_coords`

- **功能：** 将坐标转换为base坐标。可以传入参数或不传参数，传入参数时，是以参数为准进行转化；无参数则以当前所在坐标进行转化。

- **参数：**

  - **coords** – 坐标值列表(List[float])，长度 6 [x(mm), y, z, rx(angle), ry, rz]

  - **arm** - 0 - 左。 1 - 

- **返回:**

    基坐标


### 11 速度规划

**11.1** ` get_plan_acceleration(id=0)`

- **功能：** 获得规划加速度

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

- **返回**

    加速度

**11.2** ` get_plan_speed(id=0)`

- **功能：** 获得规划速度

- **参数：**

    **id** – 0/1/2/3 (ALL/左臂/右臂/腰部)

- **返回**

    规划速度。


**11.3** ` set_acceleration(id, acc)`

- **功能：**设置所有移动过程中加速度

- **参数:**

  - **id** – 1/2/3 (左臂/右臂/腰部)

  - **acc** - 1 - 100


**11.4** ` get_acceleration(id)`

- **功能：**读取所有移动过程中加速度

- **参数：**

    **id** – 1/2/3 (左臂/右臂/腰部)



### 12 碰撞检测

**12.1** ` get_joint_current(id，joint_id)`

- **功能:** 获取碰撞电流

- **参数:**

  - **id** - 0/1/2 (ALL/L/R)

  - **joint_id** - 1 - 6


**12.2** ` collision_switch(state)`

- **功能:** 碰撞检测开关

- **参数:**

    **state** (_int_) - 0 - 关闭 1 - 打开(默认关闭)

**12.3** ` collision(left_angles，right_angles)`

- **功能:** 碰撞检测主程序

- **参数:**

  - **left_angles** - 左臂角度，长度为6的列表。

  - **right_angles** – 右臂角度，长度为6的列表。

- **返回**

    整数

**12.4** ` is_collision_on()`

- **参数：** 获取碰撞检测状态
- **返回：**

    0 - 禁用
    1 - 启用**