# Mercury X1 Chassis

---

- 使用前提
  - 先在打开一个控制台终端运行下面指令，启动底盘通信。
  
  ```bash
  roslaunch turn_on_mercury_robot mapping.launch
  ```

- API使用说明
  >> API (Application Programming Interface), 也称为应用程序编程接口函数，是预定义的函数。使用以下函数接口时，请在开始时输入以下代码导入我们的 API 库，否则将无法成功运行.

  ```python
  # 示例
  from pymycobot import mercurychassis_api

  mc = ChassisControl('/dev/wheeltec_controller')

  # 获取电池电压
  voltage = mc.get_power_voltage()
  print(voltage)

  # 设置底盘RGB灯带为绿色
  mc.set_color(255, 0, 0)
  ```

## get_power_voltage()

- 功能：获取电池电压。
- 返回值：电池电压（浮点型），单位：伏特

## get_ultrasonic_value()

- 功能：获取所有超声波数据
- 返回值：所有超声波数据列表，单位：毫米。

## go_straight(speed)

- 功能：控制底盘前进
- 参数：
  - speed：（浮点型）底盘运动的速度，范围是`0 ~ 1`，单位：米每秒。

## go_back(speed)

- 功能：控制底盘后退
- 参数：
  - speed：（浮点型）底盘运动的速度，范围是`-1 ~ 0`，单位：米每秒。

## turn_left(speed)

- 功能：控制底盘向左旋转
- 参数：
  - speed：（浮点型）底盘运动的速度，范围是`0 ~ 1`，单位：米每秒。

## turn_right(speed)

- 功能：控制底盘右旋转
- 参数：
  - speed：（浮点型）底盘运动的速度，范围是`-1 ~ 0`，单位：米每秒。

## stop()

- 功能：停止运动

## set_color(r, g, b)

- 功能：设置底盘RGB灯带颜色
- 参数：
  - r (int)：0 ~ 255
  - g (int)：0 ~ 255
  - b (int)：0 ~ 255

