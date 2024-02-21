# 水星Python API

## 系统与产品信息

### get_system_version()

获取机器人主控固件版本号

* **返回**
  浮点型数据

### get_atom_version()

获取末端固件版本号

* **返回**
  浮点型数据

## 机器人整体运行状态

### power_on()

控制机器人上电

### power_off()

控制机器人下电

### is_power_off()

查询机器人是否上电

* **返回**
  `0`: 未上电
  `1`: 已上电

### release_all_servos()

机器人关闭力矩输出

### get_angle(joint_id)

获取单关节角度

* **参数**
  `joint_id`: 范围 1 ~ 7

* **返回**
  浮点型数据

### focus_all_servos()

机器人打开力矩输出

## 运行与操作

### get_angles()

获取机器人全关节角度信息

* **返回**
  长度为7的列表



### get_angle(joint_id)
获取机器人单个关节的角度信息

* **参数**

    **joint_id** – 关节ID号。

* **返回**
  指定关节的角度信息

### send_angles(angles, speed)
控制机器人全角度运动

* **参数**

    **angles** – 长度为6的列表。
    **speed** – 机器人运动的速度。

### get_coords()
获取机器人的坐标位置

* **返回**
  长度为6的列表，[x, y, z, rx, ry, rz]

### send_coords(coords, speed)
控制机器人坐标运动

* **参数**

    **coords** – 长度为6的列表。
    **speed** – 机器人运动的速度。
<!-- 
#### 未完待续...

## 案例

```python
from pymycobot import Mercury
import time
mc = Mercury("/dev/ttyAMA1", 115200)

print(mc.get_angles())

mc.send_angles([0, 0, 0, 0, 90, 0, 0], 40)
time.sleep(5)
mc.send_coords([100, 0, 300, 0, 0, 0], 40)
``` -->