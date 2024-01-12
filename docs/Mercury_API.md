### power_on()
控制机器人上电

### power_off()
控制机器人下电

### get_angles()
获取机器人全关节角度信息

* **返回**
  长度为6的列表

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