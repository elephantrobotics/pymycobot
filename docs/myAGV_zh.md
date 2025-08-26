# myAgv

<details>
<summary>API目录:</summary>

- [myAgv](#myagv)
  - [set\_led(mode, R, G, B)](#set_ledmode-r-g-b)
  - [get\_firmware\_version](#get_firmware_version)
  - [get\_motors\_current](#get_motors_current)
  - [get\_battery\_info](#get_battery_info)
  - [go\_ahead(go\_speed)](#go_aheadgo_speed)
  - [retreat(back\_speed)](#retreatback_speed)
  - [pan\_left(pan\_left\_speed)](#pan_leftpan_left_speed)
  - [pan\_right(pan\_right\_speed)](#pan_rightpan_right_speed)
  - [clockwise\_rotation(rotate\_right\_speed)](#clockwise_rotationrotate_right_speed)
  - [counterclockwise\_rotation(rotate\_left\_speed)](#counterclockwise_rotationrotate_left_speed)
  - [stop()](#stop)
  - [restore()](#restore)

</details>

## set_led(mode, R, G, B)

设置led灯颜色

- **参数**

  - **mode** – 1 - 常亮. 2 - 闪烁.

  - **R** – (int) 0 ~ 255

  - **G** – (int) 0 ~ 255

  - **B** – (int) 0 ~ 255

## get_firmware_version

获取固件版本号

## get_motors_current

获取电机电流信息

## get_battery_info

获取电池信息

- **Return**
  list : [battery_data, battery_1_voltage, battery_2_voltage].
  `battery_data`: 长度为6的字符串, 从左到右依次对应: bit5, bit4, bit3, bit2, bit1, bit0.
  bit5 : 电池2插入接口，1表示插入，0未插入.
  bit4 : 电池1插入接口，1表示插入，0未插入.
  bit3 : 适配器插入接口，1表示插入，0未插入.
  bit2 : 充电桩插入接口，1表示插入，0未插入.
  bit1 : 电池2充电灯，0表示没亮，1表示亮.
  bit0 : 电池1充电灯，0表示没亮，1表示亮.
  `battery_1_voltage` : 电池1电压.
  `battery_2_voltage` : 电池2电压.

<!-- ## move_control(direction_1, direction_2, direction_3)

运动控制接口，控制小车前进、后退、平移、旋转.

- **参数**

  - **direction_1** – (int) 控制前进或者后退: 0 ~ 127 表示后退, 129 ~ 255 表示前进, 128 表示停止.

  - **direction_2** – (int) 控制左右平移: 0 ~ 127 表示向右平移, 129 ~ 255 表示向左平移, 128 表示停止.

  - **direction_3** – (int) 控制旋转: 0 ~ 127 表示顺时针旋转, 129 ~ 255 表示逆时针旋转, 128 表示停止. -->

## go_ahead(go_speed)

控制前进

- **参数**

  - **go_speed** – (int) 1 ~ 127 数值越小，速度越慢

## retreat(back_speed)

控制后退

- **参数**

  - **back_speed** – (int) 1 ~ 127 数值越小，速度越慢

## pan_left(pan_left_speed)

控制左平移

- **参数**

  - **pan_left_speed** – (int) 1 ~ 127 数值越小，速度越慢

## pan_right(pan_right_speed)

控制右平移

- **参数**

  - **pan_right_speed** – (int) 1 ~ 127 数值越小，速度越慢

## clockwise_rotation(rotate_right_speed)

控制顺时针旋转

- **参数**

  - **rotate_right_speed** – (int) 1 ~ 127 数值越小，速度越慢

## counterclockwise_rotation(rotate_left_speed)

控制逆时针旋转

- **参数**

  - **rotate_left_speed** – (int) 1 ~ 127 数值越小，速度越慢

## stop()

停止运动

## restore()

堵转恢复
