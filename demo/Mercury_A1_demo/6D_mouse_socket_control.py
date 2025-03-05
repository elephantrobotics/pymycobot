import pygame
import time
from pymycobot import MercurySocket

# 初始化机械臂,IP和端口号需根据实际进行修改
mc = MercurySocket('192.168.1.4', 9001)

# 设置夹爪透传模式
mc.set_gripper_mode(0)

# 让机械臂回到初始点,便于坐标控制，速度为20
mc.send_angles([0, 0, 0, -90, 0, 90, 0], 20)

THRESHOLD = 0.7  # 鼠标轴触发运动的阈值
DEAD_ZONE = 0.2  # 归零判断阈值

# jog 坐标运动速度
jog_speed = 20

# 夹爪速度
gripper_speed = 70

# 夹爪状态（默认张开）
gripper_state = True

# 6D 轴映射关系，鼠标轴 -> 机械臂坐标ID映射
AXIS_MAPPING = {
    0: 2,
    1: 1,
    2: 3,
    3: 4,
    4: 5,
    5: 6
}

# 轴运动方向映射
DIRECTION_MAPPING = {
    0: (-1, 1),  # 轴0 (Y) -> 负向 -1，正向 1
    1: (-1, 1),  # 轴1 (X) -> 负向 -1，正向 1
    2: (-1, 1),  # 轴2 (Z) -> 负向 -1，正向 1
    3: (-1, 1),  # 轴3 (RX) -> 负向 -1，正向 1
    4: (1, -1),  # 轴4 (RY) -> 负向 1，正向 -1
    5: (1, -1)  # 轴5 (RZ) -> 负向 1，正向 -1
}


def handle_mouse_event(jog_callback, stop_callback, gripper_callback):
    """
    监听 6D 鼠标事件，并调用传入的回调函数控制机械臂
    :param jog_callback: 运动回调函数 (coord_id, direction)
    :param stop_callback: 停止运动回调函数 (coord_id)
    :param gripper_callback: 夹爪开合回调函数 ()
    """
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未检测到 6D 鼠标设备！")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"已连接 6D 鼠标: {joystick.get_name()}")

    active_axes = {}  # 记录当前运动状态

    try:
        while True:
            for event in pygame.event.get():
                # 处理轴运动
                if event.type == pygame.JOYAXISMOTION:
                    axis_id = event.axis
                    value = event.value

                    if axis_id in AXIS_MAPPING:
                        coord_id = AXIS_MAPPING[axis_id]
                        pos_dir, neg_dir = DIRECTION_MAPPING[axis_id]

                        if value > THRESHOLD and active_axes.get(axis_id) != 1:
                            jog_callback(coord_id, pos_dir)
                            active_axes[axis_id] = 1

                        elif value < -THRESHOLD and active_axes.get(axis_id) != -1:
                            jog_callback(coord_id, neg_dir)
                            active_axes[axis_id] = -1

                        elif -DEAD_ZONE < value < DEAD_ZONE and axis_id in active_axes:
                            stop_callback(coord_id)
                            del active_axes[axis_id]

                # 处理按键（夹爪开合）
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button in [0, 1]:  # 任意按键触发
                        gripper_callback()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("监听结束")
        pygame.quit()


# 定义回调函数
def jog_callback(coord_id, direction):
    """触发机械臂JOG坐标运动"""
    print(f"机械臂 {coord_id} 方向 {'正向' if direction == 1 else '负向'} 运动")
    if direction != 1:
        direction = 0
    mc.jog_coord(coord_id, direction, jog_speed)

def stop_callback(coord_id):
    """停止机械臂运动"""
    print(f"停止机械臂 {coord_id} 运动")
    mc.stop(1)

def gripper_callback():
    """控制夹爪开合"""
    global gripper_state
    gripper_state = not gripper_state
    flag = 1 if gripper_state else 0
    print(f"夹爪 {'闭合' if flag else '张开'}")
    mc.set_gripper_state(flag, gripper_speed)


if __name__ == '__main__':
    # 启动监听
    handle_mouse_event(jog_callback, stop_callback, gripper_callback)
