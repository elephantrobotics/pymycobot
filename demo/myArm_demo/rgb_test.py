import time
import math
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0', debug=False)
time.sleep(0.1)

# mc.send_angle(1, 0, 80)
# print(mc.get_angles(), mc.get_coords())
mc.send_angles([0, 0, 0, 0, 0, 0, 0], 50)
time.sleep(2.5)


# mc.set_color(255, 0, 0)
# time.sleep(1)
# mc.set_color(128, 0, 0)
# time.sleep(1)
# mc.set_color(32, 0, 0)
# time.sleep(1)

def breathing_led(mc, duration):
    min_brightness = 0
    max_brightness = 255
    speed = 0.02

    period = 2 * math.pi

    while True:
        start_time = time.time()
        while time.time() - start_time < duration:
            elapsed_time = time.time() - start_time
            phase = (elapsed_time * 2 * math.pi / period) % (2 * math.pi)

            brightness = int(1 - (math.cos(phase)) / 2 * (max_brightness - min_brightness) + min_brightness)
            brightness = max(min(brightness, max_brightness), min_brightness)
            print('color:', brightness)
            mc.set_color(brightness, 0, 0)
            time.sleep(speed)

        mc.set_color(0, 0, 0)


duration = 10

breathing_led(mc, duration)
