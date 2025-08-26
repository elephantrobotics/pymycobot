import time
import math
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0', debug=False)
time.sleep(0.1)

mc.send_angles([90, 0, 0, 0, 0, 0, 0], 50)
time.sleep(3)


# mc.set_color(255, 0, 0)
# time.sleep(1)
# mc.set_color(128, 0, 0)
# time.sleep(1)
# mc.set_color(32, 0, 0)
# time.sleep(1)

def breathing_led(mc, duration):
    min_brightness = 0
    max_brightness = 255
    speed = 0.01

    period = 2 * math.pi

    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]

    while True:
        for color in colors:
            start_time = time.time()
            while time.time() - start_time < duration:
                elapsed_time = time.time() - start_time
                phase = (elapsed_time * 2 * math.pi / period) % (2 * math.pi)

                brightness = int(1 - (math.cos(phase)) / 2 * (max_brightness - min_brightness) + min_brightness)
                brightness = max(min(brightness, max_brightness), min_brightness)
                print('color:', brightness)
                # mc.set_color(brightness, 0, 0)
                r = color[0] * brightness // max_brightness
                g = color[1] * brightness // max_brightness
                b = color[2] * brightness // max_brightness
                mc.set_color(r, g, b)
                time.sleep(speed)

            # mc.set_color(0, 0, 0)


duration = 6

breathing_led(mc, duration)
