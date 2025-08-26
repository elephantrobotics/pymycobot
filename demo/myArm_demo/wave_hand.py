import time
from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0', debug=False)
time.sleep(0.1)

mc.set_fresh_mode(0)
time.sleep(0.2)

mc.set_servo_data(7, 5, 8)
time.sleep(0.1)

mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
time.sleep(3)

def main():
    # three action
    mc.send_angles([-90, 0, 0, 0, 0, 0, 40], 60)
    time.sleep(2.5)

    for i in range(4):
        mc.send_angle(4, 45, 95)
        time.sleep(1.5)
        mc.send_angle(4, -45, 95)
        time.sleep(1.5)
        
    for i in range(4):
        mc.send_angle(4, 45, 20)
        time.sleep(2.5)
        mc.send_angle(4, -45, 20)
        time.sleep(2.5)
    mc.send_angles([-90, 0, 0, 0, 0, 0, 40], 50)
    time.sleep(2.5)

i = 0
while i <1:
    main()
    i+=1

