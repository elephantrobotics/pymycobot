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

    #mc.send_angles([-57.65, 35.24, -0.17, -68.11, 0.0, 14.76, 135.0], 50)
    #print(mc.get_error_information())
    #exit()
    mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)
    time.sleep(2.5)

    for i in range(4):
        mc.set_solution_angles(70, 20)
      
        time.sleep(2.5)
        mc.set_solution_angles(-70, 20)
        time.sleep(2.5)
     

    mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)
    time.sleep(2.5)

    time.sleep(5)

    # two action
    mc.send_angles([-90, 0, 0.0, -90, 0, 0, -30], 60)
    time.sleep(2.5)

    for i in range(4):
        mc.send_angle(7, -65, 50)
        time.sleep(2)
        mc.set_encoder(8, 2450)
        time.sleep(2)
        mc.send_angle(7, 35, 50)
        time.sleep(2)
        mc.set_encoder(8, 2000)
        time.sleep(2)
      
    mc.send_angles([-90, 0.87, -0.08, -90.79, 0.35, 0.26, -30], 50)
    time.sleep(2.5)  

    time.sleep(5)
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

