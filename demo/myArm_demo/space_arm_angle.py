import time

from pymycobot.myarm import MyArm

mc = MyArm('/dev/ttyAMA0', debug=False)

mc.set_fresh_mode(0)
time.sleep(0.5)

mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
time.sleep(2.5)

mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, -76.2, 0.0], 60)

# mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)

time.sleep(2.5)

for i in range(4):
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)
    

    
mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)

time.sleep(2.5)

for i in range(4):
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)
    
mc.send_angles([-58.88, 34.27, -0.43, -67.5, -0.35, 20.2, 0.0], 60)

mc.send_angles([40.25, 36.38, -0.52, -67.23, 0.26, -76.37, -0.35], 60)
time.sleep(2.5)

for i in range(4):
    mc.set_solution_angles(70, 20)
    time.sleep(2.5)
    mc.set_solution_angles(-70, 20)
    time.sleep(2.5)
    
mc.send_angles([40.25, 36.38, -0.52, -67.23, 0.26, -76.37, -0.35], 60)
time.sleep(2.5)

mc.send_angles([0, 0, 0, 0, 0, 0, 0], 60)
time.sleep(2.5)
