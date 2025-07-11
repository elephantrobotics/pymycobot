from pymycobot import ElephantRobot
import time
robot_ip="192.168.6.79"
er=ElephantRobot(robot_ip,5001)
er.start_client()

if er.state_check()==False:
    er.start_robot()
    time.sleep(2)
    er._state_on()

er.write_angles([0,-90,0,-90,0,0],1999)
time.sleep(4)
er.write_angles([0,-90,90,-90,0,0],1999)
time.sleep(4)

er.set_digital_out(0,1)
time.sleep(2)
er.set_digital_out(0,0)
