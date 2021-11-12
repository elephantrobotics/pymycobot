from pymycobot.mycobotsocket import MyCobotSocket
import time
m = MyCobotSocket("192.168.10.115", "/dev/ttyAMA0", "1000000")
# m.send_command("send_angles([0,0,0,0,0,0],40)")
# m.send_command("power_on")
# print(m.send_command("is_power_on"))
# m.send_command("release_all_servos")
# print(m.send_command("is_controller_connected"))
# m.send_command("send_angles([0,0,0,0,0,0],30)")
# m.send_command("send_angle(1,100,30)")
print(m.send_command("get_coords"))
# print(m.send_command("get_coords"))
# m.send_command("send_coords([59.2,-52.4,231.5,169.03,15.64,-91.2],20,1)")
# print(m.send_command(
#     "is_in_position([59.2,-52.4,231.5,169.03,15.64,-91.2],1)"))
# m.send_command("jog_coord(1,0,20)")
# print(m.send_command("release_servo(2)"))
# m.send_command("sync_send_angles([0,0,0,0,0,0],30)")
# m.send_command("sync_send_angles([100,0,0,0,0,0],30)")
# m.send_command(
#     "sync_send_coords([59.2,-52.4,231.5,169.03,15.64,-91.2],30,1)")
# m.send_command(
#     "sync_send_coords([53,53,412,-90.7,-1.5,12.67],30,1)")
# print(m.send_command("is_in_position([53,53,412,-90.7,-1.5,12.67],1)"))
