from pymycobot import MyCobotSocket

mc = MyCobotSocket("192.168.10.10", "9000")

print(mc.get_angles())
# mc.power_on()
# mc.power_off()
# mc.sync_send_angles([-0.08, 71.27, -116.27, 19.77, 7.55, -1.23], 20)
# mc.sync_send_angles([-1.05, -7.29, 1.4, -49.39, 7.55, -1.23], 20)
# print(mc.is_controller_connected())
# print(mc.release_all_servos())
# print(mc.get_angles())
# mc.send_angle(1, 100, 20)
# print(mc.get_coords())
# mc.send_coords([180.4, -55.1, 210.1, -172.81, 7.32, -86.97], 20, 1)
# print(mc.jog_angle(1, 1, 20))
