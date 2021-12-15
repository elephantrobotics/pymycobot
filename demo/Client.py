from pymycobot import MyCobotSocket

m = MyCobotSocket("192.168.10.10", "9000")
# connect pi
# m.connect()
print(m.get_coords())
