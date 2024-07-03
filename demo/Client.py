from pymycobot import MyCobotSocket

m = MyCobotSocket("localhost", 9000)
# connect pi
# m.connect()
print(m.wait(2))