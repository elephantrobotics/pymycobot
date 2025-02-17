from pymycobot import MyCobot280Socket

m = MyCobot280Socket("192.168.10.10", "9000")
# connect pi
# m.connect()
print(m.get_coords())
