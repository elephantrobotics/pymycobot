from pymycobot import Pro630Socket
# from ..pymycobot.pro630socket import Pro630Socket

m = Pro630Socket("localhost", 9000)
# print(m.get_pos_switch())
# print(m.wait(2))
print(m.power_on())