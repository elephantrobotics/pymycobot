from pymycobot import MyArmC, MyArmM
import serial.tools.list_ports

def get_port():
    port_list = serial.tools.list_ports.comports()
    i = 1
    res = {}
    for port in port_list:
        print("{} - {}".format(i, port.device))
        res[str(i)] = port.device
        i+=1
    return res
    
port_dict = get_port()
read_angle = input("Re check ports (y/n): ")
if read_angle == "y":
    port_dict = get_port()
port_c = input("input myArm C port: ")
port_m = input("input myArm M port: ")
c_port = port_dict[port_c]
m_port = port_dict[port_m]

c = MyArmC(c_port,debug=False)
m = MyArmM(m_port, 1000000, debug=False)
while True:
    data = c.get_servos_encoder_drag()
    data[0][3] = 4096 - data[0][3]
    if data[0][-1] < 2000:
        data[0][-1] = int(data[0][-1] / 1.2)
    m.set_servos_encoder_drag(data[0], data[1])