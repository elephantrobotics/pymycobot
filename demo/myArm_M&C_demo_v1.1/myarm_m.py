# myarm_m接收角度并执行
from pymycobot.error import MyArmDataException
from pymycobot import MyArmC, MyArmM
import serial.tools.list_ports
import time
import socket

def get_port(): # 获取所有串口号
    port_list = serial.tools.list_ports.comports()
    i = 1
    res = {}
    for port in port_list:
        print("{} - {}".format(i, port.device))
        res[str(i)] = port.device
        i += 1
    return res

def processing_data(data):
    data = data.split('\n')[-1]
    angle = list(data[1:-1].split(','))
    angle = [float(i) for i in angle]
    # angle[2] *= -1
    gripper_angle = angle.pop(-1)
    angle.append((gripper_angle - 0.08) / (-95.27 - 0.08) * (-123.13 + 1.23) - 1.23)
    return angle

def main():
    HOST = '127.0.0.1'
    PORT = 8001
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(5)
    port_dict = get_port()
    port_m = input("input myArm M port: ")
    m_port = port_dict[port_m]
    m = MyArmM(m_port, 1000000, debug=False)
    speed = 100

    print('Server start at: %s:%s' % (HOST, PORT))
    print('wait for connection...')
    # 接收客户端请求

    while True:
        conn, addr = server.accept()
        # 客户端IP
        print('Connected by ', addr)
        while True:
            try:
                data = conn.recv(1024).decode('utf-8')
                angle = processing_data(data)
                m.set_joints_angle(angle, speed)
            except MyArmDataException:
                pass
            

if __name__ == "__main__":
    main()
