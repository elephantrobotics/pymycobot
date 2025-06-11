# -*- coding: utf-8 -*-
import _thread
import socket
import json
import time
import sys
import math
import copy
from pymycobot import ElephantRobot




def connectRvsServer(ip, port=2013):
    sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        sock.connect((ip,port))
        print("ip:%s connected!" %(ip))
        return (True,sock)
    except Exception as e:
        sock.close()
        print("ip:%s disconnected!" %(ip))
        return (False,0)

def disconnect (sock):
    if(sock):
        sock.close ()
        sock=None
    else:
        sock=None


def get_send_TCP(sock_rvs):
    #TCP
    #get tcp
    time.sleep(2)
    result = erobot.get_coords()
    #result = result[0:6] 
    robot_TCP = result.copy()
    print("***get tcp***%s"%time.asctime())
    print(result)
   
    resCutStr = ' '.join(repr(e) for e in result)    
    print(resCutStr) 
    #send tcp
    sendStr = "ROBOT_TCP " + resCutStr + '#'
    print("***send tcp***%s"%time.asctime())
    print(sendStr) 
    sock_rvs.send(bytes(sendStr,encoding="utf-8"))
    print("***tcp_receive data***%s"%time.asctime())
    data=sock_rvs.recv(socket_buf_len)
    print(data)
    if data == 'ROBOT_TCP'.encode('utf-8'):
        print("***receive ROBOT_TCP***\n")
        return True,robot_TCP;
    else:
        print(str(sys._getframe().f_lineno) + "error\n")
        print("tcp_receive FAIL\n")
        time.sleep(0)
        return False;

def wait_done():
    time.sleep(2)
    while erobot.command_wait_done()!="0":   
        time.sleep(2)

if __name__ == "__main__":

    robot_ip="192.168.6.79"
    erobot = ElephantRobot(robot_ip,5001) 
    erobot.start_client()
    rvs_ip = "localhost"
    rvs_port = 2013
    conSuc_rvs, sock_rvs=connectRvsServer(rvs_ip, rvs_port)#连接RVS

    cal_joint=[-10.01, -112.825, 104.148, -82.116, -89.213, -28.698]
    J1 = cal_joint[0]
    J2 = cal_joint[1]
    J3 = cal_joint[2]
    J4 = cal_joint[3]
    J5 = cal_joint[4]
    J6 = cal_joint[5]
    
    point = []
    # home位 p0
    point.append ([J1, J2, J3, J4, J5, J6])

        # p1
    point.append ([J1, J2, J3, J4+2, J5-6, J6+2])
        # p2
    point.append ([J1-2, J2+4, J3-2, J4+4, J5-4, J6+2])
        # p3
    point.append ([J1-4, J2+2, J3-4, J4+4, J5-4, J6+2])
        # p4
    point.append ([J1+4, J2+4, J3-4, J4-4, J5-4, J6+2])
        # p5
    point.append ([J1+4, J2+2, J3-4, J4+4, J5, J6+2])
        # p6
    point.append ([J1+4, J2, J3-2, J4-2, J5, J6+2])
        # p7
    point.append ([J1+2, J2+6, J3-4, J4-4, J5, J6+2])
        # p8
    point.append ([J1+2, J2, J3-2, J4+4, J5-2, J6+2])
        # p9
    point.append ([J1+6, J2+4, J3, J4-2, J5-6, J6+2])
        # p10
    point.append ([J1-4, J2+4, J3+2, J4-2, J5+4, J6-2])
        # p11
    point.append ([J1-2, J2+2, J3-2, J4+4, J5+4, J6-3])
        # p12
    point.append ([J1-4, J2, J3-2, J4+4, J5+2, J6+4])
        # p13
    point.append ([J1, J2+2, J3+4, J4-4, J5, J6+5])
        # p14
    point.append ([J1+4, J2, J3+2, J4-2, J5, J6-7])
        # p15
    point.append ([J1-2, J2-4, J3+2, J4-4, J5+4, J6])
        # p16
    point.append ([J1-2, J2-2, J3+4, J4-4, J5-2, J6-2])
        # p17
    point.append ([J1+2, J2+2, J3+2, J4+4, J5+2, J6+1])
        # p18
    point.append ([J1+1, J2+2, J3+1, J4+2, J5+2, J6-3])
        # p19
    point.append ([J1+3, J2-2, J3+1, J4+1, J5+2, J6+4])
        # p20
    point.append ([J1+3, J2+2, J3, J4-1, J5-3, J6+2])
    

    socket_buf_len = 256
    exec_index = 0#拍照次数
    robot_speed =  1000

    if  erobot.state_check()==False:
        erobot.start_robot()

    while exec_index <len(point):
        exec_index = exec_index +1
        print("第 %d 次拍照" %exec_index)
        print("移动到拍照位")
        #移动
        erobot.write_angles(point[exec_index-1], robot_speed)
        time.sleep(3)
        wait_done()
        get_send_TCP(sock_rvs)
        print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
       
