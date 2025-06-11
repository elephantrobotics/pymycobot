# -*- coding: utf-8 -*-

import socket

import time
import numpy as np
from pymycobot import ElephantRobot
import sys






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
    
    result =erobot.get_coords()
    robot_TCP = result.copy()
    print("***get tcp***%s"%time.asctime())
    print(result)
  
    resCutStr = ' '.join(repr(e) for e in result)    
    print(resCutStr) 
    #send tcp
    sendStr = "SET_POSE " + resCutStr + '#'
    print("***send tcp***%s"%time.asctime())
    print(sendStr) 
    sock_rvs.send(bytes(sendStr,encoding="utf-8"))
    print("***tcp_receive data***%s"%time.asctime())
    read_time=time.time()
    data=sock_rvs.recv(socket_buf_len)
    print(data)
    print("read_time=",time.time()-read_time)

    data_arr = data.decode().split(",")
    print("data_arr =",data_arr )
    if data_arr[1]=="false":
        exit()
        print("未识别到工件")
    pose_offet = [0, 0, 0, 0, 0, 0] 
    pose_real = [0, 0, 0, 0, 0, 0] 

    tmp1=data_arr[0].split()
    tmp2=data_arr[2].split()


    
    for j in range(0,6):
        pose_offet[j] = float(tmp1[j])
        pose_real[j] = float(tmp2[j])


    # if pose_real[5] - result[5] > 90:
    #     # pose_offet[5]
    #     pose_real[3] *= -1
    #     pose_real[4] *= -1
    #     pose_real[5] -= 180
    # if pose_real[5] - result[5] < -90:
    #     pose_real[3] *= -1
    #     pose_real[4] *= -1
    #     pose_real[5] += 180

    # pose_offet[3]=pose_real[3]
    # pose_offet[4]=pose_real[4]
    # pose_offet[5]=pose_real[5]

    # print("after", pose_real)

    print("pose_offet=",pose_offet)
    print("pose_real=",pose_real)
    # exit()

    return [pose_offet,pose_real],data_arr[1]


def wait_done():
    time.sleep(3)
    while erobot.command_wait_done()!="0":   
        # print("fail")
        time.sleep(6)


# photo_point = [-10.01, -112.825, 104.148, -82.116, -89.213, -28.698]


photo_point = [-8.489, -102.576, 97.113, -85.259, -89.232, -31.509]
a=[41.106, -82.693, 114.778, -123.192, -90.124, 22.411]
b=[55.486, -55.102, 76.027, -111.967, -90.395, 36.775]
c=[37.356, -31.283, 32.527, -86.828, -90.269, 19.083]
d=[25.567, -57.895, 80.522, -113.726, -89.821, 6.878]

target_name=["tee","elbow","ball_valve","through"]

if __name__ == "__main__":
    
    rvs_ip = "localhost"
    rvs_port = 2013
    robot_ip="192.168.6.79"
    erobot = ElephantRobot(robot_ip,5001) 
    erobot.start_client()
    conSuc_rvs, sock_rvs=connectRvsServer(rvs_ip, rvs_port)
    robot_speed = 1999
    socket_buf_len = 1024
    exec_index = 0
    erobot.set_digital_out(0,0)
    reference_pose=[322.511, 28.932, 172.421, 179.064, 0.526, 108.679]
    if  erobot.state_check()==False:
        erobot.start_robot()
        time.sleep(2)
        erobot._state_on()
    try:
        while 1:
            start_time=time.time()
            exec_index = exec_index +1
            print("第 %d 次拍照" %exec_index)

            
            print("移动到1号拍照位")
            erobot.write_angles(photo_point, robot_speed)
            
            erobot.command_wait_done()
            time.sleep(1)
        
            pose,id=get_send_TCP(sock_rvs)
            
            if id=="false":
                break

            # if pose[0][3]>0 and pose[0][3]<90:
            #     print("no")

            # elif pose[0][3]<0 and pose[0][3]>-90:
            #     # continue
            #     print("no")
             
            else:
                erobot.command_wait_done()
                time.sleep(1)
                erobot.write_coords(pose[0],robot_speed)
                erobot.command_wait_done()
                time.sleep(1)

                erobot.write_coords(pose[1],robot_speed)
                erobot.command_wait_done()
                time.sleep(1)
                erobot.set_digital_out(0,1)
                time.sleep(2)
                
                erobot.write_coords(pose[0],robot_speed)
                erobot.command_wait_done()
                time.sleep(1)
                  
                
                erobot.write_angles(photo_point,robot_speed)
                erobot.command_wait_done()
                time.sleep(1)

                if id==target_name[0]:
                    erobot.write_angles(a,robot_speed)
                    erobot.command_wait_done()
                    time.sleep(1)
                elif id==target_name[1]:
                    erobot.write_angles(b,robot_speed)
                    erobot.command_wait_done()
                    time.sleep(1)
                elif id==target_name[2]:
                    erobot.write_angles(c,robot_speed)
                    erobot.command_wait_done()
                    time.sleep(1)
                elif id==target_name[3]:
                    erobot.write_angles(d,robot_speed)
                    erobot.command_wait_done()
                    time.sleep(1)
                
                erobot.set_digital_out(0,0)
                time.sleep(2)

                erobot.write_angles(photo_point,robot_speed)
                erobot.command_wait_done()
                time.sleep(1)

                end_time=time.time()
                print("time=",end_time-start_time)

    except KeyboardInterrupt:
        erobot.set_digital_out(0,0)
        erobot.stop_client()
        sock_rvs.close()
        
        print("end")
                

         
                







        


