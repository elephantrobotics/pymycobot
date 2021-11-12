#!/usr/bin/env python3
# coding:utf-8
import re
import socket
from pymycobot import MyCobot


def re_data_1(command):
    # (all_data)
    r1 = re.compile(r'[(](.*?)[)]')
    all_data = re.findall(r1, command)[0]
    return all_data

def re_data_2(command):
    r2 = re.compile(r'[[](.*?)[]]')
    data_list = re.findall(r2, command)[0]
    return data_list
    
def get_data(command):
    # get speed
    # get () content
    all_data = re_data_1(command) # "[0,0,0,0,0,0],60"
    # get laet ,
    spe = re.search(r'\s*([^,]*)$',all_data)
    speed = int(spe.group(1))

    # get angles or coords
    # get [] content
    angles = [float(i) for i in re_data_2(all_data).split(",")]
    return angles,speed

def main(HOST,PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST,PORT))
    print("Binding succeeded!")
    s.listen(1)
    try:
        while True:
            conn, addr = s.accept()
            port_baud = []
            command = ""
            while True:
                try:
                    data = conn.recv(1024)
                    command = data.decode('utf-8')
                    if data.decode('utf-8') == "" or data.decode('utf-8') == "end":
                        break
                    res = None
                    command = command.replace(" ","")
                    if len(port_baud)<2:
                        port_baud.append(command)
                    if len(port_baud)==2:
                        mc = MyCobot(port_baud[0],port_baud[1])
                        port_baud.append(1)
                    if "get_angles" in command:
                        res = mc.get_angles()
                        
                    elif command == "power_on":
                        mc.power_on()

                    elif command == "power_off":
                        mc.power_off()

                    elif command == "is_power_on":
                        res = mc.is_power_on()
                    elif command == "release_all_servos":
                        mc.release_all_servos()
                    elif command == "is_controller_connected":
                        res = mc.is_controller_connected()

                    elif "sync_send_angles" in command:
                        all_data = re_data_1(command)
                        angles = [float(i) for i in re_data_2(command).split(",")]
                        datas = all_data.split(',')
                        # get laet ,
                        if len(datas) == 7:
                            speed = int(datas[-1])
                        
                            timeout = 7
                        else:
                            speed = int(datas[-2])
                            timeout = int(datas[-1])
                        mc.sync_send_angles(angles,speed,timeout)
                        
                        while True:
                            if mc.is_in_position(angles,0):
                                break

                    elif "sync_send_coords" in command:
                        all_data = re_data_1(command)
                        coords = [float(i) for i in re_data_2(command).split(",")]
                        datas = all_data.split(',')
                        if len(datas) == 8:
                            speed = int(datas[-2])
                            mode = int(datas[-1])
                            timeout = 7
                        else:
                            speed = int(data[-3])
                            mode = int(datas[-2])
                            timeout = int(datas[-1])
                        mc.sync_send_coords(coords,speed,mode,timeout)
                        while True:
                            if mc.is_in_position(coords,1):
                                break

                    elif "send_angles" in command:
                        angles, speed = get_data(command)
                        mc.send_angles(angles,speed)
                    elif "send_angle" in command:
                        all_data = re_data_1(command).split(',') # ['1','60','40']
                        mc.send_angle(int(all_data[0]),float(all_data[1]),int(all_data[2]))
                    elif command == "get_coords":
                        res = mc.get_coords()

                    elif "send_coords" in command:
                        all_data = re_data_1(command)
                        coords = [float(i) for i in re_data_2(command).split(",")]
                        speed = int(all_data.split(',')[-2])
                        model = re.search(r'\S([^,]*)$',all_data)
                        model = int(model.group(1))
                        mc.send_coords(coords,speed,model)

                    elif "is_in_position" in command:
                        data_list, id = get_data(command)
                        res = mc.is_in_position(data_list,id)
                        
                    elif "is_moving" in command:
                        res = mc.is_moving()
                        

                    elif "jog_angle" in command:
                        all_data = re_data_1(command).split(',') # ['1','1','70']
                        mc.jog_angle(int(all_data[0]),int(all_data[1]),int(all_data[2]))

                    elif "jog_coord" in command:
                        all_data = re_data_1(command).split(',') # ['x','1','70']
                        mc.jog_angle(int(all_data[0]),int(all_data[1]),int(all_data[2]))
                    
                    elif "stop" in command:
                        mc.stop()

                    elif "jog_stop" in command:
                        mc.jog_stop()

                    elif "set_encoders" in command:
                        encoders,speed = get_data(command)
                        mc.set_encoders(encoders,speed)

                    elif "set_encoder" in command:
                        
                        all_data = re_data_1(command).split(',') # ['1','2046']
                        mc.set_encoder(int(all_data[0]),int(all_data[1]))
                    
                    elif "get_encoders" in command:
                        res = mc.get_encoders()
                        

                    elif "get_encoder" in command:
                        
                        all_data = re_data_1(command) # '1'
                        res = mc.get_encoder(int(all_data))
                        

                    elif "is_paused" in command:
                        res = mc.is_paused()
                        

                    elif "paues" in command:
                        mc.pause()

                    elif "resume" in command:
                        mc.resume()

                    elif "get_speed" in command:
                        res = mc.get_speed()
                        

                    elif "set_speed" in command:
                        
                        all_data = re_data_1(command) # '100'
                        mc.set_speed(int(all_data))

                    elif "get_joint_min_angle" in command:
                        
                        all_data = re_data_1(command) # '1'
                        res = mc.get_joint_min_angle(int(all_data))
                        

                    elif "get_joint_max_angle" in command:
                        
                        all_data = re_data_1(command) # '1'
                        res = mc.get_joint_max_angle(int(all_data))
                        

                    elif "is_servo_enable" in command:
                        
                        all_data = re_data_1(command) # '1'
                        res = mc.is_servo_enable(int(all_data))
                        

                    elif "is_all_servo_enable" in command:
                        res = mc.is_all_servo_enable()
                        

                    elif "set_servo_data" in command:
                        
                        all_data = re_data_1(command).slipt(',') # '1,20,2046'
                        mc.set_servo_data(int(all_data[0]),int(all_data[1]),int(all_data[2]))

                    elif "get_servo_data" in command:
                        
                        all_data = re_data_1(command).slipt(',') # '1,20'
                        res = mc.get_servo_data(int(all_data[0]),int(all_data[1]))
                        

                    elif "set_servo_calibration" in command:

                        all_data = re_data_1(command) # '1'
                        mc.set_servo_calibration(int(all_data))

                    elif "release_servo" in command:

                        all_data = re_data_1(command) # '1'
                        
                        mc.release_servo(int(all_data))

                    elif "focus_servo" in command:

                        all_data = re_data_1(command) # '1'
                        mc.focus_servo(int(all_data))

                    elif "set_color" in command:

                        all_data = re_data_1(command).slipt(',') # ['0','0','0']
                        mc.set_color(int(all_data[0]),int(all_data[1]),int(all_data[2]))

                    elif "set_pin_mode" in command:

                        all_data = re_data_1(command).slipt(',') # ['20','0']
                        mc.set_pin_mode(int(all_data[0]),int(all_data[1]))

                    elif "set_digital_output" in command:

                        all_data = re_data_1(command).slipt(',') # ['20','0']
                        mc.set_digital_output(int(all_data[0]),int(all_data[1]))

                    elif "get_digital_input" in command:

                        all_data = re_data_1(command) # '3'
                        mc.get_digital_input(int(all_data))

                    elif "set_pwm_output" in command:

                        all_data = re_data_1(command).slipt(',') # ['20','0','100']
                        mc.set_pwm_output(int(all_data[0]),int(all_data[1]),int(all_data[2]))

                    elif "get_gripper_value" in command:
                        res = mc.get_gripper_value()
                        

                    elif "set_gripper_state" in command:

                        all_data = re_data_1(command).slipt(',') # ['0','100']
                        mc.set_gripper_state(int(all_data[0]),int(all_data[1]))

                    elif "set_gripper_value" in command:

                        all_data = re_data_1(command).slipt(',') # ['0','100']
                        mc.set_gripper_value(int(all_data[0]),int(all_data[1]))

                    elif "set_gripper_ini" in command:
                        mc.set_gripper_ini()

                    elif "is_gripper_moving" in command:
                        res = mc.is_gripper_moving()
                        

                    elif "get_basic_input" in command:

                        all_data = re_data_1(command) # 2
                        res = mc.get_basic_input(int(all_data))
                        

                    elif "set_basic_output" in command:

                        all_data = re_data_1(command).slipt(',') # ['20','1']
                        mc.set_basic_output(int(all_data[0]),int(all_data[1]))

                    elif "gpio_init" in command:
                        mc.gpio_init()

                    elif "gpio_output" in command:
                        all_data = re_data_1(command).slipt(',') # ['20','1']
                        mc.gpio_output(int(all_data[0]),int(all_data[1]))
                    
                    conn.sendall(str.encode(str(res)))
                except Exception as e:
                    conn.sendall(str.encode("ERROR:"+str(e)))
                    break          

    except:
        conn.close()

if __name__ == "__main__":
    HOST = socket.gethostbyname(socket.gethostname())
    PORT = 9000
    main(HOST,PORT)
