# coding:utf-8
# // An highlighted block
import traceback
import socket
import time
import sys
# RPi's IP
SERVER_IP = "192.168.10.115"  # 输入正确的目标ip地址，请查看树莓派ip
SERVER_PORT = 9000

print("Starting socket: TCP...")

print("Please input!")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((SERVER_IP, SERVER_PORT))  # 打开链接
444
sock.sendall(str.encode("12312"))
received = sock.recv(1024)
print("read: " + received.decode())
while True:
    try:
        #
        # if len(data)>0:
        command = input()
        command += '\n'
        print("send: " + command)
        try:
            sock.sendall(str.encode(command))
            received = sock.recv(1024)
            print("read: " + received.decode())
        except ConnectionRefusedError:
            print('error')
            sock.close()
            pass
        except BlockingIOError:
            pass
        except:
            pass

    except Exception:
        # print("exception")
        print(traceback.format_exc())
        sys.exit(1)
