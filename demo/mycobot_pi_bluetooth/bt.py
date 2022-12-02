import bluetooth
import pygatt
import pathlib
import time
import json
import hashlib
from func_timeout import func_timeout, FunctionTimedOut
import os
import sys
sys.path.append('.')

from typing import Tuple

import ai.action.move.movement

mars = ai.action.move.movement.Movements()


class BluetoothService:
    pass


def list_devices() -> Tuple[str, str]:
    nearby_devices = bluetooth.discover_devices(lookup_names=True)
    return nearby_devices


def filter_marscat(bluetooth_devices: Tuple[str, str]) -> Tuple[str, str]:
    marscats = []
    for addr, name in bluetooth_devices:
        name.encode('utf-8', 'replace')
        if name == 'marscat':
            marscats.append((addr, name))
    return marscats


def start_marscat_bt() -> None:
    server_sock = bluetooth.BluetoothSocket(bluetooth.L2CAP)
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]
    uuid = '26754beb-1bd0-4017-b341-154bed30b71a'
    name = 'marscat_bt'
    bluetooth.advertise_service(
        server_sock,
        name,
        service_id=uuid,
        service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE],
        provider='',
        description='',
        protocols=[bluetooth.OBEX_UUID])
    print("Waiting for connection on RFCOMM channel {}".format(port))

    client_sock, client_info = server_sock.accept()


    print("Accepted connection from", client_info)

    while True:
        time.sleep(0.1)
        try:
            data = client_sock.recv(1024)
        except OSError:
            print("Remote client disconnected")
            data = None

        if not data:
            client_sock.close()
            print("Close Client Socket")
            print("Waiting for connection on RFCOMM channel {}".format(port))
            client_sock, client_info = server_sock.accept()
            print("Accepted connection from", client_info)
            continue

        if data == b'run':
            pass
            #mars.set_run()
        elif data == b'walk':
            pass
            #mars.set_walk()
        elif data == b'stand':
            pass
            #mars.set_stand()
        elif data == b'sit':
            pass
            #mars.set_sit()
        elif data == b'left':
            pass
            #mars.set_turn()
        elif data == b'right':
            pass
            #mars.set_turn(-1)
        elif data == b'send_update_start':
            print('start receive')

            # recv file information
            data = client_sock.recv(1024)
            file_len = int(str(data, encoding='utf8'))
            print('file size : ', file_len)

            data = client_sock.recv(1024)
            file_md5 = str(data, encoding='utf8')
            print(file_md5)

            # receive marsai.zip
            if os.path.exists(str(pathlib.Path.home()) + '/marsai.zip'):
                os.system(
                    'rm {}'.format(str(pathlib.Path.home()) + '/marsai.zip'))
            os.system(
                'touch {}'.format(str(pathlib.Path.home()) + '/marsai.zip'))

            i = 0
            buffer = 4096 

            def recv_data(buffer):
                print('start recv_data')
                data = client_sock.recv(buffer)
                return data

            with open(str(pathlib.Path.home()) + '/marsai.zip', 'ab+') as f:
                while file_len != 0:
                    time.sleep(0.1)
                    data = recv_data(buffer)
                    #while True:
                    #    try:
                    #        time.sleep(0.1)
                    #        data = func_timeout(1, recv_data, (buffer, ))
                    #    except FunctionTimedOut:
                    #        print('time out') 
                    #        client_sock.send(str(file_len).encode('utf-8'))
                    #        client_sock.close()
                    #        print('client closed')
                    #        client_sock, client_info = server_sock.accept()
                    #        print('accepted')
                    #    except bluetooth.BluetoothError as e:
                    #        print("error")
                    #    else:
                    #        break
                    f.write(data)
                    print('write', i, len(data))
                    i += 1
                    file_len -= len(data)
                    print(file_len)
                    if file_len < buffer:
                        buffer = file_len
                print('write finished')
            print('receive over')

            # recv over flag and update marsai
            while True: 
                data = client_sock.recv(4096)
                if data == b'over_and_update':
                    md5 = get_md5(str(pathlib.Path.home()) + '/marsai.zip')
                    if file_md5 == md5:
                        os.system('sh ~/marsai/tools/stop-systemd-services.sh')
                        time.sleep(3)
                        os.system('unzip -o ~/marsai.zip -d ~/marsai')
                        time.sleep(1)
                        os.system('sh ~/marsai/tools/start-systemd-services.sh')
                        client_sock.send(b'update_over')
                        print('over')
                        break
                    else:
                        print('md5 error')
                        break
                else:
                    pass




        else:
            pass

        print("Received " + str(data))

    client_sock.close()
    print("Close Client Socket")

    server_sock.close()
    print("Stop Server")

def get_md5(_file):
    m = hashlib.md5()
    with open(_file, 'rb') as f:
        for line in f:
            m.update(line)
    md5_code = m.hexdigest()
    print(md5_code)
    return md5_code


if __name__ == "__main__":
    start_marscat_bt()
