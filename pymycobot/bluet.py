# coding: utf-8
import sys

class BluetoothConnection:
    def __init__(self, bd_address=None, port=None):
        self.device = []
        import bluetooth
        self.bluetooth = bluetooth
        self.target_name = "mybuddy"
        self.nearby_devices = None
        self.bd_address = bd_address
        self.port = port
 
    def find_target_device(self):
        available_addr = []
        self.nearby_devices = self.bluetooth.discover_devices(lookup_names=True, duration=5)
        if self.nearby_devices:
            for addr, name in self.nearby_devices:
                if self.target_name == name:
                    available_addr.append(addr)
            return available_addr
        return None
 
    def connect_target_device(self):
        if self.bd_address:
            sock = self.bluetooth.BluetoothSocket(self.bluetooth.RFCOMM)
            sock.connect((self.bd_address, self.port))
            return sock
        target_address = self.find_target_device()
        if target_address:
            if len(target_address) > 1:
                device_info = ""
                i = 1
                sys.stdout.write("please select the device you want to connect:\n".format(4))
                for addr, name in target_address:
                    device_info += "{} >>> {} - {} \n".format(i,addr, name)
                sys.stdout.write(device_info)
                choose_device = input("please enter 1-{}:".format(len(target_address)))
                target_address = target_address[int(choose_device)-1][0]
            sock = self.bluetooth.BluetoothSocket(self.bluetooth.RFCOMM)
            try:
                sock.connect((target_address[0][0], 1))
                return sock
            except Exception as e:
                # print("connection fail\n", e)
                sock.close()
                return None
        return None
                
if __name__ == "__main__":
    pass