
from pymycobot import MyArmC, MyArmM

import time
from PySide6.QtCore import QThread, Signal

class CreateSerial(QThread):
    progress = Signal(dict)

    def __init__(self, port, index, parent_serial=None):
        super().__init__()
        if index in [1,2]:
            self.serial = MyArmC(port)
        else:
            self.serial = MyArmM(port)
        self.index = index
        self.parent_serial = parent_serial
        self.serial_type = self.serial.__class__.__name__
        
    def close(self):
        self.serial._serial_port.close()
        
        
    def open(self):
        self.serial._serial_port.open()
        
        
    def run(self) -> None:
        data = {"angle":[1,2,3,4,5,6,7,8], "speed":[1,2,3,4,5,6,7,8] , "acc": [1,2,3,4,5,6,7,8]}        
        while True:
            
            try:
                if self.serial._serial_port.isOpen():
                    
                    
                    if (self.index == 1 or self.index == 2) and self.parent_serial is not None and data["angle"] and data["speed"]:
                        data["angle"] = self.serial.get_servos_encoder()
                        data["speed"] = self.serial.get_servos_speed()
                        data["angle"][3] = 4096 - data["angle"][3]
                        data["angle"][-1] *= 1.1
                        data["angle"][-1] = int(data["angle"][-1])
                        if data["angle"][-1] > 2048:
                            data["angle"][-1] = 2048
                        # self.parent_serial.set_servos_encoder_drag(data["angle"], data["speed"])
                        self.parent_serial.serial.set_servos_encoder(data["angle"], 100)
                    else:
                        data["angle"] = self.serial.get_servos_encoder()
                        data["speed"] = self.serial.get_servos_speed()
                        time.sleep(1)
                    if data["angle"] and data["speed"]:
                        self.progress.emit({str(self.index):data})
            except:
                pass
            time.sleep(0.0001)