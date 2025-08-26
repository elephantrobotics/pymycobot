
from pymycobot import MyArmC, MyArmM
from pymycobot.robot_info import RobotLimit

import time
from PySide6.QtCore import QThread, Signal

class CreateSerial(QThread):
    progress = Signal(dict)

    def __init__(self, port, index, parent_serial=None):
        super().__init__()
        if index in [1,2]:
            self.serial = MyArmC(port, 1000000)
        else:
            self.serial = MyArmM(port, 1000000)
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
                        data["angle"] = self.serial.get_joints_angle()
                        data["angle"][-1] = round((data["angle"][-1] - 0.08) / (-95.27 - 0.08) * (-123.13 + 1.23) - 1.23, 2)
                        if data["angle"][-1] > 2:
                            data["angle"][-1] = 2
                        elif data["angle"][-1] < -118:
                            data["angle"][-1] = -118
                        for i in range(len(data["angle"])):
                            if data["angle"][i] > 0 and (RobotLimit.robot_limit["MyArmM"]["angles_max"][i] - data["angle"][i]) < 5:
                                data["angle"][i] = RobotLimit.robot_limit["MyArmM"]["angles_max"][i] - 5
                            elif data["angle"][i] < 0 and (data["angle"][i] - RobotLimit.robot_limit["MyArmM"]["angles_min"][i]) < 5:
                                data["angle"][i] = RobotLimit.robot_limit["MyArmM"]["angles_min"][i] + 5
                        self.parent_serial.serial.set_joints_angle(data["angle"], 100)
                    else:
                        # data["angle"] = self.serial.get_servos_encoder()
                        # data["speed"] = self.serial.get_servos_speed()
                        time.sleep(1)
                    self.progress.emit({str(self.index):data})
                    # if data["angle"]:
                    #     self.progress.emit({str(self.index):data})
            except:
                pass
            time.sleep(0.0001)