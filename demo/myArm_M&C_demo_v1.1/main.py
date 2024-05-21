import sys
import serial.tools.list_ports
import os

from PySide6.QtWidgets import QApplication, QWidget, QHeaderView
from PySide6.QtGui import QPixmap, QIcon
from functools import partial
from ui.tool_ui import Ui_Form
"""

"""
from ui.bt import SwitchButton
from get_date import CreateSerial

class MyWindow(QWidget, Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.icon = QIcon()
        self.icon.addPixmap(QPixmap(os.getcwd()+"/resources/mystudio.ico"), QIcon.Normal, QIcon.Off)

        self.setWindowIcon(self.icon)
        # 设置表格列宽均分
        
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_2.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_2.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.tableWidget_3.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_3.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.tableWidget_4.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget_4.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        
        self.serial_ports = serial.tools.list_ports.comports()
        for port in self.serial_ports:
            self.comboBox.addItem(port.device)
            self.comboBox_2.addItem(port.device)
            self.comboBox_3.addItem(port.device)
            self.comboBox_4.addItem(port.device)
            
        self.switch_button = SwitchButton()
        self.switch_button.status_button.connect(partial(self.control_serial, 1))
        self.horizontalLayout.addWidget(self.switch_button)
        
        self.switch_button_1 = SwitchButton()
        self.switch_button_1.status_button.connect(partial(self.control_serial, 2))
        self.horizontalLayout_2.addWidget(self.switch_button_1)
        
        self.switch_button_2 = SwitchButton()
        self.switch_button_2.status_button.connect(partial(self.control_serial, 3))
        self.horizontalLayout_3.addWidget(self.switch_button_2)
        
        self.switch_button_3 = SwitchButton()
        self.switch_button_3.status_button.connect(partial(self.control_serial, 4))
        self.horizontalLayout_4.addWidget(self.switch_button_3)
        
        self.serial_obj = {1:None, 2:None, 3:None, 4:None}


    def control_serial(self, index, status):
        """串口控制"""
        if index == 1:
            if self.comboBox.currentText() != "":
                if status:
                    if self.serial_obj[index] is not None:
                        self.serial_obj[index].open()
                    else:
                        self.serial_obj[index] = CreateSerial(self.comboBox.currentText(), index, self.serial_obj[3])
                        self.serial_obj[index].progress.connect(self.add_row)
                        self.serial_obj[index].start()
                else:
                    # self.serial_obj[index].next_close = True
                    self.serial_obj[index].close()
            else:
                self.switch_button.reset_state()
        elif index == 2:
            if self.comboBox_2.currentText() != "":
                if status:
                    print(index)
                    if self.serial_obj[index] is not None:
                        # print("open")
                        self.serial_obj[index].open()
                    else:
                        # print("create")
                        self.serial_obj[index] = CreateSerial(self.comboBox_2.currentText(), index, self.serial_obj[4])
                        self.serial_obj[index].progress.connect(self.add_row)
                        self.serial_obj[index].start()
                else:
                    self.serial_obj[index].close()
            else:
                self.switch_button_1.reset_state()
        elif index == 3:
            if self.comboBox_3.currentText() != "":
                if status:
                    if self.serial_obj[index] is not None:
                        self.serial_obj[index].open()
                    else:
                        self.serial_obj[index] = CreateSerial(self.comboBox_3.currentText(), index)
                        self.serial_obj[index].progress.connect(self.add_row)
                        self.serial_obj[index].start()
                else:
                    self.serial_obj[index].close()
            else:
                self.switch_button_2.reset_state()
        elif index == 4:
            if self.comboBox_4.currentText() != "":
                if status:
                    # print(index)
                    if self.serial_obj[index] is not None:
                        self.serial_obj[index].open()
                    else:
                        # print("create4")
                        self.serial_obj[index] = CreateSerial(self.comboBox_4.currentText(), index)
                        self.serial_obj[index].progress.connect(self.add_row)
                        self.serial_obj[index].start()
                else:
                    self.serial_obj[index].close()
            else:
                self.switch_button_3.reset_state()
        
        
    def add_row(self, data):
        """更新表格数据"""
        # print("get: ",data)
        if '1' in data.keys():
        # if '1' in data.keys() or '3' in data.keys():
            # if self.serial_obj[3] is not None and self.serial_obj[3].serial.port_handler.is_open:
            #     # count = 0
            #     IDs = [1, 2, 3, 4, 5, 6, 7,8] 
            #     poss = data['1']["angle"]
                
            #     for i in range(8):
            #         if (IDs[i] == 6 or IDs[i] == 8):
            #             poss[i] = 4096-poss[i]
            #         self.serial_obj[3].serial.set_servo_encoder_postion(IDs[i], poss[i], 4000)
            #         count += 1
            tableWidget = self.tableWidget
            index = '1'
            # res = ['1', '3']
        elif '2' in data.keys():
        # elif '2' in data.keys() or '4' in data.keys():
            tableWidget = self.tableWidget_2
            index = '2'
            # res = ['2', '4']
        elif '3' in data.keys():
            tableWidget = self.tableWidget_3
            index = '3'
        elif '4' in data.keys():
            tableWidget = self.tableWidget_4
            index = '4'
        # for j in res:
        # index = j
        
        # if j == "1":
        #     tableWidget = self.tableWidget
        # elif j == "2":
        #     tableWidget = self.tableWidget_2
        # elif j == "3":
        #     tableWidget = self.tableWidget_3
        #     j = "1"
        # elif j == "4":
        #     tableWidget = self.tableWidget_4
        #     j = "2"
        # index = j
            
        for i in range(8):
            angle = tableWidget.item(i, 1)
            angle.setText(str(data[index]["angle"][i]))
            speed = tableWidget.item(i, 2)
            speed.setText(str(data[index]["speed"][i]))
            # acc = tableWidget.item(i, 3)
            # acc.setText(str(data[index]["acc"][i]))
        # if 

        
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec())
