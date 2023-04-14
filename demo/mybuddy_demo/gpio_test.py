#coding=utf-8
from pymycobot import MyBuddy
import time



mc = MyBuddy('/dev/ttyACM0',115200,debug=False)
mc.set_gpio_init_mode(0)

def pump_init_1(i,j):
    mc.set_gpio_setup(i,0)
    mc.set_gpio_setup(j,0)
    
def pump_init_1(i,j):
    mc.set_gpio_setup(i,1)
    mc.set_gpio_setup(j,1)
def pump_on(i,j):
    mc.set_gpio_output(i,0)
    mc.set_gpio_output(j,0)

def pump_off(i,j):
    mc.set_gpio_output(i,1)
    mc.set_gpio_output(j,1)
    


#It is used to check the input/output status of each I/O interface
for k in range(1,17):
    mc.set_gpio_setup(k,0)


while 1:
    _a = ''
    for k in range(1,17):
        _a += str(mc.get_gpio_input(k))
    time.sleep(1)
    print(_a)