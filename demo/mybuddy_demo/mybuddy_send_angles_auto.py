import imp


import os
import sys
sys.path.append(os.getcwd())
from pymycobot.mybuddy import MyBuddy

mb = MyBuddy('COM4',115200)

mb.send_angles_auto(1,[0,0,0,0,0,0],1)

