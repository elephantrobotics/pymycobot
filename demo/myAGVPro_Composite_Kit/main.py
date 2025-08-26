#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from composite_kit import AGVProCompositeKit
from core.controller import UndefinedController, MyCobot630Controller, MyCobot320Controller, MyArmMControl
from core.drive import AGVDriveAPI

# ##############################################################################################
# MyCobot Pro630 Configuration
# ##############################################################################################
COMPOSITE_KIT_HOST = "192.168.1.100"
COMPOSITE_KIT_PORT = 5001

# ##############################################################################################
# MyCobot M750 and MyCobot 320 Configuration
# ##############################################################################################
COMPOSITE_KIT_COMPORT = "/dev/ttyACM2"

# ##############################################################################################
# MyAGV Pro Configuration
# ##############################################################################################
AGVPRO_DRIVE_COMPORT = "/dev/ttyACM0"

# ##############################################################################################
# Composite kit configuration
# ##############################################################################################
COMPOSITE_KIT_TYPE = "Undefined"  # MyCobotPro630, MyCobot320, MyArmM750, Undefined
DEBUG = False


def main():
    agvpro_driver = AGVDriveAPI(comport=AGVPRO_DRIVE_COMPORT, debug=DEBUG)

    if COMPOSITE_KIT_TYPE == "MyCobotPro630":
        print(f" # Connecting to MyCobot Pro630 at {COMPOSITE_KIT_HOST}:{COMPOSITE_KIT_PORT}...")
        arm_controller = MyCobot630Controller(host=COMPOSITE_KIT_HOST, port=COMPOSITE_KIT_PORT, debug=DEBUG)

    elif COMPOSITE_KIT_TYPE == "MyCobot320":
        print(f" # Connecting to MyCobot 320 at {COMPOSITE_KIT_COMPORT}...")
        arm_controller = MyCobot320Controller(port=COMPOSITE_KIT_COMPORT, debug=DEBUG)

    elif COMPOSITE_KIT_TYPE == "MyArmM750":
        print(f" # Connecting to MyArm M750 at {COMPOSITE_KIT_COMPORT}...")
        arm_controller = MyArmMControl(port=COMPOSITE_KIT_COMPORT, debug=DEBUG)

    else:
        print(" # Undefined composite kit type.")
        arm_controller = UndefinedController()

    composite_kit = AGVProCompositeKit(arm_controller=arm_controller, agv_pro_driver=agvpro_driver)

    if composite_kit.init_joystick():
        print(" # Joystick initialized.")
    else:
        print(" # (Error) Joystick initialization failed.")
        print(" # (Error) Please check the connection of the joystick.")
        return

    composite_kit.mainloop()
    print(" # Exit.")


if __name__ == '__main__':
    main()
