import os
import time
from pymycobot.mycobot import MyCobot


def gripper_test(mc):
    print("Start check IO part of api\n")
    # print()

    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    # Set the current position to (2048).
    # Use it when you are sure you need it.
    # Gripper has been initialized for a long time. Generally, there
    # is no need to change the method.
    # mc.set_gripper_ini()

    print("")
    mc.set_gripper_value(2048, 50)
    time.sleep(2)

    print("")
    mc.set_gripper_value(1500, 50)
    time.sleep(2)

    print("")
    mc.set_gripper_state(0, 70)
    time.sleep(2)

    print("")
    mc.set_gripper_state(1, 70)
    time.sleep(2)

    print("")
    print(mc.get_gripper_value())


if __name__ == "__main__":
    with open(os.path.dirname(__file__) + "/port.txt") as f:
        port = f.read().strip().replace("\n", "")
        print(port)
    mycobot = MyCobot(port)
    gripper_test(mycobot)
