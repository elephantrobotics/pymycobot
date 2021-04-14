import time, subprocess
from pymycobot.mycobot import MyCobot


def gripper_test(mc):
    print("Start check IO part of api\n")
    # print()

    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    print("Set the current position to zero")
    mc.set_gripper_ini()
    time.sleep(2)

    print("")
    mc.set_gripper_value(248 - 15, 50)
    time.sleep(2)

    print("")
    mc.set_gripper_value(248 + 15, 50)
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
    port = subprocess.check_output(["echo -n /dev/ttyUSB*"], shell=True).decode()
    mycobot = MyCobot(port)
    gripper_test(mycobot)
