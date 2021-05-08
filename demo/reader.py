import serial
import serial.tools.list_ports
from pymycobot.mycobot import MyCobot

port: str
mc: MyCobot
sp: int = 80


def setup():
    print("")
    global port, mc
    plist = list(serial.tools.list_ports.comports())
    idx = 1
    for port in plist:
        print("{} : {}".format(idx, port))
        idx += 1

    _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
    port = str(plist[int(_in) - 1]).split(" - ")[0].strip()
    print(port)
    print("")

    DEBUG = False
    f = input("Wether DEBUG mode[Y/n]:")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    # mc = MyCobot(port, debug=True)
    mc = MyCobot(port, debug=DEBUG)


def focus():
    for i in range(6):
        mc.focus_servo(i + 1)


def release():
    for i in range(6):
        mc.release_servo(i + 1)


def angles():
    print(mc.get_angles())


def coords():
    print(mc.get_coords())


if __name__ == "__main__":
    setup()
    while not False:
        in_ = int(input("number"))
        if in_ == 1:
            focus()
        elif in_ == 2:
            release()
        elif in_ == 3:
            angles()
        elif in_ == 4:
            coords()
