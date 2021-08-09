from pymycobot.mycobot import MyCobot
from port_setup import setup

mc: MyCobot
sp: int = 80


def setup():
    print("")
    global mc
    mc = setup()


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
