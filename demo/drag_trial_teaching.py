import time
import os
import sys
import termios
import tty
import threading
import json
import serial
import serial.tools.list_ports

from pymycobot.mycobot280 import MyCobot280
from pymycobot.mycobot320 import MyCobot320
from pymycobot.mecharm270 import MechArm270
from pymycobot.myarm import MyArm
from pymycobot.mypalletizer260 import MyPalletizer260

port: str
mc: MyCobot280
sp: int = 80


def setup():
    global port, mc

    print("")

    print("1 - MyCobot280")
    print("2 - MyCobot320")
    print("3 - MechArm270")
    print("4 - MyArm300")
    print("5 - MyPalletizer260")
    _in = input("Please input 1 - 5 to choose:")
    robot_model = None
    if _in == "1":
        robot_model = MyCobot280
        print("MyCobot280\n")
        print("Please enter the model type:")
        print("1. Pi")
        print("2. Jetson Nano")
        print("3. RISCV")
        print("Default is Pi")
        model_type = input()

        if model_type == "2":
            port = "/dev/ttyTHS1"
        elif model_type == "3":
            port = "/dev/ttyAMA0"
        else:
            pass
    elif _in == "2":
        robot_model = MyCobot320
        print("MyCobot320\n")

    elif _in == "3":
        robot_model = MechArm270
        print("MechArm270\n")

    elif _in == "4":
        robot_model = MyArm
        print("MyArm300\n")

    elif _in == '5':
        robot_model = MyPalletizer260
        print('MyPalletizer260\n')

    else:
        print("Please choose from 1 - 5.")
        print("Exiting...")
        exit()

    plist = list(serial.tools.list_ports.comports())
    idx = 1
    for port in plist:
        print("{} : {}".format(idx, port))
        idx += 1

    if not plist:
        pass
    else:
        _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
        port = str(plist[int(_in) - 1]).split(" - ")[0].strip()
    print(port)
    print("")

    baud = 1000000
    _baud = input("Please input baud(default:1000000):")
    try:
        baud = int(_baud)
    except Exception:
        pass
    print(baud)
    print("")

    DEBUG = False
    f = input("Wether DEBUG mode[Y/n](default:n):")
    if f in ["y", "Y", "yes", "Yes"]:
        DEBUG = True
    mc = robot_model(port, baud, debug=DEBUG)
    mc.power_on()


class Raw(object):
    """Set raw input mode for device"""

    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class Helper(object):
    def __init__(self) -> None:
        self.w, self.h = os.get_terminal_size()

    def echo(self, msg):
        print("\r{}".format(" " * self.w), end="")
        print("\r{}".format(msg), end="")


class TeachingTest(Helper):
    def __init__(self, mycobot) -> None:
        super().__init__()
        self.mc = mycobot
        self.recording = False
        self.playing = False
        self.record_list = []
        self.record_t = None
        self.play_t = None
        self.path = os.path.dirname(os.path.abspath(__file__)) + "/record.txt"

    def record(self):
        self.record_list = []
        self.recording = True

        # self.mc.set_fresh_mode(0)
        if isinstance(self.mc, MyCobot320):
            self.mc.release_all_servos(1)
        else:
            self.mc.release_all_servos()

        def _record():
            while self.recording:
                start_t = time.time()
                if isinstance(self.mc, (MyArm, MyPalletizer260)):
                    angles = self.mc.get_encoders()
                    interval_time = time.time() - start_t
                    if angles:
                        record = [angles, interval_time]
                        self.record_list.append(record)
                        print("\r {}".format(time.time() - start_t), end="")
                else:
                    angles = self.mc.get_encoders()
                    speeds = self.mc.get_servo_speeds()
                    interval_time = time.time() - start_t
                    if angles and speeds:
                        record = [angles, speeds, interval_time]
                        self.record_list.append(record)
                        print("\r {}".format(time.time() - start_t), end="")

        self.echo("Start recording.")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):

        if isinstance(self.mc, MyArm):
            self.mc.power_on()
        else:
            self.mc.focus_all_servos()

        if self.recording:
            self.recording = False
            self.record_t.join()
            self.echo("Stop record")

    def play(self):
        self.echo("Start play")
        if isinstance(self.mc, (MyArm, MyPalletizer260)):
            for record in self.record_list:
                encoders, interval_time = record
                self.mc.set_encoders(encoders, 100)
                time.sleep(interval_time)
        else:
            i = 0
            for record in self.record_list:
                encoders, speeds, interval_time = record
                self.mc.set_encoders_drag(encoders, speeds)
                if i == 0:
                    time.sleep(3)
                i += 1
                time.sleep(interval_time)
        self.echo("Finish play")

    def loop_play(self):
        self.playing = True

        def _loop():
            while self.playing:
                self.play()

        self.echo("Start loop play.")
        self.play_t = threading.Thread(target=_loop, daemon=True)
        self.play_t.start()

    def stop_loop_play(self):
        if self.playing:
            self.playing = False
            self.play_t.join()
            self.echo("Stop loop play.")

    def save_to_local(self):
        if not self.record_list:
            self.echo("No data should save.")
            return
        with open(self.path, "w") as f:
            json.dump(self.record_list, f, indent=2)
            self.echo("save dir:  {}\n".format(self.path))

    def load_from_local(self):
        with open(self.path, "r") as f:
            try:
                data = json.load(f)
                self.record_list = data
                self.echo("Load data success.")
            except Exception:
                self.echo("Error: invalid data.")

    def print_menu(self):
        print(
            """\
        \r q: quit
        \r r: start record
        \r c: stop record
        \r p: play once
        \r P: loop play / stop loop play
        \r s: save to local
        \r l: load from local
        \r f: release mycobot
        \r----------------------------------
            """
        )

    def start(self):
        self.print_menu()

        while not False:
            with Raw(sys.stdin):
                key = sys.stdin.read(1)
                if key == "q":
                    break
                elif key == "r":  # recorder
                    self.record()
                elif key == "c":  # stop recorder
                    self.stop_record()
                elif key == "p":  # play
                    if not self.playing:
                        self.play()
                    else:
                        print("Already playing. Please wait till current play stop or stop loop play.")
                elif key == "P":  # loop play
                    if not self.playing:
                        self.loop_play()
                    else:
                        self.stop_loop_play()
                elif key == "s":  # save to local
                    self.save_to_local()
                elif key == "l":  # load from local
                    self.load_from_local()
                elif key == "f":  # free move
                    if isinstance(self.mc, MyCobot320):
                        self.mc.release_all_servos(1)
                    else:
                        self.mc.release_all_servos()
                    self.echo("Released")
                else:
                    print(key)
                    continue


if __name__ == "__main__":
    setup()
    recorder = TeachingTest(mc)
    recorder.start()
