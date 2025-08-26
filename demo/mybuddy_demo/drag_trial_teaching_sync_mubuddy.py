'''
Drag and teach in windows version
'''
from pickle import FALSE
import time
import os
import sys
import threading
import json
import serial
import serial.tools.list_ports
import platform


sys.path.append(os.getcwd())
from pymycobot import MyBuddy


mb: MyBuddy
sp: int = 80
os_version: str
WIN = False
LINUX = False

def setup():
    print("")
    global port, mb, baud, os_version, WIN, LINUX
    os_version = platform.system()
    if os_version == 'Windows':
        WIN = True
        LINUX = False
    elif os_version == 'Linux':
        WIN = False
        LINUX = True
    if WIN:
        port = 'COM21'
        baud = 115200
    elif LINUX:
        port = '/dev/ttyACM0'
        baud = 115200
    mb = MyBuddy(port, baud, debug=True)


class Raw(object):
    """Set raw input mode for device"""


    def __init__(self, stream):
        import termios
        import tty
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
        print("\r{}".format(" " * self.w))
        print("\r{}".format(msg))


class TeachingTest(Helper):
    def __init__(self, mycobot) -> None:
        super().__init__()
        self.mb = mb
        self.recording = False
        self.playing = False
        self.record_list = []
        self.record_t = None
        self.play_t = None

    def record(self):
        self.record_list = []
        self.recording = True

        def _record():
            start_t = time.time()
            _id = 0
            while self.recording:
                _encoders = self.mb.get_encoders(_id)
                if _encoders:
                    if _encoders[-2:-1]:
                        self.record_list.append(_encoders)
                        time.sleep(0.05)
                    # print("\r {}".format(time.time() - start_t), end="")
        self.echo("Start recording.")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        if self.recording:
            self.recording = False
            self.record_t.join()
            self.echo("Stop record")

    def play(self):
        self.echo("Start play")
        for _encoders_data in self.record_list:
            print(_encoders_data)
            _encoders = _encoders_data[0:7] + _encoders_data[14:21] + _encoders_data[-2:-1]
            _speeds = _encoders_data[7:14] + _encoders_data[21:28] + _encoders_data[-1:]
            self.mb.set_encoders(0, _encoders,_speeds)
            time.sleep(0.05)
        self.echo("Finish play")

    def loop_play(self):
        self.playing = True

        def _loop():
            len_ = len(self.record_list)
            i = 0
            while self.playing:
                idx_ = i % len_
                i += 1
                _encoders_data = self.record_list[idx_]
                print(_encoders_data)
                _encoders = _encoders_data[0:7] + _encoders_data[14:21] + _encoders_data[-2:-1]
                _speeds = _encoders_data[7:14] + _encoders_data[21:28] + _encoders_data[-1:]
                self.mb.set_encoders(0, _encoders,_speeds)
                time.sleep(0.05)

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

        with open(os.path.dirname(__file__) + "/record.txt", "w") as f:
            json.dump(self.record_list, f, indent=2)
            self.echo("save dir:  {}".format(os.path.dirname(__file__)))

    def load_from_local(self):

        with open(os.path.dirname(__file__) + "/record.txt", "r") as f:
            try:
                data = json.load(f)
                self.record_list = data
                self.echo("Load data success.")
            except Exception:
                self.echo("Error: invalid data.")

    def print_menu(self):
        print(
            """\
        \r q + Enter: quit
        \r r + Enter: start record
        \r c + Enter: stop record
        \r p + Enter: play once
        \r P + Enter: loop play / stop loop play
        \r s + Enter: save to local
        \r l + Enter: load from local
        \r f + Enter: release mycobot
        \r----------------------------------
            """
        )

    def start(self):
        global WIN, LINUX
        self.print_menu()
        while not False:
            if WIN:
                key = input()
            elif LINUX:
                with Raw(sys.stdin):
                    key = sys.stdin.read(1)
            if key == "q":
                break
            elif key == "r":  # recorder
                self.record()
            elif key == "c":  # stop recorder
                self.stop_record()
            elif key == "p":  # play
                self.play()
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
                self.mb.release_all_servos(0)
                time.sleep(0.05)
                self.mb.release_all_servos(2)
                self.echo("Released")
            else:
                print(key)
                continue


if __name__ == "__main__":
    setup()
    recorder = TeachingTest(mb)
    recorder.start()
