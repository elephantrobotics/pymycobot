import time
import threading
import json

from pymycobot.mycobot import MyCobot

record = []
recording = False
need_stop = False
playing = False


def free(cobot):
    global recording, playing

    if not recording and not playing:
        cobot.set_free_mode()
    else:
        if recording:
            print('Please stop recording frist.')
        elif playing:
            print('Please stop playing frist.')


def record_angles(cobot):
    global record, recording, playing

    if playing:
        print('Please stop playing frist.')
        return
    if recording:
        print('It\'s already being recorded.')
        return

    recording = True
    record = []

    def _record():
        global recording, need_stop
        start_t = time.time()

        while True:
            angles = cobot.get_angles()
            if angles:
                record.append(angles)
                time.sleep(.1)
                print(time.time() - start_t)
            if need_stop:
                recording = False
                need_stop = False
                print('After recording.')
                return

    print('Start recording.')
    t = threading.Thread(target=_record)
    t.setDaemon(True)
    t.start()


def play_angles(cobot):
    global recording, playing
    if recording:
        print('Please stop recording frist.')
        return
    if playing:
        print('It\'s already being played.')
        return
    if not record:
        print('The record list is empty. There is nothing to play.')
        return

    def _play():
        global record, playing, need_stop
        start_t = time.time()

        for angles in record:
            # print(angles)
            cobot.send_angles(angles, 80)
            time.sleep(.1)
            print(time.time() - start_t)
            if need_stop:
                break
        need_stop = False
        playing = False
        print('After playing.')

    print('Start playing.')
    t = threading.Thread(target=_play)
    t.setDaemon(True)
    t.start()


def save_to_local():
    global recording, playing, record

    if not recording and not playing:
        with open('./record.txt', 'w') as f:
            json.dump(record, f, indent=2)
            pass
        print('After saving, check `record.txt` in the current directory.')
    else:
        if recording:
            print('Please stop recording frist.')
        elif playing:
            print('Please stop playing frist.')


def quit_program():
    global recording, playing
    recording = playing = False
    time.sleep(.2)
    exit(0)


def print_menu():
    print('''\
0: set free mode 
1: record
2: play
s: save data to local
q: quit
other: stop record or play
          ''')


def main():
    global need_stop

    # port = "/dev/cu.usbserial-02168958"
    port = input('input port:')
    cobot = MyCobot(port)

    print_menu()

    while True:
        _in = input('input command:')
        if _in == '0':
            free(cobot)
        elif _in == '1':
            record_angles(cobot)
        elif _in == '2':
            play_angles(cobot)
        elif _in == 's':
            save_to_local()
        elif _in == 'q':
            quit_program()
        else:
            if playing or recording:
                need_stop = True


if __name__ == '__main__':
    main()
