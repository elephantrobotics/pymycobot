# coding=utf8
import os
import time
from pymycobot.mybuddy import MyBuddy
import threading
from threading import Lock, Thread
import cv2 as cv


# Establish serial connection
mc = MyBuddy('/dev/ttyACM0', 115200)


def one():
    mc.set_encoders(1, [3169, 1439, 3383, 3033, 1045, 2042], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3029, 1439, 3383, 3033, 1045, 2042], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3169, 1439, 3383, 3033, 1045, 2042], 5)
    time.sleep(0.3)


def tow():
    mc.set_encoders(1, [3169, 1281, 3299, 3102, 1047, 2021], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3029, 1281, 3299, 3102, 1047, 2021], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3169, 1281, 3299, 3102, 1047, 2021], 5)
    time.sleep(0.3)


def three():
    mc.set_encoders(1, [3169, 1115, 3193, 3123, 1047, 2036], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3029, 1115, 3193, 3123, 1047, 2036], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3169, 1115, 3193, 3123, 1047, 2036], 5)
    time.sleep(0.3)


def four():
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3029, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(0.3)
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(0.3)


def five():
    mc.set_encoders(2, [949, 3068, 974, 1032, 3001, 2076], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [1049, 3068, 974, 1032, 3001, 2076], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [949, 3068, 974, 1032, 3001, 2076], 5)
    time.sleep(0.3)


def six():
    mc.set_encoders(2, [944, 3068, 974, 959, 3001, 2000], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [1044, 3068, 974, 959, 3001, 2000], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [944, 3068, 974, 959, 3001, 2000], 5)
    time.sleep(0.3)


def seven():
    mc.set_encoders(2, [3328, 1609, 3064, 1426, 1383, 1301], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [3628, 1609, 3064, 1426, 1383, 1301], 5)
    time.sleep(0.3)


def eight():
    mc.set_encoders(2, [3259, 1613, 2839, 1187, 1337, 1300], 5)
    time.sleep(0.3)
    mc.set_encoders(2, [3659, 1613, 2839, 1187, 1337, 1300], 5)
    time.sleep(0.3)


def run():
    one()
    one()
    five()
    five()
    six()
    six()
    five()
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(1)
    four()
    four()
    three()
    three()
    tow()
    tow()
    one()
    time.sleep(1)
    five()
    five()
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(0.7)
    four()
    four()
    three()
    three()
    tow()
    time.sleep(1)
    five()
    five()
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(1)
    four()
    four()
    three()
    three()
    tow()
    time.sleep(1)
    one()
    one()
    five()
    five()
    six()
    six()
    five()
    mc.set_encoders(1, [3169, 969, 2984, 3027, 1040, 2091], 5)
    time.sleep(1)
    four()
    four()
    three()
    three()
    tow()
    tow()
    one()


# show emoji
def smile():
    # The location where the emoji file is stored
    cap = cv.VideoCapture("/home/ubuntu/emo/face_video_3_2.mp4")
    out_win = "l"
    cv.namedWindow(out_win, cv.WINDOW_NORMAL)
    cv.setWindowProperty(out_win, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    # infinite loop
    while True:
        ret, frame = cap.read()
        if frame is not None:
            print(1)
            cv.imshow(out_win, frame)
        if cv.waitKey(1) & 0xFF == ord('q') or ret == False:
            cap = cv.VideoCapture("/home/ubuntu/emo/look_happy.mp4")


if __name__ == '__main__':
    # run with multithreading
    t1 = threading.Thread(target=run)
    t2 = threading.Thread(target=smile)
    t1.start()
    t2.start()
