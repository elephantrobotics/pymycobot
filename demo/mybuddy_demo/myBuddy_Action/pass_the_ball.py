# coding=utf8
import os
import time
from pymycobot.mybuddy import MyBuddy
import threading
from threading import Lock, Thread
import cv2 as cv

# Establish serial connection
mc = MyBuddy('/dev/ttyACM0', 115200)


def run():
    # infinite loop
    while True:
        en1 = [3033, 1558, 2854, 1233, 193, 1024]
        en2 = [1064, 2324, 1061, 2737, 3882, 67]
        mc.set_encoders(1, en1, 10)
        time.sleep(0.05)
        mc.set_encoders(2, en2, 10)
        time.sleep(5)

        _en2 = [962, 3061, 1182, 2172, 3877, 115]
        _en1 = [3203, 908, 2679, 1690, 368, 969]
        mc.set_encoders(2, _en2, 10)
        time.sleep(5)
        _en2[5] += 500
        mc.set_encoders(2, _en2, 4)
        time.sleep(7)
        _en2[5] -= 500
        mc.set_encoders(2, _en2, 10)
        time.sleep(3)
        mc.set_encoders(2, en2, 10)
        time.sleep(5)
        mc.set_encoders(1, _en1, 10)
        time.sleep(5)
        _en1[5] -= 350
        mc.set_encoders(1, _en1, 4)
        time.sleep(5)
        _en1[5] += 350
        mc.set_encoders(1, _en1, 10)
        time.sleep(4)
        mc.set_encoders(1, en1, 10)
        time.sleep(5)


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
        # print(frame)
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
