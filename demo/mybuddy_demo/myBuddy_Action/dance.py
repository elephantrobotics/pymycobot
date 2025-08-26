# coding=utf8
import os
import time
from pymycobot.mybuddy import MyBuddy
import threading
from threading import Lock, Thread
import cv2 as cv

# Establish serial connection
mc = MyBuddy('/dev/ttyACM0', 115200)

t = 1


def run():
    while True:
        mc.send_angles(1, [0, 0, 0, 0, 0, 0], 50)
        time.sleep(0.05)
        mc.send_angles(2, [0, 0, 0, 0, 0, 0], 50)
        time.sleep(2 * t)
        mc.send_angles(1, [0, 0, 90, 0, 0, 0], 50)
        time.sleep(0.05)
        mc.send_angles(2, [0, 0, -90, 0, 0, 0], 50)
        time.sleep(2 * t)
        for i in range(3):
            mc.send_angles(1, [0, 0, 115, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [0, 0, -65, 0, 0, 0], 50)
            time.sleep(2 * t)
            mc.send_angles(1, [0, 0, -115, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [0, 0, 65, 0, 0, 0], 50)
            time.sleep(2 * t)

        for i in range(3):
            mc.send_angles(1, [45, 0, 90, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [-45, 0, -90, 0, 0, 0], 50)
            time.sleep(2 * t)
            mc.send_angles(1, [-45, 0, 90, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [45, 0, -90, 0, 0, 0], 50)
            time.sleep(2 * t)
        mc.send_angles(1, [180, 0, 90, 0, 0, 0], 50)
        time.sleep(0.05)
        mc.send_angles(2, [-180, 0, -90, 0, 0, 0], 50)
        time.sleep(2 * t)
        for i in range(3):
            mc.send_angles(1, [180, 0, 135, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [-180, 0, -45, 0, 0, 0], 50)
            time.sleep(2 * t)
            mc.send_angles(1, [180, 0, 45, 0, 0, 0], 50)
            time.sleep(0.05)
            mc.send_angles(2, [-180, 0, -135, 0, 0, 0], 50)
            time.sleep(2 * t)


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
            cv.imshow(out_win, frame)
        if cv.waitKey(1) & 0xFF == ord('q') or ret == False:
            cap = cv.VideoCapture("/home/ubuntu/emo/look_happy.mp4")


if __name__ == '__main__':
    # run with multithreading
    t1 = threading.Thread(target=run())
    t2 = threading.Thread(target=smile)
    t1.start()
    t2.start()
