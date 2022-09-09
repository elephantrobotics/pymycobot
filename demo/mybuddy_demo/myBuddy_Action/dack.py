# coding=utf8
import time
from pymycobot.mybuddy import MyBuddy
import threading
from threading import Lock, Thread
import cv2 as cv


# Establish serial connection
mc = MyBuddy('/dev/ttyACM0', 115200)
time.sleep(1.5)


def run():
    while True:
        mc.set_encoders(1, [18, 2028, 1024, 2087, 3088, 1024], 100)
        time.sleep(0.05)
        mc.set_encoders(2, [1994, 2098, 3072, 2027, 1088, 3072], 100)
        time.sleep(0.01)
        mc.set_encoder(3, 1, 1548, 100)
        time.sleep(3)
        mc.set_encoders(1, [2026, 2028, 1024, 2112, 3088, 1024], 100)
        time.sleep(0.05)
        mc.set_encoders(2, [3980, 2098, 3076, 2022, 1088, 3072], 100)
        time.sleep(0.01)
        mc.set_encoder(3, 1, 2548, 100)
        time.sleep(3)


# show emoji
def smile():
    # The location where the emoji file is stored
    cap = cv.VideoCapture("/home/ubuntu/emo/face_video_3_2.mp4")
    out_win = "l"
    cv.namedWindow(out_win, cv.WINDOW_NORMAL)
    cv.setWindowProperty(out_win, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
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
