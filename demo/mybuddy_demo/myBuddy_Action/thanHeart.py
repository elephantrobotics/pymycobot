# coding=utf8
import os
import time
from pymycobot.mybuddy import MyBuddy
import threading
from threading import Lock, Thread
import cv2 as cv


# Establish serial connection
mc = MyBuddy('/dev/ttyACM0', 115200)



# Release the arms and record the points passed
def read():
    time.sleep(3)
    mc.release_all_servos()
    time.sleep(10)
    t = time.time()
    record_list = []
    print(1)
    while time.time() - t < 15:
        angles_1 = mc.get_encoders(1)
        angles_2 = mc.get_encoders(2)
        if angles_1 and angles_2:
            record_list.append([angles_1, angles_2])
            time.sleep(0.1)

    # Print the read location information, copy and save it as a txt file
    for i in record_list:
        print(i)


# Take out and use the saved location
def write():
    time.sleep(1.5)
    # The file here uses the saved txt file
    data = list(filter(None, open(os.path.join(os.getcwd(), 'thanHeart.txt')).read().splitlines()))
    # infinite loop
    while True:
        for angles in data:
            print(angles)
            mc.set_encoders(1, eval(angles)[0], 100)
            time.sleep(0.05)
            mc.set_encoders(2, eval(angles)[1], 100)
            time.sleep(0.1)


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
    t1 = threading.Thread(target=write)
    t2 = threading.Thread(target=smile)
    t1.start()
    t2.start()
