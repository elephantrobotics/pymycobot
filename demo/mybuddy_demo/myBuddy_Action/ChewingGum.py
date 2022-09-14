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
    time.sleep(10)
    t = time.time()
    record_list = []
    print(1)
    while time.time() - t < 5:
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
    data = list(filter(None, open(os.path.join(os.getcwd(), 'ChewingGum.txt')).read().splitlines()))
    for angles in data:
        data = eval(angles)[0]
        data[4] -= 280
        mc.set_encoders(1, data, 100)
        time.sleep(0.03)
        mc.set_encoders(2, eval(angles)[1], 100)
        time.sleep(0.2)
    time.sleep(1.5)
    mc.set_gripper_state(2, 1)
    time.sleep(1.5)
    # The file here uses the saved txt file
    data = list(filter(None, open(os.path.join(os.getcwd(), 'ChewingGum_2.txt')).read().splitlines()))
    for angles in data:
        data = eval(angles)[0]
        data[4] -= 280
        mc.set_encoders(1, data, 100)
        time.sleep(0.03)
        mc.set_encoders(2, eval(angles)[1], 100)
        time.sleep(0.2)
    time.sleep(1.5)
    mc.set_gripper_state(2, 0)
    time.sleep(1.5)
    # The file here uses the saved txt file
    data = list(filter(None, open(os.path.join(os.getcwd(), 'ChewingGum_3.txt')).read().splitlines()))
    for angles in data:
        data = eval(angles)[0]
        data[4] -= 280
        mc.set_encoders(1, data, 100)
        time.sleep(0.03)
        mc.set_encoders(2, eval(angles)[1], 100)
        time.sleep(0.2)


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
