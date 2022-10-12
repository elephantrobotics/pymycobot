# coding=utf-8

from __future__ import division
import cv2 as cv
import time
import threading
import pynput.mouse as pm


class Emoticon:
    def __init__(self, file_path, window_size: list = [], play_time=5) -> None:
        """API for playing emoticons

        Args:
            file_path: Absolute path of facial expression video.
            window_size: `[Length, width] `Size of the playback window (default is full screen).
            play_time: Playback duration. 5 seconds by default

        """
        self.file_path = file_path
        self.window_size = window_size
        self.play_time = play_time
        self.quit = False

    @property
    def window_size(self):
        return self.window_size

    @property
    def play_time(self):
        return self.play_time

    def on_click(self, x, y, button, pressed):
        if pressed:
            self.quit = True
            return True
        else:
            return False

    def ls_k_thread(self):
        while 1:
            with pm.Listener(on_click=self.on_click) as pmlistener:
                pmlistener.join()

    def analyse_pic_thread(self):
        r = threading.Thread(target=self.ls_k_thread)
        r.setDaemon(True)
        r.start()

    def play(self):
        cap = cv.VideoCapture(self.file_path)
        out_win = "frame"
        cv.namedWindow(out_win, cv.WINDOW_NORMAL)
        if not self.window_size:
            cv.setWindowProperty(out_win, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        else:
            cv.resizeWindow(out_win, self.window_size[0], self.window_size[1])
        t = time.time()
        while time.time() - t < self.play_time and self.quit == False:
            ret, frame = cap.read()
            # print(frame)
            if frame is not None:
                cv.imshow(out_win, frame)
            if ret == False:
                cap = cv.VideoCapture(self.file_path)
        cap.release()
        cv.destroyAllWindows()
