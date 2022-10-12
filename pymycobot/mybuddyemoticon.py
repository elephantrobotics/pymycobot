# coding=utf-8

from __future__ import division
import cv2 as cv
import time
import threading


class Emoticon:
    def __init__(self, file_path: list = [], window_size: list = [], play_time=5) -> None:
        """API for playing emoticons

        Args:
            file_path(list): Absolute path of facial expression video.
            window_size(list): `[Length, width] `Size of the playback window (default is full screen).
            play_time: Playback duration. 5 seconds by default

        """
        self.__file_path = file_path
        self.__window_size = window_size
        self.__play_time = play_time
        self.quit = False
        self.stop_play = False
        
    @property
    def file_path(self):
        return self.__file_path
    
    def add_file_path(self, path):
        self.__file_path.append(path)

    @property
    def window_size(self):
        return self.__window_size
    
    @window_size.setter
    def window_size(self, data):
        self.__window_size = data

    @property
    def play_time(self):
        return self.__play_time
    
    @play_time.setter
    def play_time(self, data):
        self.__play_time = data
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.quit = True
            
    def stop(self):
        self.stop_play = True
        
    def start(self):
        self.stop_play = False
        
    def run(self):
        t = threading.Thread(target=self.play, daemon=True)
        t.start()

    def play(self):
        index = 0
        cap = cv.VideoCapture(self.__file_path[index])
        out_win = "frame"
        cv.namedWindow(out_win, cv.WINDOW_NORMAL)
        if not self.window_size:
            cv.setWindowProperty(out_win, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        else:
            cv.resizeWindow(out_win, self.window_size[0], self.window_size[1])
        t = time.time()
        cv.setMouseCallback(out_win, self.mouse_callback)
        while time.time() - t < self.play_time and self.quit == False:
            while self.stop_play:
                pass
            ret, frame = cap.read()
            # print(frame)
            if frame is not None:
                cv.imshow(out_win, frame)
            if cv.waitKey(1) & 0xFF == ord('q') or ret == False:
                index += 1
                if index >= len(self.__file_path):
                    index = 0
                cap = cv.VideoCapture(self.__file_path[index])
        cap.release()
        cv.destroyAllWindows()
