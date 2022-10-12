# coding=utf-8

from __future__ import division
import cv2 as cv
import time
import threading


class Emoticon:
    def __init__(self, file_path: list = [], window_size: list = []) -> None:
        """API for playing emoticons

        Args:
            file_path(list): `[[path, time]]`The absolute path of facial expression video and the length of time to play.Time in seconds.
            window_size(list): `[Length, width] `Size of the playback window (default is full screen).

        """
        self.__file_path = file_path
        self.__window_size = window_size
        self.quit = False
        self.stop_play = False
        
    @property
    def file_path(self):
        """Get Playfile List"""
        return self.__file_path
    
    def add_file_path(self, path_time: list):
        """Add Playback File"""
        self.__file_path.append(path_time)
        
    def del_file_path(self, index: int):
        """Delete the element with the specified subscript in the playlist list"""
        if index >= len(self.__file_path):
            raise IndexError("list index out of range")
        self.__file_path.pop(index)

    @property
    def window_size(self):
        return self.__window_size
    
    @window_size.setter
    def window_size(self, data):
        """Set playback window size"""
        self.__window_size = data
        
    def mouse_callback(self, event, x, y, flags, param):
        """Mouse click callback function"""
        if event == cv.EVENT_LBUTTONDOWN:
            self.quit = True
            
    def pause(self):
        """Pause playback"""
        self.stop_play = True
        
    def run(self):
        """Continue playing"""
        self.stop_play = False

    def play(self):
        index = 0
        cap = cv.VideoCapture(self.__file_path[index][0])
        out_win = "frame"
        cv.namedWindow(out_win, cv.WINDOW_NORMAL)
        if not self.window_size:
            cv.setWindowProperty(out_win, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        else:
            cv.resizeWindow(out_win, self.window_size[0], self.window_size[1])
        t = time.time()
        cv.setMouseCallback(out_win, self.mouse_callback)
        while True:
            while time.time() - t < self.__file_path[index][1]:
                if self.quit:
                    break
                while self.stop_play and self.quit == False:
                    pass
                ret, frame = cap.read()
                # print(frame)
                if frame is not None:
                    cv.imshow(out_win, frame)
                if cv.waitKey(1) & 0xFF == ord('q') or ret == False:
                    if time.time() - t >= self.__file_path[index][1]:
                        index += 1
                        if index >= len(self.__file_path):
                            index = 0
                    cap = cv.VideoCapture(self.__file_path[index][0])
            if self.quit:
                break
        cap.release()
        cv.destroyAllWindows()
        
    def start(self):
        """Start playing"""
        self.t = threading.Thread(target=self.play, daemon=True)
        self.t.start()
        
    def joint(self):
        """Wait for the thread playing the video to finish"""
        self.t.joint()
