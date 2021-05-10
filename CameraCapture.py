from threading import Thread
from queue import Queue, Empty

import cv2
from cv2 import VideoCapture
import numpy as np

class CameraCapture:
    def __init__(self, name):
        self.cap = VideoCapture(name)
        print("camera capture")
        self.q = Queue()

        t = Thread(target=self._reader)
        t.daemon = True
        t.start()

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except :
                    pass
            self.q.put(frame)

    def read(self):
        if self.q.empty():
            return False, np.array([[]])

        return True, self.q.get()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.cap.release()

    def get(self, args):
        return self.cap.get(args)