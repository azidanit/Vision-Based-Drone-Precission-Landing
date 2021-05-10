from threading import Thread
from queue import Queue, Empty

import cv2
from cv2 import VideoCapture
import numpy as np

class CameraCapture:
    def __init__(self, name):
        self.cap = VideoCapture(name, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.is_running = True

        print("camera capture")
        self.q = Queue()

        t = Thread(target=self._reader)
        t.daemon = True
        t.start()

    def _reader(self):
        while self.is_running:
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
        self.is_running = False
        self.cap.release()

    def get(self, args):
        return self.cap.get(args)