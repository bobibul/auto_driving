import cv2
import numpy as np
import threading

class Camera(threading.Thread):

    def __init__(self, camera_id):
        threading.Thread.__init__(self)
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        self.frame = None
        self.runing = True

    def stop(self):
        self.running = False



    

