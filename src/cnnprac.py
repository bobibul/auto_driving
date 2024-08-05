import ginyung as fl
import time
import cv2
from tensorflow.keras.models import load_model
env_light = fl.libCAMERA(cam_num = 0)

if __name__ == "__main__":
    model = load_model('src/cnnmodel_1.h5')
    count = 0
    startTime = time.time()
    while(1):
        print(1)
        print(env_light.cnn_detection(model))