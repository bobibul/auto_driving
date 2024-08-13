import tensorflow as tf
import cv2
import ginyung as fl

model_path = "src/my_model.h5"
model = tf.keras.models.load_model(model_path)
env_light = fl.libCAMERA(cam_num = 1)
x, y, w, h = 250, 80, 200, 140
cv2.namedWindow("cnn", cv2.WINDOW_NORMAL)
while(1):
    res = env_light.cnn_detection(model,x,y,w,h)
    print(res)
    




