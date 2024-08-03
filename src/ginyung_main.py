from ultralytics import YOLO
import keyboard
import ginyung as fl
import serial
import time
import cv2

PORT = '/dev/ttyUSB0'

if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    env_light = fl.libCAMERA(cam_num = 4)
    ser = serial.Serial(PORT, 115200)

    # Camera Initial Setting
    model = YOLO('src/best.pt')

    while(True):
        frame1 = env_line.jinhyuk_set()
        frame2 = env_light.jinhyuk_set()

        env_line.run(frame1)
        result = env_light.detection(frame2, model)

        if (0 in result[0]) : 
            data = "r"
            print("red")
        elif(2 in result[0]):
            data = "g"
            print("green")
        else:
            data = "o" + str(-5*int(env_line.cam_steer)) + "\n"

        ser.write(data.encode())