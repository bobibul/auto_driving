from ultralytics import YOLO
import keyboard
import ginyung as fl
import serial
import time

PORT = '/dev/ttyUSB1'

if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    env_light = fl.libCAMERA(cam_num = 4)
    ser = serial.Serial(PORT, 115200)

    # Camera Initial Setting
    ch0, ch1 = env_line.initial_setting(capnum = 2)
    model = YOLO('src/best.pt')

    count = 0
    sum = 0
    while(True):
        frame1 = env_line.jinhyuk_set()
        frame2 = env_light.jinhyuk_set()

        env_light.run_yolo(frame2, model)
        env_line.run(frame1)

        data = "o" + str(int(env_line.cam_steer * -5)) + "\n"
        ser.write(data.encode())
        print(data)
        count = 0
        sum = 0