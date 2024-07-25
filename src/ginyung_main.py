from ultralytics import YOLO
import keyboard
import ginyung as fl
import serial
import time

PORT = '/dev/ttyUSB0'

if __name__ == "__main__":
    # Exercise Environment Setting
    env = fl.libCAMERA()
    ser = serial.Serial(PORT, 115200)

    # Camera Initial Setting
    ch0, ch1 = env.initial_setting(capnum=2)
    count = 0
    sum = 0
    while(True):
        frame = env.jinhyuk_set()
        env.run(frame)
        sum += env.cam_steer
        if(count >= 5):
            data = str(int(sum / 5 * 2000)) + "\n"
            ser.write(data.encode())
            print(f"send {data}")
            count = 0
            sum = 0
        else:
            count += 1
    
