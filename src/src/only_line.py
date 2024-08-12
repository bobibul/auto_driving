import ginyung as fl
import serial
import time
from rplidar import RPLidar
import cv2


PORT = 'COM8'
LIDAR_PORT = 'COM5'
TASK = 1000000000

if __name__ == "__main__":
    
    env_line = fl.libCAMERA(cam_num = 0)
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(port = LIDAR_PORT)
    lidar.stop()
    data = input("s 를 입력하여 start : ")
    ser.write(data.encode())

    for i, scan in enumerate(lidar.iter_scans(max_buf_meas = False)):
        for j in range(TASK):

            env_line.run()
            data = "o" + str(int(-5 * env_line.cam_steer)) + "\n"


            print(data)
            ser.write(data.encode())