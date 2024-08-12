import serial
from rplidar import RPLidar
import time

PORT = 'COM8'
LIDAR_PORT = 'COM5'

if __name__ == "__main__":
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(port = LIDAR_PORT)
    lidar.stop()

    data = input("p 를 입력하여 start : ")
    ser.write(data.encode())

    data = input("아무 키나 입력하시오 : ")
    ser.write(data.encode())

    
    right_distance = 1000
    sttime = time.time()
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas = False)):
        for j in range(len(scan)):
            if(scan[j][1] < 91 and scan[j][1] > 89): 
                if(scan[j][2] < 2000):
                    data = "d" 
                print(scan[j][2]) 
            if(data == "d"): break      
        if(data == "d"): break
    ser.write(data.encode())