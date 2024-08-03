from rplidar import RPLidar
import threading
import numpy as np

PORT_NAME = '/dev/ttyUSB1'
lidar = RPLidar(PORT_NAME)

def lidar_start(lidar):
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            adjusted_angle = angle % 360
            
            if adjusted_angle > 180:
                adjusted_angle -= 360 

            if -15 <= adjusted_angle <= 15:
                print(distance)
                if(distance <= 1000):
                    print("turn")

if __name__ == '__main__':
    count = 0
    lidar_flag = False
    lidar._set_pwm(10)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            adjusted_angle = angle % 360
            
            if adjusted_angle > 180:
                adjusted_angle -= 360 

            if -15 <= adjusted_angle <= 15:
                print(distance)