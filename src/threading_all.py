import ginyung as fl
import serial
import time
import threading
from rplidar import RPLidar
import queue

def lidar_start(lidar, lidar_queue):
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            adjusted_angle = angle % 360
            
            if adjusted_angle > 180:
                adjusted_angle -= 360 

            if -15 <= adjusted_angle <= 15:
                lidar_queue.put(distance)
                break

def line_start(env_line):
    frame1 = env_line.jinhyuk_set()  
    env_line.run(frame1)