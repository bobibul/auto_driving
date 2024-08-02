from rplidar import RPLidar
import threading

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
    lidar_flag = False
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            adjusted_angle = angle % 360
            
            if adjusted_angle > 180:
                adjusted_angle -= 360 

            if -15 <= adjusted_angle <= 15:
                print(distance)
                