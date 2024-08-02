import ginyung as fl
import serial
import time
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'
LIDAR_PORT = '/dev/ttyUSB1'


if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(LIDAR_PORT)
    env_line.obstacle1 = False
    sequence = 0
    dist = 0

    while(True):
        lidar_flag = False
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                adjusted_angle = angle % 360
                
                if adjusted_angle > 180:
                    adjusted_angle -= 360 

                if -15 <= adjusted_angle <= 15:
                    dist = distance
                    lidar_flag = True
                    break

            if(lidar_flag):
                break


        frame1 = env_line.jinhyuk_set()  
        env_line.run(frame1)

        if(dist < 1000):
            match sequence:
                case 0:
                    data = "x"
                    env_line.obstacle1 = True
                    sequence = 1
                    startTime = time.time()

                case 1:
                    data = "y"
                    
                case 2:
                    if(time.time() - startTime >= 10):
                        data = "z"
                        env_line.obstacle1 = False
                        sequence = 3

        else:
            if(env_line.obstacle1):
                data = "o" + str(int(5 * env_line.cam_steer)) + "\n"
            else:
                data = "o" + str(int(-5 * env_line.cam_steer)) + "\n"

        print(dist)
        print(data)
        ser.write(data.encode())
