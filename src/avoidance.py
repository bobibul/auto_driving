import ginyung as fl
from ultralytics import YOLO
import serial
import time
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'
LIDAR_PORT = '/dev/ttyUSB1'


if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    env_light = fl.libCAMERA(cam_num = 4)
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(port = LIDAR_PORT, baudrate = 115200)
    lidar.stop()
    model = YOLO('src/best.pt')
    env_line.obstacle1 = False
    sequence = 0

    for scan in lidar.iter_scans(max_buf_meas = False):
        for (_, angle, distance) in scan:
            adjusted_angle = angle % 360
            
            if adjusted_angle > 180:
                adjusted_angle -= 360 

            if -15 <= adjusted_angle <= 15:
                dist = distance
                break

        frame1 = env_line.jinhyuk_set()
        env_line.run(frame1)


        if(dist < 1200 and sequence != 3):
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

            if sequence == 1:
                sequence = 2

            elif sequence == 3:
                frame2 = env_light.jinhyuk_set()
                result = env_light.detection(frame2, model)
                if (0 in result[0]) : 
                    data = "r"
                    print("red")
                elif(2 in result[0]):
                    data = "g"
                    print("green")


        print(data)
        print(dist)
        ser.write(data.encode())
