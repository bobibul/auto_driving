import ginyung as fl
import serial
import time
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'
LIDAR_PORT = '/dev/ttyUSB1'
TASK = 2


if __name__ == "__main__":
    
    env_line = fl.libCAMERA(cam_num = 0)
    env_light = fl.libCAMERA(cam_num = 2)
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(port = LIDAR_PORT)
    lidar.stop()

    env_line.obstacle1 = False
    sequence = 0
    dist = 10000
    result = []
    
    env_light.cv2imshow()
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas = False)):
        dist = scan[0][2]

        for j in range(TASK):
            env_line.run()
            
            if(dist < 1000 and sequence != 3):
                match sequence:
                    case 0:
                        data = "x"
                        env_line.obstacle1 = True
                        sequence = 1
                        startTime = time.time()

                    case 1:
                        data = "y"
                        
                    case 2:
                        if(time.time() - startTime >= 5):
                            data = "z"
                            env_line.obstacle1 = False
                            sequence = 3
                            TASK = 10000000

            else:
                if(env_line.obstacle1):
                    data = "o" + str(int(5 * env_line.cam_steer)) + "\n"
                else:
                    data = "o" + str(int(-5 * env_line.cam_steer)) + "\n"

                if sequence == 1:
                    sequence = 2

                elif sequence == 3:
                    env_light.cv2imshow()
                    


            print(data)
            print(dist)
            ser.write(data.encode())

