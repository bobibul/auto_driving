import ginyung as fl
import serial
import time
from rplidar import RPLidar
import tensorflow as tf

PORT = 'COM8'
LIDAR_PORT = 'COM5'
TASK = 2
ROI_X,ROI_Y,ROI_W,ROI_H = 250, 80, 200, 140


if __name__ == "__main__":
    
    env_line = fl.libCAMERA(cam_num = 0)
    env_light = fl.libCAMERA(cam_num = 1)
    ser = serial.Serial(PORT, 115200)
    lidar = RPLidar(port = LIDAR_PORT)
    lidar.stop()

    env_line.obstacle1 = False
    sequence = 0
    dist = 10000    
    result = []
    model_path = "src/my_model.h5"
    model = tf.keras.models.load_model(model_path)

    


    data = input("s 를 입력하여 장애물 회피 미션 수행 : ")
    ser.write(data.encode())
    
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas = False)):
        dist = scan[0][2]
        result = env_light.cv2_imshow(ROI_X,ROI_Y,ROI_W,ROI_H)

        for j in range(TASK):
            env_line.run()
            
            if(dist < 1300 and sequence != 3):
                if(sequence == 0):
                    data = "x"
                    env_line.obstacle1 = True
                    sequence = 1
                    startTime = time.time()

                elif(sequence == 1):
                    data = "y"

                elif(sequence == 2):
                    if(time.time() - startTime >= 5):
                        data = "z"
                        env_line.obstacle1 = False
                        sequence = 3
                        TASK = 10000000

            else:
                if(env_line.obstacle1):
                    data = "o" + str(int(6 * env_line.cam_steer)) + "\n"
                else:
                    data = "o" + str(int(-6 * env_line.cam_steer)) + "\n"

                if sequence == 1:
                    sequence = 2

                elif sequence == 3:
                    result = env_light.cnn_detection(model,ROI_X,ROI_Y,ROI_W,ROI_H)
                    if(result > 0.999) : data = "r"
                    elif(result < 0.001) : data = "g"
                    


            print(data)
            print(dist)
            ser.write(data.encode())

