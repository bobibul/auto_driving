import ginyung as fl
import serial
import time

PORT = '/dev/ttyUSB0'


if __name__ == "__main__":
    # Exercise Environment Setting
    env_line = fl.libCAMERA(cam_num = 2)
    ser = serial.Serial(PORT, 115200)
    env_line.obstacle1 = False
    dist_count = 0
    left_or_right = 0
    obs_time = 0

    while(True):
        frame1 = env_line.jinhyuk_set()  
        dist = float(ser.readline().decode())
        print(dist)
        env_line.run(frame1)
        
        if(dist < 120 and left_or_right != 4):
            if(left_or_right == 0):
                data = "x"
                env_line.obstacle1 = True
                left_or_right = 1
                startTime = time.time()

            elif(left_or_right == 1):
                left_or_right = 2
                data = "y"

            elif(left_or_right == 3 and time.time() - startTime >= 10):
                data = "z"
                env_line.obstacle1 = False
                left_or_right = 4

        else:
            if(env_line.obstacle1):
                data = "o" + str(int(5 * env_line.cam_steer)) + "\n"
            else:
                data = "o" + str(int(-5 * env_line.cam_steer)) + "\n"

            if(left_or_right == 2):
                left_or_right = 3

        print(data)
        ser.write(data.encode())