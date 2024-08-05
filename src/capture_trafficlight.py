import ginyung as fl
import time
import cv2
env_light = fl.libCAMERA(cam_num = 0)

if __name__ == "__main__":
    count = 0
    startTime = time.time()
    while(1):
        
        env_light.run()
        if(time.time() - startTime > 0.1):
            count += 1
            cv2.imwrite("/home/jinhyuk/auto_driving/cnnImage/" + str(count) + ".jpg" ,env_light.frame)
            startTime = time.time()