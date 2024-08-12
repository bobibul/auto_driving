import ginyung as fl
import time
import cv2

if __name__ == "__main__":
    count = 0
    startTime = time.time()
    env_light = fl.libCAMERA(cam_num = 0)
    x, y, w, h = 250, 50, 200, 100
    while(1):
        env_light.captureimage(x,y,w,h)
        if(time.time() - startTime > 0.2):
            count += 1
            if(count % 4 == 0):
                cv2.imwrite("C:/Users/sunmu/OneDrive/Desktop/auto_driving/cnnimages/test/2_red/" + str(count) + ".jpg" , env_light.roi)
            else:
                cv2.imwrite("C:/Users/sunmu/OneDrive/Desktop/auto_driving/cnnimages/train/2_red/" + str(count) + ".jpg" , env_light.roi)
            startTime = time.time()