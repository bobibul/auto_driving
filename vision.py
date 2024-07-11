import cv2
import numpy as np
import threading

"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
FORWARD_THRESHOLD = 0.3
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
FORWARD, LEFT, RIGHT = (0, 1, 2)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
HUE_THRESHOLD = ([20, 170], [30, 90], [110, 130], [20, 40])
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

REGION = np.array([
            [(15, CAMERA_HEIGHT), (CAMERA_WIDTH - 15, CAMERA_HEIGHT), (CAMERA_WIDTH - 170, 300), (170, 300)]
        ], dtype= np.int32)

DST_POINTS = np.array([
    [0, CAMERA_HEIGHT],        
    [CAMERA_WIDTH, CAMERA_HEIGHT], 
    [CAMERA_WIDTH, 0],       
    [0, 0]                  
], dtype=np.int32)

"""-----------------------------------------------------"""
# 안녕하세요

class Camera(threading.Thread):

    def __init__(self, camera_id):
        threading.Thread.__init__(self)
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        self.frame = None
        self.runing = True

        self.region = np.array([
            [(100, CAMERA_HEIGHT), (500, CAMERA_HEIGHT), (CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2)]
        ], dtype= np.int32)


    def run(self):
        while(self.runing):
            self.ret, self.frame = self.cap.read()
            
            if(not self.ret):
                print("프레임을 읽을 수 없습니다")
                break

            self.edge_detection(threshold=50)
            #self.color_detection(sample= 16, roi = RED)
            #self.color_detection(sample= 16, roi = GREEN)
            cv2.imshow(f'{self.camera_id}-original', self.frame)
            cv2.imshow(f'{self.camera_id}-bird_eye_view', self.bird_eye_view)
            #cv2.imshow(f'{self.camera_id}-canny', self.canny)   
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False

    def color_detection(self, sample = 0, roi = None):
        result = None
        self.replica = self.frame.copy()

        for color in (RED, YELLOW, GREEN):
            self.extract = self.color_filtering(roi)
            gray = cv2.cvtColor(self.extract, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=80,
                                    param1=200, param2=10, minRadius=40, maxRadius=100)
            
            if circles is not None:
                for circle in circles[0]:
                    center, count = (int(circle[0]), int(circle[1])), 0
                    print("Traffic Light: ", result)

                    h, s, v = cv2.split(self.hsv_img)

                    # Searching the surrounding pixels
                    for _ in range(sample):
                        x, y = int(center[1] - sample / 2), int(center[0] - sample / 2)
                        s_cond = s[x][y] > SATURATION
                        if color == RED:
                            h_cond = (h[x][y] < HUE_THRESHOLD[color][0]) | (h[x][y] > HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count
                        else:
                            h_cond = (h[x][y] > HUE_THRESHOLD[color][0]) & (h[x][y] < HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count

                    if count > sample / 2:
                        result = COLOR[color]
                        cv2.circle(self.replica, center, int(circle[2]), (0, 0, 255), 2)

            if result is not None:
                print("Traffic Light: ", result)

        return result
    
    def color_filtering(self, roi=None):
        self.row, self.col, self.dim = self.frame.shape
        self.hsv_img = cv2.cvtColor(self.frame.copy(), cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(self.hsv_img)

        s_cond = s > SATURATION
        if roi == RED:
            h_cond = (h < HUE_THRESHOLD[roi][0]) | (h > HUE_THRESHOLD[roi][1])
        else:
            h_cond = (h > HUE_THRESHOLD[roi][0]) & (h < HUE_THRESHOLD[roi][1])

        v[~h_cond], v[~s_cond] = 0, 0
        hsv_image = cv2.merge([h, s, v])
        result = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        return result
    
    def edge_detection(self, threshold = 0):
        self.replica = self.frame.copy()
        self.bird_eye_view = self.bird_eye_view_trans()
        gray_scale = cv2.cvtColor(self.bird_eye_view, cv2.COLOR_BGR2GRAY)
        hist = cv2.equalizeHist(gray_scale)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        dst = cv2.dilate(cv2.erode(hist, kernel), kernel)
        self.blurring = cv2.GaussianBlur(dst, (5,5), 0)
        self.canny = cv2.Canny(self.blurring, 100, 200)
        lines = cv2.HoughLinesP(self.canny, 1, np.pi/180, threshold, lines=np.array([]),
                                   minLineLength=50, maxLineGap=5)
        self.new_line_detection(lines)

    def bird_eye_view_trans(self):
        M = cv2.getPerspectiveTransform(REGION.astype(np.float32), DST_POINTS.astype(np.float32))
        bird_eye_view = cv2.warpPerspective(self.frame, M, (CAMERA_WIDTH, CAMERA_HEIGHT))
        return bird_eye_view

    def new_line_detection(self, lines):
        if(lines is not None):
            left_lines = []
            right_lines = []
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if(np.abs(x1 - x2) < np.abs(y1 - y2)):
                        if(x1 < CAMERA_WIDTH / 2):
                            cv2.line(self.bird_eye_view, (x1, y1), (x2, y2), (0, 0, 255), 3)
                            left_lines.append(x1)
                        else:
                            cv2.line(self.bird_eye_view, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            right_lines.append(x1)
                
            if(len(left_lines) == 0):
                print("turn left")

            elif(len(right_lines) == 0):
                print("turn right")

            else:
                angle = np.mean(left_lines) - (CAMERA_WIDTH- np.mean(right_lines))
                if(angle < 0):
                    print(f"turn left {angle}")
                else:
                    print(f"turn right {angle}")
        
   



    

