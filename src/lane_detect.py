import cv2
import numpy as np
from vision import Camera

"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640
THRESHOLD = 50

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

class Lane(Camera):

    def lane_detect(self):
        while(self.runing):
            self.ret, self.frame = self.cap.read()
            
            if(not self.ret):
                print("프레임을 읽을 수 없습니다")
                break

            self.edge_detection()

            cv2.imshow(f'{self.camera_id}-original', self.frame)
            cv2.imshow(f'{self.camera_id}-bird_eye_view', self.bird_eye_view)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

    def edge_detection(self):
        self.replica = self.frame.copy()
        self.bird_eye_view = self.bird_eye_view_trans()
        gray_scale = cv2.cvtColor(self.bird_eye_view, cv2.COLOR_BGR2GRAY)
        hist = cv2.equalizeHist(gray_scale)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        dst = cv2.dilate(cv2.erode(hist, kernel), kernel)
        self.blurring = cv2.GaussianBlur(dst, (5,5), 0)
        self.canny = cv2.Canny(self.blurring, 100, 200)
        lines = cv2.HoughLinesP(self.canny, 1, np.pi/180, THRESHOLD, lines=np.array([]),
                                   minLineLength=50, maxLineGap=5)
        self.line_detection(lines)

    def bird_eye_view_trans(self):
        M = cv2.getPerspectiveTransform(REGION.astype(np.float32), DST_POINTS.astype(np.float32))
        bird_eye_view = cv2.warpPerspective(self.frame, M, (CAMERA_WIDTH, CAMERA_HEIGHT))
        return bird_eye_view

    def line_detection(self, lines):
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
        
    