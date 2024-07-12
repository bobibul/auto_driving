import cv2
from camera_detect import Camera
from serial_connect import SerialConnect

"""--------------Computer Vision Variable--------------"""
NULL = 0
SATURATION = 150
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
SAMPLE = 16
HUE_THRESHOLD = ([20, 170], [30, 90], [110, 130], [20, 40])


"""-----------------------------------------------------"""

class TrafficLight(Camera, SerialConnect):

    def __init__(self, camera_id, port):
        Camera.__init__(self, camera_id)
        SerialConnect.__init__(self, port)
        print("신호등 카메라 연결되었습니다.")

        self.red_count = 0
        self.green_count = 0

    def traffic_light_detect(self):
        while(self.runing):
            self.ret, self.frame = self.cap.read()
            
            if(not self.ret):
                print("프레임을 읽을 수 없습니다")
                break

            if(self.red_count < 20):
                self.color_detection(roi = RED)

            elif(self.red_count >= 20 and self.green_count < 20):
                self.color_detection(roi = GREEN)

            cv2.imshow(f'{self.camera_id}-original', self.replica)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

    def color_detection(self, roi = None):
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
                    for _ in range(SAMPLE):
                        x, y = int(center[1] - SAMPLE / 2), int(center[0] - SAMPLE / 2)
                        s_cond = s[x][y] > SATURATION
                        if color == RED:
                            h_cond = (h[x][y] < HUE_THRESHOLD[color][0]) | (h[x][y] > HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count
                        else:
                            h_cond = (h[x][y] > HUE_THRESHOLD[color][0]) & (h[x][y] < HUE_THRESHOLD[color][1])
                            count += 1 if h_cond and s_cond else count

                    if count > SAMPLE / 2:
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