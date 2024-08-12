
import sys
import cv2                           
import numpy as np                                
from math import *


np.set_printoptions(threshold=sys.maxsize, linewidth=150)

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

REGION = np.array([
            [(327 , 475), (616, 479), (480, 342), (327, 345)]      
        ], dtype= np.int32)

REGION2 = np.array([
            [(270 , 328), (83, 324), (48, 391), (250, 407)]      
        ], dtype= np.int32)

DST_POINTS = np.array([
    [0, CAMERA_HEIGHT],        
    [CAMERA_WIDTH, CAMERA_HEIGHT], 
    [CAMERA_WIDTH, 0],       
    [0, 0]                  
], dtype=np.int32)

class libCAMERA(object):
    def __init__(self, cam_num):
        self.capnum = 0
        self.row, self.col, self.dim = (0, 0, 0)
        self.nothing_flag = False
        self.cam_flag = False
        self.obstacle1 = False
        self.cap = cv2.VideoCapture(cam_num, cv2.CAP_DSHOW)

    
    def captureimage(self,x,y,w,h):
        _, self.frame = self.cap.read()
        self.roi = self.frame[y:y+h, x:x+w]
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.imshow('img', self.roi)
        cv2.waitKey(1)

    def detect_color(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([160, 40, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        combined_color = cv2.bitwise_and(img, img, mask=white_mask)
        return combined_color

    def img_warp(self, img, white_color):
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        if self.obstacle1 == False:
            M = cv2.getPerspectiveTransform(REGION.astype(np.float32), DST_POINTS.astype(np.float32))
        else:
            M = cv2.getPerspectiveTransform(REGION2.astype(np.float32), DST_POINTS.astype(np.float32))
        white_line = cv2.warpPerspective(white_color, M, (CAMERA_WIDTH, CAMERA_HEIGHT))
        return white_line
 
    
    def img_binary(self, white_line):
        bin = cv2.cvtColor(white_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line # 480 * 640

    def detect_nothing(self):
        self.nothing_left_x_base = round(self.img_x * 0.3)
        self.nothing_right_x_base = self.img_x - round(self.img_x * 0.3)

        self.nothing_pixel_left_x = np.array(
            np.zeros(self.nwindows) + round(self.img_x * 0.3)
        )

        self.nothing_pixel_right_x = np.array(
            np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.3)
        )

        self.nothing_pixel_y = np.array(
            [round(self.window_height / 2) * index for index in range(0, self.nwindows)]
        ) 
        
               
    def window_search(self, binary_line):
        bottom_half_y = binary_line.shape[0]/2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)

        if self.obstacle1 == False:
            reversed_index = np.argmax(histogram[::-1])
            right_x_base = len(histogram) - 1 - reversed_index
        else:
            right_x_base = np.argmax(histogram[::])

        if right_x_base == 0:
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base

        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        nwindows = self.nwindows
        window_height = self.window_height
        margin = 40
        min_pix = min_pix = round((margin * 2 * window_height) * 0.04)

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        self.left_count = 0
        self.right_count = 0
        
        right_x = []
        right_y = []

        for window in range(nwindows):
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
       
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            if right_x_current != 0:
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 0, 255),
                    2,
                )

            good_right_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_high)
                & (lane_pixel_x >= win_x_right_low)
                & (lane_pixel_x < win_x_right_high)
            ).nonzero()[0]

            if len(good_right_idx) > min_pix*2:
                right_x1 = lane_pixel_x[good_right_idx]
                right_y1 = lane_pixel_y[good_right_idx]
                right_x.append(np.median(right_x1))
                right_y.append(np.median(right_y1))

            if len(good_right_idx) > min_pix:
                right_x_current = np.int32(np.median(lane_pixel_x[good_right_idx]))
                self.right_count += 1

        right_x = np.array(right_x)
        right_y = np.array(right_y)

        if len(right_x) == 0:
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y
        else:
            pass
        right_fit = np.polyfit(right_y, right_x, 1, rcond=0.001)

        plot_y = np.linspace(0, binary_line.shape[0] - 1, 20)

        right_fit_x = right_fit[0] * plot_y + right_fit[1]
        right_slope = right_fit[0]

        right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, right, right_x, right_y, right_slope

    def meter_per_pixel(self):
        world_warp = np.array(
            [[100, 1610], [102, 1610], [102, 1608], [100, 1608]], np.float32
        )
        meter_x = np.sum((world_warp[0] - world_warp[3]) ** 2)
        meter_y = np.sum((world_warp[0] - world_warp[1]) ** 2)
        meter_per_pix_x = meter_x / self.img_x
        meter_per_pix_y = meter_y / self.img_y
        return meter_per_pix_x, meter_per_pix_y



    def calc_vehicle_offset(self, sliding_window_img, right_x, right_y):

        right_fit = np.polyfit(right_y, right_x, 1)
        bottom_y = sliding_window_img.shape[0] - 1
        bottom_x_right = (
            right_fit[0] * bottom_y + right_fit[1]
        )
        if self.obstacle1:
            vehicle_offset = (
                522 - bottom_x_right
            )
        else:
            vehicle_offset = (
                383 - bottom_x_right
            )

        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()
        vehicle_offset *= meter_per_pix_x

        return vehicle_offset

    def cam_cal_steer(self, vehicle_offset, right_slope):
        err = np.arctan(right_slope)*180/np.pi
        cam_steer = - err/80 - vehicle_offset * 2.2
        return cam_steer
    
    def cnn_detection(self, model,x,y,w,h):
        _, frame = self.cap.read()
        frame = frame[y:y+h, x:x+w]
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(rgb_frame, (h, w))  # 크기 조정
        normalized_frame = resized_frame / 255.0  # 정규화
        input_data = np.expand_dims(normalized_frame, axis= 0 )  # 배치와 채널 차원 추가
        res = model.predict(input_data)
        cv2.imshow('cnn', frame)
        cv2.waitKey(1)

        return res

    def run(self): 
        _, self.frame = self.cap.read()
        self.nwindows = 20
        self.window_height = np.int32(self.frame.shape[0] / self.nwindows)
        
        blend_color = self.detect_color(self.frame)
        blend_line = self.img_warp(self.frame, blend_color)
        binary_line = self.img_binary(blend_line)
        if self.nothing_flag == False:
            self.detect_nothing()
            self.nothing_flag = True
        (
         sliding_window_img,
         right,
         right_x,
         right_y,
         right_slope
        ) = self.window_search(binary_line)
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()
        vehicle_offset = self.calc_vehicle_offset(sliding_window_img, right_x, right_y)
        self.cam_steer = self.cam_cal_steer(vehicle_offset, right_slope)
        self.cam_flag = False
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.namedWindow("sliding_window_img", cv2.WINDOW_NORMAL)
        cv2.imshow("img", self.frame)
        cv2.imshow("sliding_window_img", sliding_window_img)
        cv2.waitKey(1)