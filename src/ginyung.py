"""
-------------------------------------------------------------------
  FILE NAME: Function_Library.py
  Copyright: Sungkyunkwan University, Automation Lab.
-------------------------------------------------------------------
  This file is included library class for below subject.
  1) Arduino
  2) LiDAR
  3) Camera
-------------------------------------------------------------------
  Authors: Jonghun Kim, YoungSoo Do, SungBhin Oh, HyeongKeun Hong

  Generated: 2022-11-10
  Revised: 2022-11-18
-------------------------------------------------------------------
  If you find some wrong code, plz contact me(Main Author: Jonghun Kim).
-------------------------------------------------------------------
  You should never modify this file during workshop exercise.
-------------------------------------------------------------------
"""

import sys
import cv2                           # pip install opencv
import time
import serial                        # pip install serial
import numpy as np                   # pip install numpy
import matplotlib.pyplot as plt      # pip install matplotlib
from rplidar import RPLidar          # pip install rplidar-roboticia
from math import *
import datetime
import os

np.set_printoptions(threshold=sys.maxsize, linewidth=150)

"""------------------Arduino Variable------------------"""
WAIT_TIME = 2
"""----------------------------------------------------"""


"""-------------------LIDAR Variable-------------------"""
SCAN_TYPE = "normal"
SAMPLE_RATE = 10
MAX_BUFFER_SIZE = 3000
MIN_DISTANCE = 0
"""----------------------------------------------------"""


"""--------------Computer Vision Variable--------------"""
NULL = 0
VARIANCE = 30
SATURATION = 150
FORWARD_THRESHOLD = 0.3
RED, GREEN, BLUE, YELLOW = (0, 1, 2, 3)
FORWARD, LEFT, RIGHT = (0, 1, 2)
COLOR = ("RED", "GREEN", "BLUE", "YELLOW")
DIRECTION = ("FORWARD", "LEFT", "RIGHT")
HUE_THRESHOLD = ([4, 176], [40, 80], [110, 130], [20, 40])
CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640
# REGION = np.array([
#             [(15, CAMERA_HEIGHT), (CAMERA_WIDTH - 15, CAMERA_HEIGHT), (CAMERA_WIDTH - 170, 300), (170, 300)]
            
#         ], dtype= np.int32)
REGION = np.array([
             [(241 , 461), (618, 478), (470, 289), (245, 291)]
            
        ], dtype= np.int32)
DST_POINTS = np.array([
    [0, CAMERA_HEIGHT],        
    [CAMERA_WIDTH, CAMERA_HEIGHT], 
    [CAMERA_WIDTH, 0],       
    [0, 0]                  
], dtype=np.int32)
"""-----------------------------------------------------"""


"""
-------------------------------------------------------------------
  CLASS PURPOSE: Camera Sensor Exercise Library
  Author: Jonghun Kim
  Revised: 2022-11-12
-------------------------------------------------------------------
"""
# noinspection PyMethodMayBeStatic
class libCAMERA(object):
    def __init__(self, cam_num):
        self.capnum = 0
        self.row, self.col, self.dim = (0, 0, 0)
        self.nothing_flag = False
        self.cam_flag = False
        self.cap = cv2.VideoCapture(cam_num)

    def loop_break(self):
        if cv2.waitKey(10) & 0xFF == ord('q'):
            print("Camera Reading is ended.")
            return True
        else:
            return False
    
    def jinhyuk_set(self):
        ret, frame = self.cap.read()
        return frame


    def camera_read(self, cap1, cap2=None):
        result, capset = [], [cap1, cap2]

        for idx in range(0, self.capnum):
            ret, frame = capset[idx].read()
            result.extend([ret, frame])

        return result
    
    def initial_setting(self, cam0port=0, cam1port=1, capnum=1):
        # OpenCV Initial Setting
        print("OpenCV Version:", cv2.__version__)
        channel0 = None
        channel1 = None
        self.capnum = capnum

        if capnum == 1:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")
        elif capnum == 2:
            channel0 = cv2.VideoCapture(cv2.CAP_DSHOW + cam0port)
            if channel0.isOpened():
                print("Camera Channel0 is enabled!")

            channel1 = cv2.VideoCapture(cv2.CAP_DSHOW + cam1port)
            if channel1.isOpened():
                print("Camera Channel1 is enabled!")

        return channel0, channel1
    

    def image_show(self, frame0, frame1=None):
        if frame1 is None:
            cv2.imshow('frame0', frame0)
        else:
            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)

    ### Save image

    def save_image(self, frame):
        # images 디렉토리가 존재하지 않으면 생성
        if not os.path.exists('images'):
            os.makedirs('images')

        # 현재 시간을 파일명에 포함시켜 저장
        filename = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
        filepath = os.path.join('images', filename)
        cv2.imwrite(filepath, frame)
        print(f"Image saved as {filepath}")


    ### OBJECT Detection

    def detection(self, img, model):
        result = model.predict(source=img, save=False)
        try:
            obj_list = ['red','crosswalk', 'green']
            box_color_list = [(50,50,255), (0,204,0), (194,153,255), (255,204,51), (255,102,204), (0,153,255)]
            det_result_obj = []
            det_result_size = []
            det_result_coord = result[0].boxes.xyxy.tolist()

            for i in range(len(result[0].boxes.cls.tolist())):
                x1 = int(result[0].boxes.xyxy.tolist()[i][0])
                x2 = int(result[0].boxes.xyxy.tolist()[i][2])
                y1 = int(result[0].boxes.xyxy.tolist()[i][1])
                y2 = int(result[0].boxes.xyxy.tolist()[i][3])
                det_result_obj.append(int(result[0].boxes.cls[i]))
                det_result_size.append(round((x2-x1)*(y2-y1)/(img.shape[0]*img.shape[1])*100,3))

                box_color = box_color_list[int(result[0].boxes.cls[i])]
                text_color = (0, 0, 0)
                cv2.rectangle(img, (x1, y1), (x2, y2), box_color, 2)
                txt_loc = (max(x1+2, 0), max(y1+2, 0))
                txt = obj_list[int(result[0].boxes.cls[i])]
                img_h, img_w, _ = img.shape
                if txt_loc[0] >= img_w or txt_loc[1] >= img_h:
                    cv2.imshow('result', img)
                    cv2.waitKey(1)
                    return [det_result_obj, det_result_size, det_result_coord]
                margin = 3
                size = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)
                w = size[0][0] + margin * 2
                h = size[0][1] + margin * 2
                cv2.rectangle(img, (x1-1, y1-1-h), (x1+w, y1), box_color, -1)
                cv2.putText(img, txt, (x1+margin, y1-margin-2), cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, lineType=cv2.LINE_AA)
            cv2.imshow('result', img)
            cv2.waitKey(1)
            return [det_result_obj, det_result_size, det_result_coord]
        except Exception:
            return [0]




    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        # Define range of blend color in HSV
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([160, 40, 255])
        
        # Slightly greenish white color range in HSV
        # greenish_white_lower = np.array([60, 0, 200])
        # greenish_white_upper = np.array([120, 150, 255])
    
        # Threshold the HSV image to get only white colors
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        # greenish_white_mask = cv2.inRange(hsv, greenish_white_lower, greenish_white_upper)
    
        # Combine the masks
        # combined_mask = cv2.bitwise_or(white_mask, greenish_white_mask)
    
        # Threshold the HSV image to get blend colors
        combined_color = cv2.bitwise_and(img, img, mask=white_mask)
        return combined_color

    def img_warp(self, img, white_color):
        # shape of img
        self.img_x, self.img_y = img.shape[1], img.shape[0]
        # print(f'self.img_x:{self.img_x}, self.img_y:{self.img_y}')
        # img_size = [640, 480]

        # ROI
        # src_side_offset = [round(self.img_x * 0.046875), round(self.img_y * 0.208)]
        # src_center_offset = [round(self.img_x * 0.14), round(self.img_y * 0.083)]
        # src = np.float32(
        #     [
        #         [src_side_offset[0], self.img_y - src_side_offset[1]],
        #         [
        #             self.img_x / 2 - src_center_offset[0],
        #             self.img_y / 2 + src_center_offset[1],
        #         ],
        #         [
        #             self.img_x / 2 + src_center_offset[0],
        #             self.img_y / 2 + src_center_offset[1],
        #         ],
        #         [self.img_x - src_side_offset[0], self.img_y - src_side_offset[1]],
        #     ]
        # )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        # dst_offset = [round(self.img_x * 0.125), 0]
        # # offset x 값이 작아질 수록 dst box width 증가합니다.
        # dst = np.float32(
        #     [
        #         [dst_offset[0], self.img_y],
        #         [dst_offset[0], 0],
        #         [self.img_x - dst_offset[0], 0],
        #         [self.img_x - dst_offset[0], self.img_y],
        #     ]
        # )

        # M = cv2.getPerspectiveTransform(np.array([
        #     [(2, 208), (318, 204), (295, 103), (9, 104)]
        # ], dtype= np.int32).astype(np.float32), DST_POINTS.astype(np.float32))
        M = cv2.getPerspectiveTransform(REGION.astype(np.float32), DST_POINTS.astype(np.float32))
        white_line = cv2.warpPerspective(white_color, M, (CAMERA_WIDTH, CAMERA_HEIGHT))
        # # find perspective matrix
        # matrix = cv2.getPerspectiveTransform(src, dst)
        # matrix_inv = cv2.getPerspectiveTransform(dst, src)
        # white_line = cv2.warpPerspective(white_color, matrix, [self.img_x, self.img_y])
        return white_line
 
    
    def img_binary(self, white_line):
        bin = cv2.cvtColor(white_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line # 480 * 640

    def detect_nothing(self):
        # self.nothing_left_x_base = round(self.img_x * 0.140625)
        # self.nothing_right_x_base = self.img_x - round(self.img_x * 0.140625)

        # self.nothing_pixel_left_x = np.array(
        #     np.zeros(self.nwindows) + round(self.img_x * 0.140625)
        # )

        # self.nothing_pixel_right_x = np.array(
        #     np.zeros(self.nwindows) + self.img_x - round(self.img_x * 0.140625)
        # )

        # self.nothing_pixel_y = np.array(
        #     [round(self.window_height / 2) * index for index in range(0, self.nwindows)]
        # )
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
    def window_search(self, binary_line): # 변수 : 윈도우 개수, 마진, 곱하는 상수들 변수
        # histogram을 생성합니다.
        # y축 기준 절반 아래 부분만을 사용하여 x축 기준 픽셀의 분포를 구합니다.
        bottom_half_y = binary_line.shape[0] / 2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스를 반환합니다.

        right_x_base = np.argmax(histogram[:])
        # show histogram
        # plt.hist(histogram)
        # plt.show()

        if right_x_base == 0:
            right_x_current = self.nothing_right_x_base
        else:
            right_x_current = right_x_base

        # conver 1 to 255 img
        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        ## window parameter
        # 적절한 윈도우의 개수를 지정합니다.
        nwindows = self.nwindows
        # 개수가 너무 적으면 정확하게 차선을 찾기 힘듭니다.
        # 개수가 너무 많으면 연산량이 증가하여 시간이 오래 걸립니다.
        window_height = self.window_height
        # 윈도우의 너비를 지정합니다. 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정합니다.
        # margin = 80
        margin = 45
        # 탐색할 최소 픽셀의 개수를 지정합니다.
        min_pix = min_pix = round((margin * 2 * window_height) * 0.04)

        # lane_pixel = white color's row and col return.

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        # pixel index를 담을 list를 만들어 줍니다.
        left_lane_idx = []
        right_lane_idx = []
        self.left_count = 0
        self.right_count = 0
        left_x = []
        left_y = []
        right_x = []
        right_y = []

        # Step through the windows one by one
        for window in range(nwindows):
            # window boundary를 지정합니다. (가로)
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            # print("check param : \n",window,win_y_low,win_y_high)

            # position 기준 window size. current left, right 기준으로 window number만큼 좌우, 상하 너비의 윈도우가 설정이 된다. 
       
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            # window 시각화입니다.
            if right_x_current != 0:
                cv2.rectangle(
                    out_img,
                    (win_x_right_low, win_y_low),
                    (win_x_right_high, win_y_high),
                    (0, 0, 255),
                    2,
                )

            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index를 저장합니다.
            good_right_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_high)
                & (lane_pixel_x >= win_x_right_low)
                & (lane_pixel_x < win_x_right_high)
            ).nonzero()[0]

            # Append these indices to the lists
            # if len(good_left_idx) > min_pix:
            #     left_lane_idx.append(good_left_idx)
            # if len(good_right_idx) > min_pix:
            #     right_lane_idx.append(good_right_idx)

            # left_lane_idx.append(good_left_idx)
            # right_lane_idx.append(good_right_idx)
        

            if len(good_right_idx) > min_pix*2:
                right_x1 = lane_pixel_x[good_right_idx]
                right_y1 = lane_pixel_y[good_right_idx]
                right_x.append(np.median(right_x1))
                right_y.append(np.median(right_y1))


            # window내 설정한 pixel개수 이상이 탐지되면, 픽셀들의 x 좌표 평균으로 업데이트 합니다.
            if len(good_right_idx) > min_pix:
                right_x_current = np.int32(np.median(lane_pixel_x[good_right_idx]))
                self.right_count += 1

        # # np.concatenate(array) => axis 0으로 차원 감소 시킵니다.(window개수로 감소)
        # left_lane_idx = np.concatenate(left_lane_idx)
        # right_lane_idx = np.concatenate(right_lane_idx)

        # # window 별 좌우 도로 픽셀 좌표입니다.
        # left_x = lane_pixel_x[left_lane_idx]
        # left_y = lane_pixel_y[left_lane_idx]
        # right_x = lane_pixel_x[right_lane_idx]
        # right_y = lane_pixel_y[right_lane_idx]

        # print(self.left_count,self.right_count)
        # 좌우 차선 별 2차함수 계수를 추정합니다.
        right_x = np.array(right_x)
        right_y = np.array(right_y)

        if len(right_x) == 0:
            right_x = self.nothing_pixel_right_x
            right_y = self.nothing_pixel_y
        else:
            pass
        right_fit = np.polyfit(right_y, right_x, 1, rcond=0.001)



        # 좌우 차선 별 추정할 y좌표입니다.

        plot_y = np.linspace(0, binary_line.shape[0] - 1, 20)
        # 좌우 차선 별 2차 곡선을 추정합니다.
        right_fit_x = right_fit[0] * plot_y + right_fit[1]

        right_slope = right_fit[0]
        # # window안의 lane을 black 처리합니다.
        # out_img[lane_pixel_y[left_lane_idx], lane_pixel_x[left_lane_idx]] = (0, 0, 0)
        # out_img[lane_pixel_y[right_lane_idx], lane_pixel_x[right_lane_idx]] = (0, 0, 0)



        # 양쪽 차선 및 중심 선 pixel 좌표(x,y)로 변환합니다.
        right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, right, right_x, right_y, right_slope

    def meter_per_pixel(self):
        # world_warp = np.array(
        #     [[97, 1610], [109, 1610], [109, 1606], [97, 1606]], np.float32
        # )
        world_warp = np.array(
            [[100, 1610], [102, 1610], [102, 1608], [100, 1608]], np.float32
        )
        meter_x = np.sum((world_warp[0] - world_warp[3]) ** 2)
        meter_y = np.sum((world_warp[0] - world_warp[1]) ** 2)
        meter_per_pix_x = meter_x / self.img_x
        meter_per_pix_y = meter_y / self.img_y
        return meter_per_pix_x, meter_per_pix_y



    def calc_vehicle_offset(self, sliding_window_img, right_x, right_y):
        # Args:
        # sliding_window_img (_type_): _description_
        # left_x (np.array): 왼쪽 차선 pixel x 값
        # left_y (np.array): 왼쪽 차선 pixel y 값
        # right_x (np.array): 오른쪽 차선 pixel x 값
        # right_y (np.array): 오른쪽 차선 pixel y 값
        # Returns:
        # float: 차선 중앙으로부터 이탈정도를 확인합니다. (왼쪽 -, 오른쪽 +)

        # 좌우 차선 별 2차함수 계수 추정합니다.
        right_fit = np.polyfit(right_y, right_x, 1)

        # Calculate vehicle center offset in pixels
        bottom_y = sliding_window_img.shape[0] - 1
        bottom_x_right = (
            right_fit[0] * bottom_y + right_fit[1]
        )
        vehicle_offset = (
            468 - bottom_x_right
        )
        # print(bottom_x_right)
        # Convert pixel offset to meter
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel()
        vehicle_offset *= meter_per_pix_x

        return vehicle_offset

    def cam_cal_steer(self, vehicle_offset, right_slope):
        err = np.arctan(right_slope)*180/np.pi
        cam_steer = - err/30 - vehicle_offset*3
        #print("err: ", err, "vehicle offset: ", vehicle_offset, "steer: ",cam_steer)
        return cam_steer



    def run(self, img):  # 기능의 순서를 배치하는 함수(run) 설정
        self.nwindows = 20
        self.window_height = np.int32(img.shape[0] / self.nwindows)
        
        blend_color = self.detect_color(img)
        blend_line = self.img_warp(img, blend_color)
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
        cv2.imshow("img", img)
        cv2.imshow("sliding_window_img", sliding_window_img)
        cv2.waitKey(1)
    