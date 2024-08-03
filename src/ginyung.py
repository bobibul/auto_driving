
import sys
import cv2                           
import numpy as np                   
import matplotlib.pyplot as plt     
from rplidar import RPLidar          
from math import *

np.set_printoptions(threshold=sys.maxsize, linewidth=150)

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

REGION = np.array([
            [(207 , 479), (618, 478), (486, 337), (207, 340)]      
        ], dtype= np.int32)

REGION2 = np.array([
            [(214 , 358), (8, 354), (63, 281), (218, 287)]      
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
        self.cap = cv2.VideoCapture(cam_num)

    def jinhyuk_set(self):
        ret, frame = self.cap.read()
        return frame

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

        left_lane_idx = []
        right_lane_idx = []
        self.left_count = 0
        self.right_count = 0
        left_x = []
        left_y = []
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
        cam_steer = - err/30 - vehicle_offset*3
        return cam_steer


    def run(self, img): 
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