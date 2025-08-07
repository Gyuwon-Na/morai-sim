#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_steering_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000  # 기본 속도

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.height, self.width = img.shape[:2]

        # 차선 색상 범위(노란색, 흰색)
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([45, 255, 255])
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        filtered = cv2.bitwise_and(img, img, mask=combined_mask)

        # 투시 변환
        src = np.float32([
            [0, 420],
            [275, 260],
            [self.width-275, 260],
            [self.width, 420]
        ])
        dst = np.float32([
            [self.width//8, 480],
            [self.width//8, 0],
            [self.width//8*7, 0],
            [self.width//8*7, 480]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        warped_img = cv2.warpPerspective(filtered, M, (self.width, self.height))

        # 이진화
        gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray)
        bin_img[gray > 50] = 255

        # 선 검출
        edges = cv2.Canny(bin_img, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=15)

        warped_color = warped_img.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(warped_color, (x1, y1), (x2, y2), (0, 255, 0), 2)

        self.setCenter(bin_img)

        # 시각화
        cv2.imshow("Warped + Lines", warped_color)
        cv2.waitKey(1)


    def setCenter(self, bin_img):
        # 차선 중심 계산 (histogram 방식)
        histogram = np.sum(bin_img[self.height//2:, :], axis=0)
        midpoint = self.width // 2
        lane_center = np.mean(np.where(histogram > 50)) if np.any(histogram > 50) else midpoint

        # 조향 계산
        is_curve = self.isCurve(bin_img)
        print(is_curve)

        if is_curve is False:
            offset = (lane_center - midpoint) / midpoint
            steer = 0.5 + offset * 0.4                   
            steer = np.clip(steer, 0.0, 1.0)
        else:
            steer = 0.5

        # 메시지 퍼블리시
        self.steer_msg.data = steer
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)
        

    def isCurve(self, bin_img):
        """
        bin_img 에서 슬라이딩 윈도우로 좌/우 차선 픽셀을 찾고
        2차 다항식으로 피팅한 후, 계수 a 의 절댓값이 threshold 이상이면 커브로 판단.
        """
        # 1) nonzero 좌표
        nonzero = bin_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # 2) 히스토그램 기반 시작점
        histogram = np.sum(bin_img[self.height//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_current  = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint

        # 3) 슬라이딩 윈도우 파라미터
        nwindows = 9
        window_height = np.int(self.height // nwindows)
        margin = 100
        minpix = 50

        left_inds = []
        right_inds = []
        # 4) 윈도우마다 픽셀 인덱스 수집
        for w in range(nwindows):
            y_low  = self.height - (w+1)*window_height
            y_high = self.height -  w   *window_height
            x_l_low  = leftx_current  - margin
            x_l_high = leftx_current  + margin
            x_r_low  = rightx_current - margin
            x_r_high = rightx_current + margin

            good_l = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                      (nonzerox >= x_l_low) & (nonzerox < x_l_high)).nonzero()[0]
            good_r = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                      (nonzerox >= x_r_low) & (nonzerox < x_r_high)).nonzero()[0]

            left_inds.append(good_l)
            right_inds.append(good_r)

            if len(good_l) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_l]))
            if len(good_r) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_r]))

        # 5) 인덱스 합치기
        left_inds  = np.concatenate(left_inds)
        right_inds = np.concatenate(right_inds)

        # 6) 좌표 분리
        leftx  = nonzerox[left_inds]
        lefty  = nonzeroy[left_inds]
        rightx = nonzerox[right_inds]
        righty = nonzeroy[right_inds]

        # 7) 2차 다항식 피팅
        if len(leftx) < minpix or len(rightx) < minpix:
            return False  # 픽셀 부족하면 직선으로 간주

        left_fit  = np.polyfit(lefty,  leftx,  2)
        right_fit = np.polyfit(righty, rightx, 2)

        # 8) 곡률 계수(a) 평균
        a_coeff = (left_fit[0] + right_fit[0]) / 2.0

        # 9) threshold 판단 (실험적으로 1e-4 정도부터 커브로 봄)
        curve_thresh = 1e-4
        return abs(a_coeff) > curve_thresh



def main():
    try:
        Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
