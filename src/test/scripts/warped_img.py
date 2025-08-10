#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import driving as driving

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_steering_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)

        self.bridge = CvBridge()
        self.driving = driving.AutonomousDriving()


    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.height, self.width = img.shape[:2]

        # 차선 색상 범위 설정
        yellow_lower = np.array([15, 128, 0])
        yellow_upper = np.array([40, 255, 255])
        white_lower = np.array([0, 0, 192])
        white_upper = np.array([179, 64, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        filtered = cv2.bitwise_and(img, img, mask=combined_mask)

        # 투시 변환
        x = self.width
        src = np.float32([[0, 420], [275, 260], [x - 275, 260], [x, 420]])
        dst = np.float32([[x // 8, 480], [x // 8, 0], [x // 8 * 7, 0], [x // 8 * 7, 480]])
        M = cv2.getPerspectiveTransform(src, dst)
        warped_img = cv2.warpPerspective(filtered, M, (self.width, self.height))

        # 그레이 & 이진화
        gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray)
        bin_img[gray > 50] = 255

        self.driving.action(self.width, self.height, bin_img)
        
        edges = cv2.Canny(bin_img, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=15)

        left_lines, right_lines = [], []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_center = (x1 + x2) // 2

                if x_center < self.width // 2:
                    left_lines.append(x_center)
                else:
                    right_lines.append(x_center)
                cv2.line(warped_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 시각화 (디버깅용)
        cv2.imshow("Warped View", warped_img)
        cv2.imshow("Binary Lane", bin_img)
        cv2.waitKey(1)
