#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class SlidingWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None

    def apply(self, binary_warped):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
        midpoint = np.int32(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = np.int32(binary_warped.shape[0] // nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        margin = 100
        minpix = 50

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        if len(leftx) > 0 and len(lefty) > 0:
            self.left_fit = np.polyfit(lefty, leftx, 2)
        else:
            self.left_fit = None

        if len(rightx) > 0 and len(righty) > 0:
            self.right_fit = np.polyfit(righty, rightx, 2)
        else:
            self.right_fit = None

        return self.left_fit, self.right_fit

class Lane_sub:
    def __init__(self):
        rospy.init_node("lane_steering_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        self.sliding = SlidingWindow()

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

        self.action(bin_img)

        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

        # 시각화 (디버깅용)
        cv2.imshow("Warped View", warped_img)
        cv2.imshow("Binary Lane", bin_img)
        cv2.waitKey(1)

    def action(self, bin_img):
        left_fit, right_fit = self.sliding.apply(bin_img)

        if left_fit is not None and right_fit is not None:
            self.setSteeringinCurve(left_fit, right_fit)
            self.speed_msg.data = 1000  # 곡선 구간
        else:
            self.setSteeringinStraight(bin_img)
            self.speed_msg.data = 500   # 직선 구간

    def setSteeringinStraight(self, bin_img):
        histogram = np.sum(bin_img[self.height // 2:, :], axis=0)
        midpoint = self.width // 2
        if np.any(histogram > 50):
            lane_center = np.mean(np.where(histogram > 50))
        else:
            lane_center = midpoint

        offset = (lane_center - midpoint) / midpoint
        steer = 0.5 + offset * 0.4
        steer = np.clip(steer, 0.0, 1.0)
        self.steer_msg.data = steer

    def setSteeringinCurve(self, left_fit, right_fit):
        y_eval = int(self.height * 0.6)
        left_x = np.polyval(left_fit, y_eval)
        right_x = np.polyval(right_fit, y_eval)
        lane_center = (left_x + right_x) / 2
        img_center = self.width / 2
        offset = (lane_center - img_center) / img_center

        a = (left_fit[0] + right_fit[0]) / 2.0  # 곡률 정보
        curvature_threshold = 2e-4
        steer_base = 0.5 + offset * 0.8

        if abs(a) > curvature_threshold:
            steer_base += np.sign(offset) * abs(a) * 18

        self.steer_msg.data = np.clip(steer_base, 0.0, 1.0)

if __name__ == "__main__":
    try:
        Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
