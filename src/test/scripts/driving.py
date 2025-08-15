#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
from camera import SlidingWindow
from control import Traffic
from camera import DetectStopLine
import cv2
from enum import Enum

class Mission(Enum):
    STRAIGHT = 0
    CORNER = 1

class AutonomousDriving:
    def __init__(self):
        self.sliding = SlidingWindow.SlidingWindow()
        self.stop_line_detector = DetectStopLine.StopLine()
        self.traffic_sub = Traffic.Traffic()
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000 # Default speed
        self.mission_flag = 0


    def action(self, width, height, bin_img):
        self.width = width
        self.height = height
        left_fit, right_fit = self.sliding.apply(bin_img)


        # self.detect_stop_line(bin_img)
        # self.traffic_signal()

        try:
            if left_fit is not None and right_fit is not None:
                self.setSteeringinCurve(left_fit, right_fit)
            else:
                self.setSteeringinStraight(bin_img)
        except Exception as e:
            pass
        
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)


    # def traffic_signal(self):
    #     signal = self.traffic_sub.traffic_signal
    #     # print(self.stop_line_distance, "m away")
    #     if signal == 1:  # 빨간불
    #         if self.stop_line_detected and self.stop_line_distance <= 0.3:
    #             self.speed_msg.data = 0  # 정지선 0.3m 이내에서만 정지
    #         elif self.stop_line_detected and self.stop_line_distance <= 1.0:
    #             self.speed_msg.data = 300  # 정지선 근처에서 감속
    #         else:
    #             self.speed_msg.data = 500
    #     elif signal == 4:
    #         self.speed_msg.data = 300
    #     elif signal == 16 or signal == 33:
    #         self.speed_msg.data = 1000


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
        curvature_threshold = 5e-4
        steer_base = 0.5 + offset * 0.8

        if abs(a) > curvature_threshold:
            steer_base += np.sign(offset) * abs(a) * 18

        self.steer_msg.data = np.clip(steer_base, 0.0, 1.0)