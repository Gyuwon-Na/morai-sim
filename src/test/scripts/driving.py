#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
import sliding_window as sliding_window

class AutonomousDriving:
    def __init__(self):
        self.sliding = sliding_window.SlidingWindow()
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000 # Default speed

    def action(self, width, height, bin_img):
        self.width = width
        self.height = height
        left_fit, right_fit = self.sliding.apply(bin_img)

        if left_fit is not None and right_fit is not None:
            self.setSteeringinCurve(left_fit, right_fit)
        else:
            self.setSteeringinStraight(bin_img)
        
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

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


