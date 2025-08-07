#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
import sliding_window as sliding_window
import traffic as traffic_sub
from obstacle_avoidance import Obstacle

class AutonomousDriving:
    def __init__(self):
        self.sliding = sliding_window.SlidingWindow()
        self.traffic_sub = traffic_sub.Traffic_Sub()
        self.obstacle_avoidance = Obstacle()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000 # Default speed

    def action(self, width, height, bin_img):
        self.width = width
        self.height = height
        left_fit, right_fit = self.sliding.apply(bin_img)

        ## 미션 구간 범위 지정해두고 그때만 쓰게 하기
        # self.traffic_signal()

        # if self.obstacle_avoidance.detected:
        #     avoidance_degree = self.obstacle_avoidance.avoidance()
        #     self.steer_msg.data = avoidance_degree

        if left_fit is not None and right_fit is not None:
            self.setSteeringinCurve(left_fit, right_fit)
        else:
            self.setSteeringinStraight(bin_img)

        
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)


#########    직선구간에서 선 안잡힐 때      ##########

    # def action(self, width, height, bin_img):
    #     self.width = width
    #     self.height = height

    #     with warnings.catch_warnings(record=True) as w:
    #         warnings.simplefilter("always")  # 모든 경고를 기록하도록
    #         left_fit, right_fit = self.sliding.apply(bin_img)

    #         # RankWarning 발생 여부 체크
    #         rank_warning_occurred = any(item.category == np.RankWarning for item in w)

    #     if rank_warning_occurred:
    #         # 경고 발생 시 직선으로 판단
    #         self.setSteeringinStraight(bin_img)
    #     else:
    #         try:
    #             if left_fit is not None and right_fit is not None:
    #                 self.setSteeringinCurve(left_fit, right_fit)
    #             else:
    #                 self.setSteeringinStraight(bin_img)
    #         except Exception as e:
    #             self.setSteeringinStraight(bin_img)

    #     self.steer_pub.publish(self.steer_msg)
    #     self.speed_pub.publish(self.speed_msg)


    def traffic_signal(self):
        signal = self.traffic_sub.traffic_signal
        dist = 0 

        if signal == 1: # 빨간불
            if dist < 0.3:
                self.speed_msg.data = 0
            else:
                self.speed_msg.data = 300
        elif signal == 4:
            self.speed_msg.data = 300
        elif signal == 16 or signal == 33:
            self.speed_msg.data = 1000
            

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

    def setSteeringinRotary(self, bin_img):
        pass

