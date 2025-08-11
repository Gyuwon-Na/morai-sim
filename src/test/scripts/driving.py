#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import numpy as np
import sliding_window as sliding_window
import traffic as traffic_sub
import cv2

class AutonomousDriving:
    def __init__(self):
        self.sliding = sliding_window.SlidingWindow()
        self.traffic_sub = traffic_sub.Traffic_Sub()
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 1000 # Default speed

        self.stop_line_detected = False
        self.stop_line_distance = float('inf')
        self.pixel_to_meter = 0.1
        self.last_valid_distance = float('inf')  # 마지막으로 유효했던 거리 저장
        self.camera_offset = 0.2 

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


    def traffic_signal(self):
        signal = self.traffic_sub.traffic_signal
        # print(self.stop_line_distance, "m away")
        if signal == 1:  # 빨간불
            if self.stop_line_detected and self.stop_line_distance <= 0.3:
                self.speed_msg.data = 0  # 정지선 0.3m 이내에서만 정지
            elif self.stop_line_detected and self.stop_line_distance <= 1.0:
                self.speed_msg.data = 300  # 정지선 근처에서 감속
            else:
                self.speed_msg.data = 500
        elif signal == 4:
            self.speed_msg.data = 300
        elif signal == 16 or signal == 33:
            self.speed_msg.data = 1000


    def detect_stop_line(self, bin_img):
        """정지선을 검출하고 거리를 계산 - 개선된 버전"""
        # 더 넓은 ROI 영역 사용 (이미지 하단 절반)
        roi_height = self.height // 2
        roi = bin_img[self.height - roi_height:, :]
        
        # 여러 크기의 커널로 수평선 검출 시도
        kernels = [
            np.ones((2, 10), np.uint8),  # 작은 커널
            np.ones((3, 15), np.uint8),  # 중간 커널
            np.ones((4, 20), np.uint8)   # 큰 커널
        ]
        
        best_detection = None
        best_score = 0
        
        for kernel in kernels:
            horizontal_lines = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel)
            horizontal_histogram = np.sum(horizontal_lines, axis=1)
            
            # 다양한 임계값으로 시도
            thresholds = [
                self.width * 0.4 * 255,  # 40%
                self.width * 0.5 * 255,  # 50%
                self.width * 0.6 * 255   # 60%
            ]
            
            for threshold in thresholds:
                stop_line_rows = np.where(horizontal_histogram > threshold)[0]
                
                if len(stop_line_rows) > 0:
                    score = np.max(horizontal_histogram[stop_line_rows])
                    if score > best_score:
                        best_score = score
                        best_detection = stop_line_rows
        
        if best_detection is not None:
            # 가장 가까운 정지선의 위치 (가장 아래쪽)
            closest_stop_line_row = best_detection[-1]
            # 실제 이미지에서의 Y 좌표
            actual_y = self.height - roi_height + closest_stop_line_row
            
            # 차량으로부터의 거리 계산
            pixels_to_bottom = self.height - actual_y
            self.stop_line_distance = pixels_to_bottom * self.pixel_to_meter
            self.stop_line_detected = True
            self.last_valid_distance = self.stop_line_distance  # 유효한 거리 저장

            # print(f"Pixels from bottom: {pixels_to_bottom}, Distance: {self.stop_line_distance:.3f}m")
        else:
            # 정지선이 검출되지 않았지만 이전에 가까운 거리에서 검출되었다면
            if self.last_valid_distance < 1.0:
                # 카메라에서는 안 보이지만 차량 전면은 정지선 근처
                estimated_distance = self.last_valid_distance - self.camera_offset
                if estimated_distance <= 0.0:
                    self.stop_line_distance = self.camera_offset  # 카메라 오프셋만큼 떨어져 있음
                else:
                    self.stop_line_distance = self.last_valid_distance
                self.stop_line_detected = True
                # print(f"Stop line not visible but estimated vehicle front distance: {estimated_distance:.3f}m")
            else:
                self.stop_line_detected = False
                self.stop_line_distance = float('inf')
                # print("No stop line detected")  

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

