#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2, rospy
import numpy as np

class StopLine:
    def __init__(self):
        self.stop_line_detected = False
        self.stop_line_distance = float('inf')
        self.pixel_to_meter = 0.1
        self.last_valid_distance = float('inf')  # 마지막으로 유효했던 거리 저장
        self.camera_offset = 0.2 
        self.stop_line_num = 0
        self.prev_stop_line_num = 0
        self.flag = False
        
        
    def detect(self, width, height, bin_img):
        """정지선을 검출하고 거리를 계산 - 개선된 버전"""
        # 더 넓은 ROI 영역 사용 (이미지 하단 절반)
        roi_height = height // 2
        roi = bin_img[height - roi_height:, :]
        
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
                width * 0.4 * 255,  # 40%
                width * 0.5 * 255,  # 50%
                width * 0.6 * 255   # 60%
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
            actual_y = height - roi_height + closest_stop_line_row
            
            # 차량으로부터의 거리 계산
            pixels_to_bottom = height - actual_y
            self.stop_line_distance = pixels_to_bottom * self.pixel_to_meter
            self.stop_line_detected = True
            self.last_valid_distance = self.stop_line_distance  # 유효한 거리 저장

            # print(f"Pixels from bottom: {pixels_to_bottom}, Distance: {self.stop_line_distance:.3f}m")
            if self.flag == False:
                self.prev_stop_line_num = self.stop_line_num
                self.stop_line_num += 1
                self.flag = True
                print(f"Stop line detected: {self.stop_line_num} times")

        else:
            if self.flag == True:
                self.flag = False
                
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

    def isStop(self):
        return self.stop_line_detected