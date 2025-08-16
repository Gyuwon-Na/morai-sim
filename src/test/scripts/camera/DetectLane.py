#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2, rospy
import numpy as np

class StopLane:
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
                self.estimated_distance = self.last_valid_distance - self.camera_offset
                if self.estimated_distance <= 0.0:
                    self.stop_line_distance = self.camera_offset  # 카메라 오프셋만큼 떨어져 있음
                else:
                    self.stop_line_distance = self.last_valid_distance
                self.stop_line_detected = True
                print(f"Stop line not visible but estimated vehicle front distance: {self.estimated_distance:.3f}m")
            else:
                self.stop_line_detected = False
                self.stop_line_distance = float('inf')
                self.estimated_distance = 0.0
                # print("No stop line detected")  

    def isStop(self):
        return self.stop_line_detected
    



class CurveLane:
    def __init__(self):
        """
        급격한 커브(코너)를 감지하는 클래스
        """
        self.curve_detected = False
        self.corner_point = None  # 코너의 좌표
        self.corner_angle = 0.0   # 코너의 각도

    def detect(self, bin_img, nwindows=9, threshold_angle=30):
        """
        이진 이미지에서 차선의 중심점들을 분석하여 급격한 코너를 감지합니다.

        Args:
            bin_img (numpy.ndarray): 조감도(Bird's-Eye View)로 변환된 이진 이미지
            nwindows (int): 탐색에 사용할 슬라이딩 윈도우의 수
            threshold_angle (float): 코너로 판단할 최소 각도 변화량 (단위: degree)
        """
        # 기본값 초기화
        self.curve_detected = False
        self.corner_point = None
        self.corner_angle = 0.0

        # 이미지의 높이와 너비
        height, width = bin_img.shape

        # 오른쪽 차선을 기준으로 탐색 (이미지 오른쪽 절반)
        histogram = np.sum(bin_img[height // 2:, width // 2:], axis=0)
        if np.max(histogram) == 0:
            return # 오른쪽 차선이 감지되지 않으면 종료

        rightx_base = np.argmax(histogram) + width // 2

        # 윈도우 설정
        window_height = height // nwindows
        
        # 0이 아닌 픽셀(차선)의 좌표
        nonzero = bin_img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        rightx_current = rightx_base
        margin = 100  # 윈도우 좌우 여백
        minpix = 50   # 윈도우 안의 최소 픽셀 수

        # 윈도우 별 중심점(centroid)을 저장할 리스트
        centroids = []
        for window in range(nwindows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # 현재 윈도우에 속하는 픽셀들의 인덱스 찾기
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            # 픽셀이 충분히 있으면 중심점 계산 후 다음 윈도우 위치 업데이트
            if len(good_right_inds) > minpix:
                avg_x = int(np.mean(nonzerox[good_right_inds]))
                avg_y = (win_y_low + win_y_high) // 2
                centroids.append((avg_x, avg_y))
                rightx_current = avg_x

        # 중심점이 3개 이상 있어야 각도 계산 가능
        if len(centroids) < 3:
            return

        # 중심점들을 이용해 각도 계산
        angles = []
        for i in range(len(centroids) - 1):
            x1, y1 = centroids[i]
            x2, y2 = centroids[i+1]
            # arctan2를 이용해 두 점 사이의 각도 계산 (y축이 반대이므로 y2-y1이 아닌 y1-y2)
            angle = np.arctan2(y1 - y2, x2 - x1)
            angles.append(np.degrees(angle))

        # 각도 변화량이 가장 큰 지점 탐지
        if len(angles) > 1:
            angle_diffs = np.abs(np.diff(angles))
            max_diff_idx = np.argmax(angle_diffs)
            max_angle_diff = angle_diffs[max_diff_idx]

            # 변화량이 임계값을 넘으면 코너로 판단
            if max_angle_diff > threshold_angle:
                self.curve_detected = True
                # 코너가 시작되는 지점의 좌표와 각도를 저장
                self.corner_point = centroids[max_diff_idx + 1]
                self.corner_angle = max_angle_diff
                # print(f"Sharp corner detected! Angle change: {self.corner_angle:.2f} degrees")

    def isCurve(self):
        """
        급격한 커브가 감지되었는지 여부를 반환합니다.

        Returns:
            bool: 커브가 감지되었으면 True, 아니면 False
        """
        return self.curve_detected
    

class YellowLaneDetector:
    def __init__(self):
        self.yellow_lane_detected = False
        # 노란색을 검출할 HSV 색상 범위 (튜닝 필요)
        self.lower_yellow = np.array([15, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])

    def detect(self, original_img):
        """
        원본 컬러 이미지에서 노란색 차선을 감지합니다.
        
        Args:
            original_img (numpy.ndarray): 왜곡 보정만 거친 원본 컬러 이미지
        """
        # 1. 이미지를 HSV 색상 공간으로 변환
        hsv_img = cv2.cvtColor(original_img, cv2.COLOR_BGR2HSV)
        
        # 2. 지정된 HSV 범위로 마스크 생성
        yellow_mask = cv2.inRange(hsv_img, self.lower_yellow, self.upper_yellow)
        
        # 3. 이미지 하단 영역만 ROI로 설정하여 노이즈 줄이기
        height = original_img.shape[0]
        roi = yellow_mask[height-150:, :] # 이미지 하단 150px 영역만 확인
        
        # 4. ROI 내에 노란색 픽셀이 일정 개수 이상이면 감지된 것으로 판단
        nonzero_count = np.count_nonzero(roi)
        # print(f"Yellow pixels detected: {nonzero_count}") # 디버깅용
        
        if nonzero_count > 100:  # 임계값, 튜닝 필요
            self.yellow_lane_detected = True
        else:
            self.yellow_lane_detected = False
            
        return self.yellow_lane_detected