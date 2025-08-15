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
    STRAIGHT = 1                  # 일반 차선 추종 주행
    FIRST_CORNER = 2        # 첫 번째 코너 진입
    SECOND_CORNER = 3       # 두 번째 코너 진입
    ENTRY_ROTARY = 4       # 로터리 진입
    EXIT_ROTARY = 5        # 로터리 탈출
    TRAFFICLIGHT = 6
    THIRD_CORNER = 7

class AutonomousDriving:
    def __init__(self):
        self.sliding = SlidingWindow.SlidingWindow()
        self.stop_line_detector = DetectStopLine.StopLine()
        self.traffic_sub = Traffic.Traffic()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.steer_msg = Float64()
        self.speed_msg = Float64()

        self.speed_msg.data = 1500 # Default speed
        self.mission_flag = Mission.TRAFFICLIGHT.value

        # self.mission_completed = [True, True, True, False, False]  # 각 미션 완료 여부를 저장하는 리스트
        self.mission_completed = [False] * 5  # 각 미션 완료 여부를 저장하는 리스트
        

    def action(self, width, height, bin_img):
        self.width = width
        self.height = height
        left_fit, right_fit = self.sliding.apply(bin_img)


        # self.detect_stop_line(bin_img)

        self.stop_line_detector.detect(width,height,bin_img)


        if self.mission_flag == Mission.STRAIGHT.value:
            print("STRAIGHT LINE")
            self.setSteeringinStraight(bin_img)
            if self.stop_line_detector.stop_line_num == 3:
                self.mission_flag = Mission.FIRST_CORNER.value
            elif self.stop_line_detector.stop_line_num == 4 and self.mission_completed[2] == True: # 두번째 코너 확인하기 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
                self.mission_flag = Mission.SECOND_CORNER.value
            elif self.stop_line_detector.stop_line_num == 5:
                self.mission_flag = Mission.ENTRY_ROTARY.value

        elif self.mission_flag == Mission.FIRST_CORNER.value:
            print("FIRST CORNER")
            self.speed_msg.data = 1000
            self.setSteeringinCurve(left_fit, right_fit)

            if left_fit is not None and right_fit is not None:
                self.setSteeringinCurve(left_fit, right_fit)
            else:
                self.setSteeringinStraight(bin_img) # 차선을 놓치면 직진으로 임시 보정
            
            # 코너 탈출 조건: 조향각이 거의 중앙으로 돌아오면 직선 주행으로 복귀
            if abs(self.steer_msg.data - 0.5) < 0.1:
                print("Out from First Corner")
                self.mission_flag = Mission.STRAIGHT.value
        
        elif self.mission_flag == Mission.SECOND_CORNER.value:  # 두번째 코너 확인하기 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            print("SECOND CORNER")
            try:
                if left_fit is not None and right_fit is not None:
                    self.setSteeringinCurve(left_fit, right_fit)
                else:
                    self.setSteeringinStraight(bin_img)
            except Exception as e:
                pass

        elif self.mission_flag == Mission.ENTRY_ROTARY.value:
            print("Enter to Rotary")
            
            # self.setSteeringinRotary(left_fit, right_fit)

            # self.rotary.run()
            
            # if self.rotary.is_finished:
            #     self.mission_flag = Mission.EXIT_ROTARY.value
            #     self.mission_completed[3] = True
        
        elif self.mission_flag == Mission.EXIT_ROTARY.value:
            print("Escape Rotary")
            self.mission_flag = Mission.TRAFFICLIGHT.value

        elif self.mission_flag == Mission.TRAFFICLIGHT.value:
            print("At Traffic Light")
            self.traffic_signal()
            if left_fit is not None and right_fit is not None:
                self.setSteeringinCurve(left_fit, right_fit)
            else:
                self.setSteeringinStraight(bin_img)

            if self.stop_line_detector.stop_line_num == 7:
                self.mission_completed[4] = True  # 5번째 미션 완료
                self.mission_flag = Mission.THIRD_CORNER.value  # 세번째 코너로 이동

        elif self.mission_flag == Mission.THIRD_CORNER.value:
            print("THIRD CORNER")
            try:
                if left_fit is not None and right_fit is not None:
                    self.setSteeringinCurve(left_fit, right_fit)
                else:
                    self.setSteeringinStraight(bin_img)
            except Exception as e:
                pass

            if self.stop_line_detector.stop_line_num == 8:
                self.mission_flag = Mission.STRAIGHT.value  # 세번째 코너 탈출 후 직진으로 복귀


        # try:
        #     if left_fit is not None and right_fit is not None:
        #         self.setSteeringinCurve(left_fit, right_fit)
        #     else:
        #         self.setSteeringinStraight(bin_img)
        # except Exception as e:
        #     pass
        
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)



    def traffic_signal(self):
        signal = self.traffic_sub.traffic_signal
        # print(self.stop_line_distance, "m away")
        if signal == 1:  # 빨간불
            if self.stop_line_detector.stop_line_detected and self.stop_line_detector.stop_line_distance <= 0.3:
                self.speed_msg.data = 0  # 정지선 0.3m 이내에서만 정지
            elif self.stop_line_detector.stop_line_detected and self.stop_line_detector.stop_line_distance <= 1.0:
                self.speed_msg.data = 300  # 정지선 근처에서 감속
            else:
                self.speed_msg.data = 500
        elif signal == 4:
            self.speed_msg.data = 300
        elif signal == 16 or signal == 33:
            self.speed_msg.data = 1000


    def setSteeringinStraight(self, bin_img):
        """
        [최종 개선 버전] 한쪽 차선만 보일 때도 가상 중앙선을 추정하여 주행 안정성을 높인 함수
        """
        histogram = np.sum(bin_img[self.height // 2:, :], axis=0)
        midpoint = self.width // 2

        # 1. 히스토그램을 좌우로 나누고 각 영역의 차선 피크(Peak)를 찾음
        left_half = histogram[:midpoint]
        right_half = histogram[midpoint:]
        leftx_base = np.argmax(left_half)
        rightx_base = np.argmax(right_half) + midpoint

        # 이상적인 차선 폭 (전체 이미지 폭의 70%로 가정, 필요시 튜닝)
        LANE_WIDTH_PIXELS = self.width * 0.7

        # --- 차선 검출 상태에 따른 중앙점 계산 ---
        
        # CASE 1: 양쪽 차선이 모두 잘 보일 때 (가장 이상적인 경우)
        if histogram[leftx_base] > 50 and histogram[rightx_base] > 50:
            lane_center = (leftx_base + rightx_base) / 2
            print("Status: Two lanes detected (1)")
        
        # ⭐️ CASE 2: 한쪽 차선만 보일 때 (핵심 개선 부분)
        else:
            # 오른쪽 차선만 보일 경우
            if histogram[rightx_base] > 50:
                # 오른쪽 차선 위치에서 차선 폭의 절반만큼 왼쪽으로 이동한 지점을 가상 중앙으로 설정
                lane_center = rightx_base - (LANE_WIDTH_PIXELS / 2)
                print("Status: Right lane only (2-Right)")
            # 왼쪽 차선만 보일 경우
            elif histogram[leftx_base] > 50:
                # 왼쪽 차선 위치에서 차선 폭의 절반만큼 오른쪽으로 이동한 지점을 가상 중앙으로 설정
                lane_center = leftx_base + (LANE_WIDTH_PIXELS / 2)
                print("Status: Left lane only (2-Left)")
            # CASE 3: 양쪽 차선이 모두 안 보일 경우
            else:
                lane_center = midpoint
                print("Status: No lanes detected (3)")
        
        # 최종 조향각 계산
        offset = (lane_center - midpoint) / midpoint
        steer = 0.5 + offset * 0.7
        self.steer_msg.data = np.clip(steer, 0.0, 1.0)

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

    def setSteeringForInnerRightTurn(self, right_fit):
        """
        [신규] 안쪽(오른쪽) 차선을 기준으로 일정한 거리를 유지하며 우회전하는 함수
        """
        # 튜닝 포인트: 차량 중심이 안쪽 차선에서 떨어져야 할 이상적인 거리 (픽셀 단위)
        # 이 값을 조절하여 회전 반경을 조절할 수 있습니다.
        DESIRED_OFFSET_PIXELS = 120  # 예시 값, 주행해보며 튜닝 필요

        # 차선 인식을 위한 기준 높이 (화면의 70% 지점)
        y_eval = int(self.height * 0.7)
        
        # 해당 높이에서 오른쪽 차선의 x좌표 계산
        right_x = np.polyval(right_fit, y_eval)
        
        # 목표 주행 지점 설정: 오른쪽 차선 위치 - 이상적인 오프셋
        target_x = right_x - DESIRED_OFFSET_PIXELS
        
        # 차량의 현재 중심(이미지 중앙)
        img_center = self.width / 2
        
        # 목표 지점과 현재 중심 사이의 오차 계산
        offset = target_x - img_center

        # 오차를 기반으로 조향각 계산 (게인 값 0.9로 약간 더 민감하게 설정)
        steer = 0.5 + (offset / img_center) * 0.9
        
        self.steer_msg.data = np.clip(steer, 0.0, 1.0)
        print(f"Inner Right Turn... Target X: {target_x:.1f}, Steer: {self.steer_msg.data:.2f}")
